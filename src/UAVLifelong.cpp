#include "UAVLifelong.h"
#include <algorithm>
#include <cmath>
#include <numeric>

UAVLifelongSystem::UAVLifelongSystem(const Instance& instance, const LifelongOptions& options) :
    base_instance(instance), planning_instance(instance), options(options), rng(options.seed)
{
    auto starts = instance.getStarts();
    auto goals = instance.getGoals();
    airports = starts;
    airports.insert(airports.end(), goals.begin(), goals.end());
    std::sort(airports.begin(), airports.end());
    airports.erase(std::unique(airports.begin(), airports.end()), airports.end());

    aircraft.resize(starts.size());
    for (int i = 0; i < (int)starts.size(); i++)
    {
        aircraft[i].id = i;
        aircraft[i].current_location = starts[i];
        int destination = goals[i];
        if (destination == starts[i])
            destination = chooseDestination(starts[i]);
        createFlight(i, destination, sampleInclusive(0, options.initial_release_max));
    }
}

int UAVLifelongSystem::sampleInclusive(int lower, int upper)
{
    if (upper <= lower)
        return lower;
    std::uniform_int_distribution<int> distribution(lower, upper);
    return distribution(rng);
}

int UAVLifelongSystem::chooseDestination(int origin)
{
    vector<int> candidates;
    candidates.reserve(airports.size());
    for (int airport : airports)
    {
        if (airport != origin)
            candidates.push_back(airport);
    }
    if (candidates.empty())
        return origin;
    return candidates[sampleInclusive(0, (int)candidates.size() - 1)];
}

int UAVLifelongSystem::createFlight(int aircraft_id, int destination, int release_time)
{
    auto& plane = aircraft[aircraft_id];
    FlightLeg flight;
    flight.flight_id = next_flight_id++;
    flight.aircraft_id = aircraft_id;
    flight.leg_id = plane.generated_legs++;
    flight.origin = plane.current_location;
    flight.destination = destination;
    flight.release_time = release_time;
    flight.next_attempt_time = release_time;
    flights.push_back(std::move(flight));
    return (int)flights.size() - 1;
}

vector<Path> UAVLifelongSystem::buildActiveReservations(int global_time) const
{
    vector<Path> reservations;
    for (const auto& flight : flights)
    {
        // A flight still occupies its final cell at arrival_time. Arrivals are
        // processed before READY admissions at the same event, so retain the
        // final one-cell suffix even after the status becomes COMPLETED.
        bool occupies_now = flight.status == FlightStatus::ACTIVE ||
                            (flight.status == FlightStatus::COMPLETED &&
                             flight.arrival_time == global_time);
        if (!occupies_now || flight.takeoff_time > global_time || flight.arrival_time < global_time)
            continue;
        int offset = global_time - flight.takeoff_time;
        Path suffix(flight.path.begin() + offset, flight.path.end());
        reservations.push_back(std::move(suffix));
    }
    return reservations;
}

bool UAVLifelongSystem::startIsSafe(int origin, const vector<Path>& reservations) const
{
    for (const auto& path : reservations)
    {
        if (path.empty())
            continue;
        int occupied = path.front().location;
        if (occupied == origin)
            return false;
        if (base_instance.hasSafeRadius())
        {
            const auto& neighbors = base_instance.getSafeNeighbors(origin);
            if (std::find(neighbors.begin(), neighbors.end(), occupied) != neighbors.end())
                return false;
        }
    }
    return true;
}

bool UAVLifelongSystem::pathIsSafe(const Path& candidate, const vector<Path>& reservations) const
{
    for (const auto& reserved : reservations)
    {
        int overlap = min(candidate.size(), reserved.size());
        for (int t = 0; t < overlap; t++)
        {
            int candidate_location = candidate[t].location;
            int reserved_location = reserved[t].location;
            if (candidate_location == reserved_location)
                return false;

            if (base_instance.hasSafeRadius())
            {
                const auto& neighbors = base_instance.getSafeNeighbors(candidate_location);
                if (std::find(neighbors.begin(), neighbors.end(), reserved_location) != neighbors.end())
                    return false;
            }

            if (t > 0 && candidate[t - 1].location == reserved_location &&
                reserved[t - 1].location == candidate_location)
            {
                return false;
            }
        }
    }
    return true;
}

bool UAVLifelongSystem::planFlight(FlightLeg& flight, const vector<Path>& reservations)
{
    planning_instance.setAgents({flight.origin}, {flight.destination});
    LNS lns(planning_instance, options.planning_time,
            options.init_algo, options.replan_algo, options.destroy_strategy,
            options.neighbor_size, 0, options.use_init_lns,
            options.init_destroy_strategy, options.use_sipp,
            options.screen >= 2 ? options.screen - 1 : 0, options.pibt_option,
            &reservations, 0);
    if (!lns.run() || lns.agents.empty() || lns.agents.front().path.empty())
        return false;

    const Path& path = lns.agents.front().path;
    // Waiting before the first move is ground delay, not an airborne trajectory.
    if (path.size() < 2 || path[1].location == path[0].location)
        return false;
    // Defense in depth: a candidate must never become ACTIVE unless it passes
    // an independent check against every time-aligned ACTIVE suffix.
    if (!pathIsSafe(path, reservations))
    {
        if (options.screen >= 1)
        {
            cout << "[lifelong] rejected flight " << flight.flight_id
                 << " because candidate path conflicts with an ACTIVE reservation" << endl;
        }
        return false;
    }
    flight.path = path;
    return true;
}

void UAVLifelongSystem::processArrivals(int global_time)
{
    for (auto& plane : aircraft)
    {
        if (plane.active_leg < 0)
            continue;
        auto& flight = flights[plane.active_leg];
        if (flight.status != FlightStatus::ACTIVE || flight.arrival_time != global_time)
            continue;

        flight.status = FlightStatus::COMPLETED;
        plane.current_location = flight.destination;
        plane.active_leg = -1;
        last_progress_time = global_time;
        if (plane.generated_legs >= options.legs_per_aircraft)
        {
            plane.finished = true;
            continue;
        }
        // Initial releases are drawn in aircraft order in the constructor.
        // Key each later task independently of the order in which flights arrive.
        if (options.paired_tasks)
        {
            std::seed_seq task_key{options.seed, plane.id, plane.generated_legs};
            rng.seed(task_key);
        }
        int rest = sampleInclusive(options.rest_min, options.rest_max);
        int destination = chooseDestination(plane.current_location);
        createFlight(plane.id, destination, global_time + rest);
    }
}

void UAVLifelongSystem::processReleases(int global_time)
{
    for (auto& flight : flights)
    {
        if (flight.status == FlightStatus::PENDING && flight.release_time <= global_time)
        {
            flight.status = FlightStatus::READY;
            flight.next_attempt_time = global_time;
            last_progress_time = global_time;
        }
    }
}

void UAVLifelongSystem::deferFlight(FlightLeg& flight, const string& reason)
{
    flight.status = FlightStatus::DEFERRED;
    flight.reason = reason;
    aircraft[flight.aircraft_id].finished = true;
}

bool UAVLifelongSystem::processReady(int global_time)
{
    vector<int> ready;
    for (int i = 0; i < (int)flights.size(); i++)
    {
        if (flights[i].status == FlightStatus::READY && flights[i].next_attempt_time <= global_time)
            ready.push_back(i);
    }

    std::sort(ready.begin(), ready.end(), [&](int lhs, int rhs)
    {
        const auto& a = flights[lhs];
        const auto& b = flights[rhs];
        int wait_a = global_time - a.release_time;
        int wait_b = global_time - b.release_time;
        if (wait_a != wait_b)
            return wait_a > wait_b;
        int distance_a = planning_instance.getHeuristicTable(a.destination)[a.origin];
        int distance_b = planning_instance.getHeuristicTable(b.destination)[b.origin];
        if (distance_a != distance_b)
            return distance_a < distance_b;
        return a.flight_id < b.flight_id;
    });

    bool admitted = false;
    for (int index : ready)
    {
        auto& flight = flights[index];
        if (global_time - flight.release_time > options.max_wait)
        {
            deferFlight(flight, "MAX_WAIT");
            continue;
        }
        if (flight.retries >= options.max_retries)
        {
            deferFlight(flight, "MAX_RETRIES");
            continue;
        }

        vector<Path> reservations = buildActiveReservations(global_time);
        bool success = startIsSafe(flight.origin, reservations) && planFlight(flight, reservations);
        if (!success)
        {
            flight.retries++;
            flight.next_attempt_time = global_time + options.retry_interval;
            continue;
        }

        flight.status = FlightStatus::ACTIVE;
        flight.takeoff_time = global_time;
        flight.arrival_time = global_time + (int)flight.path.size() - 1;
        aircraft[flight.aircraft_id].active_leg = index;
        last_progress_time = global_time;
        admitted = true;
        if (options.screen >= 1)
        {
            cout << "[lifelong] flight " << flight.flight_id << " aircraft " << flight.aircraft_id
                 << " leg " << flight.leg_id << " takeoff=" << flight.takeoff_time
                 << " arrival=" << flight.arrival_time << " delay="
                 << flight.takeoff_time - flight.release_time << endl;
        }
    }
    return admitted;
}

int UAVLifelongSystem::nextEventTime(int current_time) const
{
    int next = MAX_TIMESTEP;
    for (const auto& flight : flights)
    {
        if (flight.status == FlightStatus::PENDING && flight.release_time > current_time)
            next = min(next, flight.release_time);
        else if (flight.status == FlightStatus::READY && flight.next_attempt_time > current_time)
            next = min(next, flight.next_attempt_time);
        else if (flight.status == FlightStatus::ACTIVE && flight.arrival_time > current_time)
            next = min(next, flight.arrival_time);
    }
    return next;
}

bool UAVLifelongSystem::run()
{
    if (!base_instance.hasSafeRadius())
    {
        cerr << "Lifelong UAV mode requires --safeRadius > 0" << endl;
        return false;
    }
    if (options.legs_per_aircraft <= 0 || options.retry_interval <= 0 || options.horizon < 0)
    {
        cerr << "Invalid lifelong configuration" << endl;
        return false;
    }

    int current_time = -1;
    while (true)
    {
        int event_time = nextEventTime(current_time);
        if (event_time == MAX_TIMESTEP || event_time > options.horizon)
            break;
        current_time = event_time;
        final_time = current_time;
        processArrivals(current_time);
        processReleases(current_time);
        processReady(current_time);

        bool has_active = false;
        bool has_ready = false;
        for (const auto& flight : flights)
        {
            has_active = has_active || flight.status == FlightStatus::ACTIVE;
            has_ready = has_ready || flight.status == FlightStatus::READY;
        }
        if (!has_active && has_ready && current_time - last_progress_time >= options.max_no_progress)
        {
            for (auto& flight : flights)
            {
                if (flight.status == FlightStatus::READY)
                    deferFlight(flight, "NO_PROGRESS");
            }
        }
    }

    final_time = options.horizon;
    for (auto& flight : flights)
    {
        if (flight.status == FlightStatus::PENDING || flight.status == FlightStatus::READY)
            deferFlight(flight, "HORIZON");
    }

    int completed = 0, active = 0, deferred = 0;
    for (const auto& flight : flights)
    {
        completed += flight.status == FlightStatus::COMPLETED;
        active += flight.status == FlightStatus::ACTIVE;
        deferred += flight.status == FlightStatus::DEFERRED;
    }
    cout << "UAVLifelong: flights=" << flights.size() << ", completed=" << completed
         << ", active=" << active << ", deferred=" << deferred
         << ", horizon=" << options.horizon << endl;
    return validate();
}

bool UAVLifelongSystem::validate() const
{
    for (const auto& flight : flights)
    {
        if (flight.takeoff_time < 0)
            continue;
        if (flight.path.empty() || flight.path.front().location != flight.origin ||
            flight.path.back().location != flight.destination ||
            flight.arrival_time != flight.takeoff_time + (int)flight.path.size() - 1)
        {
            cerr << "Invalid path metadata for flight " << flight.flight_id << endl;
            return false;
        }
        for (int t = 1; t < (int)flight.path.size(); t++)
        {
            if (!base_instance.validMove(flight.path[t - 1].location, flight.path[t].location))
            {
                cerr << "Invalid move in flight " << flight.flight_id << endl;
                return false;
            }
        }
    }

    for (int i = 0; i < (int)flights.size(); i++)
    {
        if (flights[i].takeoff_time < 0)
            continue;
        for (int j = i + 1; j < (int)flights.size(); j++)
        {
            if (flights[j].takeoff_time < 0)
                continue;
            int begin = max(flights[i].takeoff_time, flights[j].takeoff_time);
            int end = min(flights[i].arrival_time, flights[j].arrival_time);
            for (int time = begin; time <= end; time++)
            {
                int i_index = time - flights[i].takeoff_time;
                int j_index = time - flights[j].takeoff_time;
                int loc_i = flights[i].path[i_index].location;
                int loc_j = flights[j].path[j_index].location;
                if (loc_i == loc_j)
                {
                    cerr << "Vertex conflict between flights " << flights[i].flight_id << " and "
                         << flights[j].flight_id << " at global timestep " << time << endl;
                    return false;
                }
                const auto& safe = base_instance.getSafeNeighbors(loc_i);
                if (std::find(safe.begin(), safe.end(), loc_j) != safe.end())
                {
                    cerr << "Safe-radius conflict between flights " << flights[i].flight_id << " and "
                         << flights[j].flight_id << " at global timestep " << time << endl;
                    return false;
                }
                if (time > begin && i_index > 0 && j_index > 0 &&
                    flights[i].path[i_index - 1].location == loc_j &&
                    flights[j].path[j_index - 1].location == loc_i)
                {
                    cerr << "Edge conflict between flights " << flights[i].flight_id << " and "
                         << flights[j].flight_id << " at global timestep " << time << endl;
                    return false;
                }
            }
        }
    }
    return true;
}

const char* UAVLifelongSystem::statusName(FlightStatus status)
{
    switch (status)
    {
        case FlightStatus::PENDING: return "PENDING";
        case FlightStatus::READY: return "READY";
        case FlightStatus::ACTIVE: return "ACTIVE";
        case FlightStatus::COMPLETED: return "COMPLETED";
        case FlightStatus::DEFERRED: return "DEFERRED";
    }
    return "UNKNOWN";
}

void UAVLifelongSystem::writeFlights(const string& file_name) const
{
    string name = file_name;
    if (name.size() < 4 || name.substr(name.size() - 4) != ".csv")
        name += "-lifelong.csv";
    ofstream output(name);
    output << "flight_id,aircraft_id,leg_id,origin_row,origin_col,destination_row,destination_col,"
              "release_time,takeoff_time,arrival_time,departure_delay,path_length,retries,status,reason\n";
    for (const auto& flight : flights)
    {
        output << flight.flight_id << ',' << flight.aircraft_id << ',' << flight.leg_id << ','
               << base_instance.getRowCoordinate(flight.origin) << ','
               << base_instance.getColCoordinate(flight.origin) << ','
               << base_instance.getRowCoordinate(flight.destination) << ','
               << base_instance.getColCoordinate(flight.destination) << ','
               << flight.release_time << ',' << flight.takeoff_time << ',' << flight.arrival_time << ','
               << (flight.takeoff_time < 0 ? -1 : flight.takeoff_time - flight.release_time) << ','
               << flight.path.size() << ',' << flight.retries << ',' << statusName(flight.status) << ','
               << flight.reason << '\n';
    }
}

void UAVLifelongSystem::writePaths(const string& file_name) const
{
    ofstream output(file_name);
    for (const auto& flight : flights)
    {
        if (flight.path.empty())
            continue;
        output << "Flight " << flight.flight_id << " aircraft=" << flight.aircraft_id
               << " leg=" << flight.leg_id << " release=" << flight.release_time
               << " takeoff=" << flight.takeoff_time << " arrival=" << flight.arrival_time << ':';
        for (const auto& state : flight.path)
        {
            output << '(' << base_instance.getRowCoordinate(state.location) << ','
                   << base_instance.getColCoordinate(state.location) << ")->";
        }
        output << '\n';
    }
}
