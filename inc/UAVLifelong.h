#pragma once

#include "LNS.h"
#include <random>

enum class FlightStatus
{
    PENDING,
    READY,
    ACTIVE,
    COMPLETED,
    DEFERRED
};

struct FlightLeg
{
    int flight_id = -1;
    int aircraft_id = -1;
    int leg_id = -1;
    int origin = -1;
    int destination = -1;
    int release_time = -1;
    int next_attempt_time = -1;
    int takeoff_time = -1;
    int arrival_time = -1;
    int retries = 0;
    FlightStatus status = FlightStatus::PENDING;
    string reason;
    Path path;
};

struct LifelongOptions
{
    int horizon = 2000;
    int legs_per_aircraft = 3;
    int initial_release_max = 0;
    int rest_min = 10;
    int rest_max = 30;
    int retry_interval = 5;
    int max_retries = 100;
    int max_wait = 600;
    int max_no_progress = 600;
    double planning_time = 10;
    int seed = 0;
    bool paired_tasks = false;
    int screen = 1;
    bool use_sipp = true;
    int neighbor_size = 8;
    bool use_init_lns = true;
    string init_algo = "PP";
    string replan_algo = "PP";
    string destroy_strategy = "Adaptive";
    string init_destroy_strategy = "Adaptive";
    PIBTPPS_option pibt_option;
};

class UAVLifelongSystem
{
public:
    UAVLifelongSystem(const Instance& instance, const LifelongOptions& options);
    bool run();
    bool validate() const;
    void writeFlights(const string& file_name) const;
    void writePaths(const string& file_name) const;

private:
    struct Aircraft
    {
        int id = -1;
        int current_location = -1;
        int generated_legs = 0;
        int active_leg = -1;
        bool finished = false;
    };

    const Instance& base_instance;
    Instance planning_instance;
    LifelongOptions options;
    vector<int> airports;
    vector<Aircraft> aircraft;
    vector<FlightLeg> flights;
    std::mt19937 rng;
    int next_flight_id = 0;
    int last_progress_time = 0;
    int final_time = 0;

    int sampleInclusive(int lower, int upper);
    int chooseDestination(int origin);
    int createFlight(int aircraft_id, int destination, int release_time);
    vector<Path> buildActiveReservations(int global_time) const;
    bool startIsSafe(int origin, const vector<Path>& reservations) const;
    bool pathIsSafe(const Path& candidate, const vector<Path>& reservations) const;
    bool planFlight(FlightLeg& flight, const vector<Path>& reservations);
    void processArrivals(int global_time);
    void processReleases(int global_time);
    bool processReady(int global_time);
    int nextEventTime(int current_time) const;
    void deferFlight(FlightLeg& flight, const string& reason);
    static const char* statusName(FlightStatus status);
};
