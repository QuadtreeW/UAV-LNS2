#include "PathTable.h"

// Store actual agent locations only. Safe-radius expansion is performed once,
// on the query side, so the configured radius is not applied twice.
void PathTable::markCell(int agent_id, int loc, int t)
{
    if (table[loc].size() <= t)
        table[loc].resize(t + 1, NO_AGENT);
    table[loc][t] = agent_id;
}

void PathTable::unmarkCell(int agent_id, int loc, int t)
{
    if (table[loc].size() > t && table[loc][t] == agent_id)
        table[loc][t] = NO_AGENT;
}

// Query the actual cell and, in safe mode, its Euclidean safety neighborhood.
bool PathTable::isCellOccupied(int loc, int t) const
{
    if (immutable_reservations != nullptr && immutable_reservations->isCellOccupied(loc, t))
        return true;
    if (!table.empty())
    {
        if (table[loc].size() > t && table[loc][t] != NO_AGENT)
            return true;
        if (safe_neighbors)
        {
            for (int nb : (*safe_neighbors)[loc])
            {
                if (table[nb].size() > t && table[nb][t] != NO_AGENT)
                    return true;
            }
        }
    }
    return false;
}

bool PathTable::isGoalOccupied(int loc, int t) const
{
    if (immutable_reservations != nullptr && immutable_reservations->isGoalOccupied(loc, t))
        return true;
    if (agents_disappear_at_goal)
        return false;
    if (goals[loc] <= t)
        return true;
    if (safe_neighbors)
    {
        for (int nb : (*safe_neighbors)[loc])
        {
            if (goals[nb] <= t)
                return true;
        }
    }
    return false;
}

void PathTable::insertPath(int agent_id, const Path& path)
{
    if (path.empty())
        return;
    for (int t = 0; t < (int)path.size(); t++)
    {
        markCell(agent_id, path[t].location, t);
    }
    if (!agents_disappear_at_goal)
    {
        assert(goals[path.back().location] == MAX_TIMESTEP);
        goals[path.back().location] = (int) path.size() - 1;
    }
    makespan = max(makespan, (int) path.size() - 1);
}

void PathTable::deletePath(int agent_id, const Path& path)
{
    if (path.empty())
        return;
    for (int t = 0; t < (int)path.size(); t++)
    {
        unmarkCell(agent_id, path[t].location, t);
    }
    if (!agents_disappear_at_goal)
        goals[path.back().location] = MAX_TIMESTEP;
    if (makespan == (int) path.size() - 1) // re-compute makespan
    {
        makespan = 0;
        if (!agents_disappear_at_goal)
        {
            for (int time : goals)
            {
                if (time < MAX_TIMESTEP && time > makespan)
                    makespan = time;
            }
        }
        else
        {
            // With release-on-arrival there is no goal table to recover the
            // makespan from, so use the last real occupied timestep.
            for (const auto& timeline : table)
            {
                for (int t = (int)timeline.size() - 1; t > makespan; t--)
                {
                    if (timeline[t] != NO_AGENT)
                    {
                        makespan = t;
                        break;
                    }
                }
            }
        }
    }
}

bool PathTable::constrained(int from, int to, int to_time) const
{
    if (immutable_reservations != nullptr && immutable_reservations->constrained(from, to, to_time))
        return true;
    if (to_time < reservation_start_timestep)
        return false;
    if (isCellOccupied(to, to_time))
        return true; // vertex conflict (includes neighbor conflicts in safe mode)
    if (!table.empty())
    {
        if (!safe_neighbors)
        {
            if (table[to].size() >= to_time && table[from].size() > to_time && !table[to].empty() &&
                     table[to][to_time - 1] != NO_AGENT && table[from][to_time] == table[to][to_time - 1])
                return true; // edge conflict
        }
        else
        {
            // In safe mode, edge conflicts are already covered by vertex conflicts on neighbors:
            // moving from A→B at time t means B is occupied at t (checked above),
            // and A's neighbor cells conflict with any agent moving into A or its neighbors.
            // The safe-radius vertex check already handles this because neighbors of B are marked,
            // so any agent in neighbor of B at time t is a vertex conflict.
            // We keep the edge check for safety compatibility:
            if (table[to].size() >= to_time && table[from].size() > to_time && !table[to].empty() &&
                     table[to][to_time - 1] != NO_AGENT && table[from][to_time] == table[to][to_time - 1])
                return true;
        }
    }
    if (!goals.empty())
    {
        if (isGoalOccupied(to, to_time))
            return true; // target conflict, including one query-side radius expansion
    }
    return false;
}

void PathTable::getConflictingAgents(int agent_id, set<int>& conflicting_agents, int from, int to, int to_time) const
{
    if (table.empty())
        return;
    if (table[to].size() > to_time && table[to][to_time] != NO_AGENT)
        conflicting_agents.insert(table[to][to_time]); // vertex conflict
    if (safe_neighbors)
    {
        for (int nb : (*safe_neighbors)[to])
        {
            if (table[nb].size() > to_time && table[nb][to_time] != NO_AGENT)
                conflicting_agents.insert(table[nb][to_time]);
        }
    }
    if (table[to].size() >= to_time && table[from].size() > to_time &&
        table[to][to_time - 1] != NO_AGENT && table[from][to_time] == table[to][to_time - 1])
        conflicting_agents.insert(table[from][to_time]); // edge conflict
    // TODO: collect target conflicts as well.
}

void PathTable::get_agents(set<int>& conflicting_agents, int loc) const
{
    if (loc < 0)
        return;
    for (auto agent : table[loc])
    {
        if (agent >= 0)
            conflicting_agents.insert(agent);
    }
    if (safe_neighbors)
    {
        for (int nb : (*safe_neighbors)[loc])
        {
            for (auto agent : table[nb])
            {
                if (agent >= 0)
                    conflicting_agents.insert(agent);
            }
        }
    }
}

void PathTable::get_agents(set<int>& conflicting_agents, int neighbor_size, int loc) const
{
    if (loc < 0)
        return;

    vector<int> cells = {loc};
    if (safe_neighbors)
        cells.insert(cells.end(), (*safe_neighbors)[loc].begin(), (*safe_neighbors)[loc].end());

    int t_max = -1;
    for (int cell : cells)
    {
        int last = (int)table[cell].size() - 1;
        while (last >= 0 && table[cell][last] == NO_AGENT)
            last--;
        t_max = max(t_max, last);
    }
    if (t_max <= 0)
        return;

    int t0 = rand() % t_max;
    auto collect_at_time = [&](int t)
    {
        for (int cell : cells)
        {
            if ((int)table[cell].size() > t && table[cell][t] != NO_AGENT)
            {
                conflicting_agents.insert(table[cell][t]);
                if ((int)conflicting_agents.size() == neighbor_size)
                    return true;
            }
        }
        return false;
    };

    if (collect_at_time(t0))
        return;
    int delta = 1;
    while (t0 - delta >= 0 || t0 + delta <= t_max)
    {
        if (t0 - delta >= 0 && collect_at_time(t0 - delta))
            return;
        if (t0 + delta <= t_max && collect_at_time(t0 + delta))
            return;
        delta++;
    }
}

// get the holding time after the earliest_timestep for a location
int PathTable::getHoldingTime(int location, int earliest_timestep = 0) const
{
    int immutable_holding_time = earliest_timestep;
    if (immutable_reservations != nullptr)
        immutable_holding_time = immutable_reservations->getHoldingTime(location, earliest_timestep);
    // A UAV vanishes immediately after occupying its final path entry. It only
    // needs a safe arrival timestep; it does not need to hold the goal forever.
    if (agents_disappear_at_goal || table.empty())
        return immutable_holding_time;

    int rst = immutable_holding_time;
    auto update_holding_time = [&](int cell)
    {
        int cell_holding_time = (int)table[cell].size();
        while (cell_holding_time > earliest_timestep &&
               table[cell][cell_holding_time - 1] == NO_AGENT)
        {
            cell_holding_time--;
        }
        rst = max(rst, cell_holding_time);
    };

    update_holding_time(location);
    if (safe_neighbors)
    {
        for (int nb : (*safe_neighbors)[location])
            update_holding_time(nb);
    }
    return rst;
}

void PathTableWC::insertPath(int agent_id, const Path& path)
{
    paths[agent_id] = &path;
    if (path.empty())
        return;
    for (int t = 0; t < (int)path.size(); t++)
    {
        if (table[path[t].location].size() <= t)
            table[path[t].location].resize(t + 1);
        table[path[t].location][t].push_back(agent_id);
    }
    if (!agents_disappear_at_goal)
    {
        assert(goals[path.back().location] == MAX_TIMESTEP);
        goals[path.back().location] = (int) path.size() - 1;
    }
    makespan = max(makespan, (int) path.size() - 1);
}
void PathTableWC::insertPath(int agent_id)
{
    assert(paths[agent_id] != nullptr);
    insertPath(agent_id, *paths[agent_id]);
}
void PathTableWC::deletePath(int agent_id)
{
    const Path & path = *paths[agent_id];
    if (path.empty())
        return;
    for (int t = 0; t < (int)path.size(); t++)
    {
        assert(table[path[t].location].size() > t &&
               std::find (table[path[t].location][t].begin(), table[path[t].location][t].end(), agent_id)
               != table[path[t].location][t].end());
        table[path[t].location][t].remove(agent_id);
    }
    if (!agents_disappear_at_goal)
        goals[path.back().location] = MAX_TIMESTEP;
    if (makespan == (int) path.size() - 1) // re-compute makespan
    {
        makespan = 0;
        if (!agents_disappear_at_goal)
        {
            for (int time : goals)
            {
                if (time < MAX_TIMESTEP && time > makespan)
                    makespan = time;
            }
        }
        else
        {
            for (const auto& timeline : table)
            {
                for (int t = (int)timeline.size() - 1; t > makespan; t--)
                {
                    if (!timeline[t].empty())
                    {
                        makespan = t;
                        break;
                    }
                }
            }
        }

    }
}

void PathTableWC::getAgentsAt(int location, int timestep, set<int>& occupying_agents) const
{
    auto collect = [&](int cell)
    {
        if ((int)table[cell].size() <= timestep)
            return;
        occupying_agents.insert(table[cell][timestep].begin(), table[cell][timestep].end());
    };

    collect(location);
    if (safe_neighbors && timestep >= safe_radius_start_timestep)
    {
        for (int nb : (*safe_neighbors)[location])
            collect(nb);
    }
}

int PathTableWC::getFutureNumOfCollisions(int loc, int time) const
{
    if (agents_disappear_at_goal)
        return 0;
    assert(goals[loc] == MAX_TIMESTEP);
    int rst = 0;
    if (!table.empty())
    {
        auto count_future_at = [&](int cell)
        {
            for (int t = time + 1; t < (int)table[cell].size(); t++)
                rst += (int)table[cell][t].size();
        };

        count_future_at(loc);
        if (safe_neighbors)
        {
            for (int nb : (*safe_neighbors)[loc])
            {
                for (int t = max(time + 1, safe_radius_start_timestep);
                     t < (int)table[nb].size(); t++)
                {
                    rst += (int)table[nb][t].size();
                }
            }
        }
    }
    return rst;
}

int PathTableWC::getNumOfCollisions(int from, int to, int to_time) const
{
    int rst = 0;
    if (!table.empty())
    {
        auto count_at = [&](int cell)
        {
            if ((int)table[cell].size() > to_time)
                rst += (int)table[cell][to_time].size();
        };
        count_at(to);
        if (safe_neighbors && to_time >= safe_radius_start_timestep)
        {
            for (int nb : (*safe_neighbors)[to])
                count_at(nb);
        }
        if (from != to && table[to].size() >= to_time && table[from].size() > to_time)
        {
            for (auto a1 : table[to][to_time - 1])
            {
                for (auto a2: table[from][to_time])
                {
                    if (a1 == a2)
                        rst++; // edge conflict
                }
            }
        }
    }
    if (!agents_disappear_at_goal && !goals.empty())
    {
        if (goals[to] < to_time)
            rst++; // target conflict
    }
    return rst;
}
bool PathTableWC::hasCollisions(int from, int to, int to_time) const
{
    if (!table.empty())
    {
        auto occupied_at = [&](int cell)
        {
            return (int)table[cell].size() > to_time && !table[cell][to_time].empty();
        };
        if (occupied_at(to))
            return true; // vertex or safe-radius conflict
        if (safe_neighbors && to_time >= safe_radius_start_timestep)
        {
            for (int nb : (*safe_neighbors)[to])
            {
                if (occupied_at(nb))
                    return true;
            }
        }
        if (from != to && table[to].size() >= to_time && table[from].size() > to_time)
        {
            for (auto a1 : table[to][to_time - 1])
            {
                for (auto a2: table[from][to_time])
                {
                    if (a1 == a2)
                        return true; // edge conflict
                }
            }
        }
    }
    if (!agents_disappear_at_goal && !goals.empty())
    {
        if (goals[to] < to_time)
            return true; // target conflict
    }
    return false;
}
bool PathTableWC::hasEdgeCollisions(int from, int to, int to_time) const
{
    if (!table.empty() && from != to && table[to].size() >= to_time && table[from].size() > to_time)
    {
        for (auto a1 : table[to][to_time - 1])
        {
            for (auto a2: table[from][to_time])
            {
                if (a1 == a2)
                    return true; // edge conflict
            }
        }
    }
    return false;
}

int PathTableWC::getAgentWithTarget(int target_location, int latest_timestep) const
{
    if (agents_disappear_at_goal)
        return -1;
    if (table.empty() or goals.empty() or goals[target_location] > latest_timestep)
        return -1;
    for (auto id : table[target_location][goals[target_location]]) // look at all agents at the goal time
    {
        if (paths[id]->back().location == target_location) // if agent id's goal is to, then this is the agent we want
        {
            return id;
        }
    }
    assert(false); // this should never happen
    return -1;
}

int PathTableWC::getLastCollisionTimestep(int location) const
{
    // Future traffic through the same airport does not constrain an aircraft
    // that leaves the map at its arrival timestep.
    if (agents_disappear_at_goal || table.empty())
        return -1;

    auto last_occupied_at = [&](int cell)
    {
        for (int t = (int)table[cell].size() - 1; t >= 0; t--)
        {
            if (!table[cell][t].empty())
                return t;
        }
        return -1;
    };

    int last_timestep = last_occupied_at(location);
    if (safe_neighbors)
    {
        for (int nb : (*safe_neighbors)[location])
        {
            int neighbor_last_timestep = last_occupied_at(nb);
            if (neighbor_last_timestep >= safe_radius_start_timestep)
                last_timestep = max(last_timestep, neighbor_last_timestep);
        }
    }
    return last_timestep;
}

void PathTableWC::clear()
{
    table.clear();
    goals.clear();
    paths.clear();
}
