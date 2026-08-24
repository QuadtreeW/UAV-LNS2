#pragma once
#include "common.h"
#include <vector>

#define NO_AGENT -1

class PathTable
{
public:
    int makespan = 0;
    vector< vector<int> > table; // actual path locations only; the value is the id of the agent
    vector<int> goals; // classic MAPF only: goal location -> permanent occupancy start time
    const vector<vector<int>>* safe_neighbors = nullptr; // UAV safety radius (null = classic MAPF)
    const PathTable* immutable_reservations = nullptr; // ACTIVE UAV paths; queried but never destroyed
    int reservation_start_timestep = 0; // temporary activation grace period
    bool agents_disappear_at_goal = false; // UAV lifecycle: occupy the arrival timestep, then leave the map
    void reset() { auto map_size = table.size(); table.clear(); table.resize(map_size); goals.assign(map_size, MAX_COST); makespan = 0; }
    void insertPath(int agent_id, const Path& path);
    void deletePath(int agent_id, const Path& path);
    bool constrained(int from, int to, int to_time) const;

    void get_agents(set<int>& conflicting_agents, int loc) const;
    void get_agents(set<int>& conflicting_agents, int neighbor_size, int loc) const;
    void getConflictingAgents(int agent_id, set<int>& conflicting_agents, int from, int to, int to_time) const;;
    int getHoldingTime(int location, int earliest_timestep) const;
    int getFutureNumOfCollisions(int loc, int time) const;
    int getMakespan() const
    {
        return immutable_reservations == nullptr ? makespan : max(makespan, immutable_reservations->getMakespan());
    }
    explicit PathTable(int map_size = 0) : table(map_size), goals(map_size, MAX_COST) {}
private:
    void markCell(int agent_id, int loc, int t);
    void unmarkCell(int agent_id, int loc, int t);
    bool isCellOccupied(int loc, int t) const;
    bool isGoalOccupied(int loc, int t) const;
};

class PathTableWC // with collisions
{
public:
    int makespan = 0;
    vector< vector< list<int> > > table; // this stores the paths, the value is the id of the agent
    vector<int> goals; // classic MAPF only: goal location -> permanent occupancy start time
    const vector<vector<int>>* safe_neighbors = nullptr; // query-side UAV safety radius
    int safe_radius_start_timestep = 0; // temporary grace period; exact vertex/edge conflicts still apply
    bool agents_disappear_at_goal = false; // UAV lifecycle: occupy the arrival timestep, then leave the map
    void reset() { auto map_size = table.size(); table.clear(); table.resize(map_size); goals.assign(map_size, MAX_COST); makespan = 0; }
    void insertPath(int agent_id, const Path& path);
    void insertPath(int agent_id);
    void deletePath(int agent_id);
    const Path* getPath(int agent_id) const {return paths[agent_id]; }
    int getFutureNumOfCollisions(int loc, int time) const; // return #collisions when the agent waiting at loc starting from time forever
    int getNumOfCollisions(int from, int to, int to_time) const;
    bool hasCollisions(int from, int to, int to_time) const;
    bool hasEdgeCollisions(int from, int to, int to_time) const;
    int getLastCollisionTimestep(int location) const;
    void getAgentsAt(int location, int timestep, set<int>& occupying_agents) const;
    // return the agent who reaches its target target_location before timestep earliest_timestep
    int getAgentWithTarget(int target_location, int latest_timestep) const;
    void clear();
    explicit PathTableWC(int map_size = 0, int num_of_agents = 0) : table(map_size), goals(map_size, MAX_COST),
        paths(num_of_agents, nullptr) {}
private:
    vector<const Path*> paths;
};
