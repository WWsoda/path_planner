//
// Created by wwsoda
//

#ifndef PATH_PLANNER__PATHPLANNER_HPP_
#define PATH_PLANNER__PATHPLANNER_HPP_

#include <string>
#include <vector>
#include <memory>
#include "grid_map_core/grid_map_core.hpp"

namespace PathPlannerNS
{
    class ReferencePath;
    class State;
    class SlState;
    class Map;
    class CollisionChecker;
    class VehicleState;
    class PathPlanner
    {
    public:
        PathPlanner() = delete;
        PathPlanner(const State &start_state,
                    const State &end_state,
                    const grid_map::GridMap &map);
        PathPlanner(consty PathPlanner &planner) = delete;
        PathPlanner &operator=(const PathPlanner &planner) = delete;

        // call this to get the path.
        bool solve(const std::vector<State> &reference_points, std::vector<SlState> *final_path);

        // only for visualization purpose.
        const ReferencePath &getReferencePath() const;

    private:
        // core fucntion.
        bool optimizePath(std::vector<SlState> *final_path);

        // divide smoothed path into segments.
        bool processReferencePath();

        // calculate init error between the vehicle and the ref.
        void processInitState();

        void setReferencePathLength();

        std::shared_ptr<Map> grid_map_;
        std::shared_ptr<CollisionChecker> collision_checker_;
        std::shared_ptr<ReferencePath> reference_path_;
        std::shared_ptr<VehicleState> vehicle_state_;
    };
}

#endif // PATH_PLANNER__PATHPLANNER_HPP_
