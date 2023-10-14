//
//
// Created by wwsoda on 22-4-29

#ifndef PATH_PLANNER_INCLUDE_PATH_PLANNERR_DATA_STRUCT_VEHICLE_STATE_FRENET_HPP_
#define PATH_PLANNER_INCLUDE_PATH_PLANNERR_DATA_STRUCT_VEHICLE_STATE_FRENET_HPP_
#include <vector>

namespace PathPlannerNS
{
    class State;

    class VehicleState
    {
    public:
        VehicleState();
        VehicleState(const State &start_state,
                     const State &end_state,
                     double offset = 0.0,
                     double heading_error = 0.0);
        ~VehicleState();
        const State &getStartState() const;
        const State &getTargetState() const;
        void setStartState(const State &state);
        void setTargetState(const State &state);
        std::vector<double> getInitError() const;
        void setInitError(double init_offset, double init_heading_error);

    private:
        // Initial state.
        State *start_state_;
        // Target state.
        State *target_state_;
        // Initial error with reference line.
        double initial_offset_{};
        double initial_heading_error_{};
    }; // namespace name
}
#endif PATH_PLANNER_INCLUDE_PATH_PLANNERR_DATA_STRUCT_VEHICLE_STATE_FRENET_HPP_