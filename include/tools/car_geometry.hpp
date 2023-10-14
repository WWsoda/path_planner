//
// Created by wwsoda on 22-3-10
//

#ifndef PATH_PLANNER_INCLUDE_TOOLS_CAR_GEOMETRY_HPP_
#define PATH_PLANNER_INCLUDE_TOOLS_CAR_GEOMETRY_HPP_

#include <iostream>
#include <vector>
#include "data_struct/data_struct.hpp"

namespace PathPlannerNS
{

    class CarGeometry
    {
    public:
        CarGeometry() = default;
        CarGeometry(double width, double back_length, double front_length);
        void init(double width, double back_length, double front_length);
        std::vector<Circle> getCircles(const State &pos) const;
        Circle getBoundingCircle(const State &pos) const;

    private:
        void setCircles();
        double width_{}, length_{}, front_length_{}, back_length_{};
        // Four corners:
        // f: front, r: rear
        // l: left, r: right
        State fl_p_, fr_p_, rl_p_, rr_p_;
        std::vector<Circle> circles_;
        Circle bounding_c_;
    };
}

#endif // PATH_PLANNER_INCLUDE_TOOLS_CAR_GEOMETRY_HPP_