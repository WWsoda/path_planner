//
// Created by wwsoda on 22- 2-10.
//

#ifndef COLLOSION_CHECKER_HPP
#define COLLOSION_CHECKER_HPP

#include "map.hpp"
#include "car_geometry.hpp"
#include "../data_struct/data_struct.hpp"

namespace PathPlannerNS
{
    class Config;

    class CollisionChecker
    {
    public:
        CollisionChecker() = delete;
        CollisionChecker(const grid_map::GridMap &in_gm);

        bool isSingleStateCollisionFreeImproved(const State &current);

        bool isSingleStateCollisionFree(const State &current);

    private:
        const Map map_;
        CarGeometry car_;
    };
}

#endif // COLLOSION_CHECKER_HPP