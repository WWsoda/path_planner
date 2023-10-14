#include <glog/logging.h>
#include "reference_path_smoother/reference_path_smoother.hpp"
#include "tools/spline.h"
#include "tools/tools.hpp"
#include "tools/map.hpp"
#include "config/planning_flags.hpp"
#include "data_struct/reference_path.hpp"
#include "reference_path_smoother/tension_smoother.hpp"
#include "reference_path_smoother/tension_smoother_2.hpp"
#include "OsqpEigen/OsqpEigen.h"

namespace PathPlannerNS
{
    std::unique_ptr<ReferencePathSmoother> ReferencePathSmoother::create(const std::string &type,
                                                                         const std::vector<State> &input_points,
                                                                         const State &start_state,
                                                                         const Map &grid_map)
    {
        if (type == "TENSION")
        {
            return std::unique_ptr<ReferencePathSmoother>{new TensionSmoother(input_points, start_state, grid_map)};
        }
        else if (type == "TENSION2")
        {
            return std::unique_ptr<ReferencePathSmoother>{new TensionSmoother2(input_points, start_state, grid_map)};
        }
        else
        {
            LOG(ERROR) << "No such smoother!";
            return nullptr;
        }
    }

    bool ReferencePathSmoother::solved(std::shared_ptr<PathPlannerNS::ReferencePath> reference_path)
    {
        // TODO: deal with short reference path.
        if (input_points_.size() < 4)
        {
            LOG(ERROR) << "Few reference points.";
            return false;
        }

        bSpline();

        if (!smooth(reference_path))
            return false;
        if (!graphSearchDp(reference_path))
            return false;
        return postSmooth(reference_path);
    }

    bool ReferencePathSmoother::segmentRawReference(std::vector<double> *x_list,
                                                    std::vector<double> *y_list,
                                                    std::vector<double> *s_list,
                                                    std::vector<double> *angle_list,
                                                    std::vector<double> *k_list) const
    {
        if (s_list_.size() != x_list_.size() || s_list_.size() != y_list_.size())
        {
            LOG(ERROR) << "Raw path x y and s size not equal!";
            return false;
        }
        double max_s = s_list_.back();
        tk::spline x_spline, y_spline;
        x_spline.set_points(s_list_, x_list_);
        y_spline.set_points(s_list_, y_list_);
        // Divide the raw path.
        double delta_s = 1.0;
        s_list->emplace_back(0);
        while (s_list->back() < max_s)
        {
            s_list->emplace_back(s_list->back() + delta_s);
        }
        if (max_s - s_list->back() > 1)
        {
            s_list->emplace_back(max_s);
        }
        auto point_num = s_list->size();
        // Store reference states in vectors. They will be used later.
        for (size_t i = 0; i != point_num; ++i)
        {
            double length_on_ref_path = s_list->at(i);
            double dx = x_spline.deriv(1, length_on_ref_path);
            double dy = y_spline.deriv(1, length_on_ref_path);
            double ddx = x_spline.deriv(2, length_on_ref_path);
            double ddy = y_spline.deriv(2, length_on_ref_path);
            double angle = atan2(dy, dx);
            angle_list->emplace_back(angle);
            double curvature = (dx * ddy - dy * ddx) / pow(dx * dx + dy * dy, 1.5);
            k_list->emplace_back(curvature);
            x_list->emplace_back(x_spline(length_on_ref_path));
            y_list->emplace_back(y_spline(length_on_ref_path));
        }
        return true;
    }

    std::vector<std::vector<double>> ReferencePathSmoother::display() const
    {
        return std::vector<std::vector<double>>{x_list_, y_list_, s_list_};
    }

    double ReferencePathSmoother::getG(const PathPlannerNS::APoint &point,
                                       const PathPlannerNS::APoint &parent) const
    {
        // Obstacle cost.
        grid_map::Position position(point.x, point.y);
        double obstacle_cost = 0;
        double distance_to_obs = grid_map_.getObstacleDistance(position);
        double safety_distance = 5;
        if (distance_to_obs < safety_distance)
        {
            obstacle_cost = (safety_distance - distance_to_obs) / safety_distance * FLAGS_search_obstacle_cost;
        }
        // Deviation cost.
        double offset_cost = fabs(point.l) / FLAGS_search_lateral_range * FLAGS_search_deviation_cost;
        // Smoothness cost.
        return parent.g + offset_cost + obstacle_cost;
    }
    void ReferencePathSmoother::calculateCostAt(std::vector<std::vector<DpPoint>> &samples,
                                                int layer_index,
                                                int lateral_index) const
    {
        if (layer_index == 0)
            return;
        if (!samples[layer_index][lateral_index].is_feasible)
            return;

        static const double weight_ref_offset = 1.0;
        static const double weight_obstacle = 0.5;
        static const double weight_angle_change = 16.0;
        static const double weight_ref_angle_diff = 0.5;
        static const double safe_distance = 3.0;

        auto &point = samples[layer_index][lateral_index];
        double self_cost = 0;
        if (point.dis_to_obs < safe_distance)
            self_cost += (safe_distance - point.dis_to_obs) / safe_distance * weight_obstacle;
        self_cost += fabs(point.l) / FLAGS_search_lateral_range * weight_ref_offset;

        audo min_cost = DBL_MAX;
        for (const auto &pre_point : samples[layer_index - 1])
        {
            if (!pre_point.is_feasible)
                continue;
            if (fabs(pre_point.l - point.l) > (point.s - pre_point.s))
                continue;
            double direction = atan2(point.y - pre_point.y, point.x - pre_point.x);
            double edge_cost = fabs(constrainAngle(direction - pre_point.dir)) / M_PI_2 * weight_angle_change + fabs(constrainAngle(direction - point.heading)) / M_PI_2 * weight_ref_angle_diff;
            double total_cost = self_cost + edge_cost_ + pre_point.cost;
            if (total_cost < min_cost)
            {
                min_cost = total_cost;
                point.parent = &pre_point;
                point.dir = direction;
            }
        }
        if (point.parent)
            point.cost = min_cost;
    }

}