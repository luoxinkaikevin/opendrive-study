#pragma once

#include <array>
#include <memory>
#include <vector>
#include <cmath>
#include <utility>
#include "op_planner/PolynomialCurve.h"
#include "op_planner/RoadNetwork.h"
#include "op_planner/PathTimeGraph.h"
#include "op_planner/box2d.h"

namespace PlannerHNS
{
    class CollisionChecker
    {
    public:
        explicit CollisionChecker(
            const std::vector<const DetectedObject *> &obstacles,
            const double ego_vehicle_s,
            const double ego_vehicle_d,
            const std::vector<WayPoint> &discretized_reference_line,
            const ReferenceLineInfo *ptr_reference_line_info,
            const std::shared_ptr<PathTimeGraph> &ptr_path_time_graph);

        bool InCollision(const DiscretizedTrajectory &discretized_trajectory);

        std::vector<const DetectedObject *> Obstacles_considered()
        {
            return obstacles_considered;
        }

    private:
        void BuildPredictedEnvironment(
            const std::vector<const DetectedObject *> &obstacles,
            const double ego_vehicle_s,
            const double ego_vehicle_d,
            const std::vector<WayPoint> &discretized_reference_line);

        bool IsEgoVehicleInLane(const double ego_vehicle_s,
                                const double ego_vehicle_d);

        bool IsObstacleBehindEgoVehicle(
            const DetectedObject *obstacle, const double ego_vehicle_s,
            const std::vector<WayPoint> &discretized_reference_line);

    private:
        const ReferenceLineInfo *ptr_reference_line_info_;
        std::shared_ptr<PathTimeGraph> ptr_path_time_graph_;
        CAR_BASIC_INFO car_params;
        std::vector<std::vector<math::Box2d>> predicted_bounding_rectangles_;

        std::vector<const DetectedObject *> obstacles_considered;
        // double ego_vehicle_lane_width_;
    };

} // namespace PlannerHNS
