#pragma once

#include <memory> 
#include <string>
#include <unordered_map>
#include <vector>
#include <string>
#include "op_planner/RoadNetwork.h"
#include "op_planner/PlanningHelpers.h"
#include "op_planner/log.h"

namespace PlannerHNS
{

    class PredictionQuerier
    { 
    public:
        explicit PredictionQuerier(
            const std::vector<const DetectedObject *> &obstacles,
            const std::shared_ptr<std::vector<WayPoint>> &ptr_reference_lin);

        virtual ~PredictionQuerier() = default;
        std::vector<const DetectedObject *> GetObstacles() const;
        std::unordered_map<std::string, const DetectedObject *> GetObstaclesMap() const;

        double ProjectVelocityAlongReferenceLine(
            const std::string &obstacle_id, const double s, const double t) const;

    private:
        std::unordered_map<std::string, const DetectedObject *> id_obstacle_map_;

        std::vector<const DetectedObject *> obstacles_;
        // std::shared_ptr<std::vector<DetectedObject>> ptr_objects_in_lane_;

        std::shared_ptr<std::vector<WayPoint>> ptr_reference_line_;
    };

} // namespace PlannerHNS
