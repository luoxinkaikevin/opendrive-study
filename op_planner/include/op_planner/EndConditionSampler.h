#pragma once

#include <array>
#include <utility>
#include <vector>
#include <memory>
#include <string>
#include <algorithm>
#include "op_planner/PathTimeGraph.h"
#include "op_planner/FeasibleRegion.h"
#include "op_planner/PredictionQuerier.h"
#include "op_planner/RoadNetwork.h"

namespace PlannerHNS
{

    // Input: planning objective, vehicle kinematic/dynamic constraints,
    // Output: sampled ending 1 dimensional states with corresponding time duration.
    class EndConditionSampler
    {
    public:
        EndConditionSampler(
            const std::array<double, 3> &init_s, const std::array<double, 3> &init_d,
            std::shared_ptr<PathTimeGraph> ptr_path_time_graph,
            std::shared_ptr<PredictionQuerier> ptr_prediction_querier);

        virtual ~EndConditionSampler() = default;

        std::vector<std::pair<std::array<double, 3>, double>>
        SampleLatEndConditions() const;

        std::vector<std::pair<std::array<double, 3>, double>>
        SampleLonEndConditionsForCruising(const double ref_cruise_speed) const;

        std::vector<std::pair<std::array<double, 3>, double>>
        SampleLonEndConditionsForStopping(const double ref_stop_point) const;

        std::vector<Condition> SampleLonEndConditionsForPathTimePoints() const;

    private:
        void QueryFollowPathTimePoints(
            const std::string &obstacle_id,
            std::vector<SamplePoint> *const sample_points) const;
        void QueryOvertakePathTimePoints(
            const std::string &obstacle_id,
            std::vector<SamplePoint> *sample_points) const;
        double ProjectVelocityAlongReferenceLine(
            const std::string &obstacle_id, const double s, const double t) const;
        std::vector<SamplePoint> QueryPathTimeObstacleSamplePoints() const;

    private:
        std::array<double, 3> init_s_;
        std::array<double, 3> init_d_;
        std::shared_ptr<PathTimeGraph> ptr_path_time_graph_;
        std::shared_ptr<PredictionQuerier> ptr_prediction_querier_;
        FeasibleRegion feasible_region_;
        double lat_seed_width_;
        CAR_BASIC_INFO car_params;
    };

} // namespace PlannerHNS