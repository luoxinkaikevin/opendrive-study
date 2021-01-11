#pragma once

#include <utility>
#include <vector>
#include <algorithm>
#include <list>

#include "op_planner/PolynomialCurve.h"
#include "op_planner/RoadNetwork.h"
#include "op_planner/log.h"
#include "op_planner/planning_gflags.h"

namespace PlannerHNS
{

    class TrajectoryStitcher
    {
    public:
        TrajectoryStitcher() = delete;

        static std::vector<WayPoint> ComputeStitchingTrajectory(
            const WayPoint &vehicle_state, const double current_timestamp,
            const double planning_cycle_time,
            const PublishableTrajectory *prev_trajectory, bool *is_replan);

    private:
        static std::pair<double, double> ComputePositionProjection(
            const double x,
            const double y, const PublishableTrajectory &prev_trajectory);

        static std::vector<WayPoint> ComputeReinitStitchingTrajectory(
            const WayPoint &vehicle_state);
    };

} // namespace PlannerHNS
