#pragma once
#include "op_planner/PolynomialCurve.h"
#include "op_planner/log.h"
#include "op_planner/reference_line/reference_line_info.h"

namespace PlannerHNS
{
    class ConstraintChecker
    {
    public:
        ConstraintChecker() = delete;
        static bool ValidTrajectory(
        const DiscretizedTrajectory &trajectory,
        const ReferenceLineInfo *ptr_reference_line_info,
        const Curve1d &lat_trajectory,
        std::size_t &num_speed_lower_bound,
        std::size_t &num_longitudinal_acceleration_lower_bound,
        std::size_t &num_kappa_bound,
        std::size_t &num_lane_border_limit);
    };

    class ConstraintChecker1d
    {
    public:
        ConstraintChecker1d() = delete;

        static bool IsValidLongitudinalTrajectory(const Curve1d &lon_trajectory);

        static bool IsValidLateralTrajectory(const Curve1d &lat_trajectory,
                                             const Curve1d &lon_trajectory);
    };

} // namespace PlannerHNS
