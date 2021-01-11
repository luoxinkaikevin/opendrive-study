#pragma once
#include <limits>
#include "op_planner/planning_gflags.h"
#include "op_planner/reference_line/reference_line_info.h"

namespace PlannerHNS
{
    namespace util
    {

        double GetADCStopDeceleration(ReferenceLineInfo *const reference_line_info,
                                      const double stop_line_s,
                                      const double min_pass_s_distance);

    } // namespace util
} // namespace PlannerHNS
