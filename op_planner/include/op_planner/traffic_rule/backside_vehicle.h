#pragma once

#include <string>

#include "op_planner/reference_line/reference_line_info.h"
#include "op_planner/reference_line/frame.h"

namespace PlannerHNS
{

    class BacksideVehicle
    {
    public:
        virtual ~BacksideVehicle() = default;

        bool ApplyRule(Frame *const frame,
                       ReferenceLineInfo *const reference_line_info);

    private:
        /**
   * @brief When the reference line info indicates that there is no lane change,
   * use lane keeping strategy for back side vehicles.
   */
        void MakeLaneKeepingObstacleDecision(const SLBoundary &adc_sl_boundary,
                                             PathDecision *path_decision);
    
    double  backside_lane_width = 4.0;
    };

} // namespace PlannerHNS
