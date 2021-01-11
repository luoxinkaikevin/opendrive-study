#pragma once

#include <string>
#include <algorithm>
#include <vector>

#include "op_planner/traffic_rule/util.h"
#include "op_planner/planning_gflags.h"
#include "op_planner/reference_line/frame.h"
#include "op_planner/reference_line/reference_line_info.h"
#include "op_planner/RoadNetwork.h"

namespace PlannerHNS
{

    /**
 * This class decides whether we should stop for destination.
 * situation.
 */
    class Destination
    {
    public:
        virtual ~Destination() = default;

        bool ApplyRule(Frame *const frame,
                                 ReferenceLineInfo *const reference_line_info);

    private:
        void MakeDecisions(Frame *const frame,
                           ReferenceLineInfo *const reference_line_info);
        int BuildStopDecision(Frame *const frame,
                              ReferenceLineInfo *const reference_line_info);
        int Stop(Frame *const frame,
                 ReferenceLineInfo *const reference_line_info,
                 const std::string lane_id,
                 const double lane_s);
        bool CheckPullOver(ReferenceLineInfo *const reference_line_info,
                           const std::string lane_id,
                           const double lane_s,
                           GPSPoint *dest_point);
        int PullOver(GPSPoint *const dest_point);

    private:
        // flag to enable pullover upon arriving destination
        bool enable_pull_over = false;
        // stop distance from destination line
        double stop_distance = 0.5;
        // meter
        // distance to stop point to start planning pull over
        // TODO(all): must be in sync with the same config in PULL_OVER will remove
        double pull_over_plan_distance = 35.0;
    };

} // namespace PlannerHNS