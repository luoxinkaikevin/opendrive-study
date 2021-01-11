#include "op_planner/traffic_rule/traffic_decider.h"

namespace PlannerHNS
{

    void TrafficDecider::BuildPlanningTarget(ReferenceLineInfo *reference_line_info)
    {
        double min_s = std::numeric_limits<double>::infinity();
        StopPoint stop_point;
        for (const auto *obstacle :
             reference_line_info->path_decision()->path_obstacles().Items())
        {
            if (obstacle->obstacle()->IsVirtual() &&
                obstacle->HasLongitudinalDecision() &&
                obstacle->LongitudinalDecision().decision_type == stop &&
                obstacle->PerceptionSLBoundary().start_s < min_s)
            {
                min_s = obstacle->PerceptionSLBoundary().start_s;
                const auto &stop_code = obstacle->LongitudinalDecision().reason_code;
                if (stop_code == StopReasonCode::STOP_REASON_DESTINATION ||
                    stop_code == StopReasonCode::STOP_REASON_CROSSWALK ||
                    stop_code == StopReasonCode::STOP_REASON_STOP_SIGN ||
                    stop_code == StopReasonCode::STOP_REASON_YIELD_SIGN ||
                    stop_code == StopReasonCode::STOP_REASON_CREEPER ||
                    stop_code == StopReasonCode::STOP_REASON_SIGNAL)
                {
                    stop_point.type = HARD;
                    ADEBUG << "Hard stop at: " << min_s
                           << "REASON: " << stop_code;
                }
                else if (stop_code == StopReasonCode::STOP_REASON_YELLOW_SIGNAL)
                {
                    stop_point.type = SOFT;
                    ADEBUG << "Soft stop at: " << min_s << "  STOP_REASON_YELLOW_SIGNAL";
                }
                else
                {
                    ADEBUG << "No planning target found at reference line.";
                }
            }
        }
        if (min_s != std::numeric_limits<double>::infinity())
        {
            stop_point.s = min_s + FLAGS_virtual_stop_wall_length / 2.0;
            reference_line_info->SetStopPoint(stop_point);
        }
    }

    bool TrafficDecider::Execute(Frame *frame,
                                 ReferenceLineInfo *reference_line_info,
                                 const RoadNetwork &map)
    {
        CHECK_NOTNULL(frame);
        CHECK_NOTNULL(reference_line_info);
        bool destination_decision_flag = true;
        bool crosswalk_flag = true;
        bool traffic_light_flag = true;
        destination_decision_flag = (destination_decision.ApplyRule(frame, reference_line_info));
        crosswalk_flag = crosswalk.ApplyRule(frame, reference_line_info, map);
        if (!traffic_state_.empty())
        {
            traffic_light_flag = signal_light.ApplyRule(frame, reference_line_info, map, traffic_state_);
        }

        BuildPlanningTarget(reference_line_info);

        bool decision_flag = destination_decision_flag && crosswalk_flag && traffic_light_flag;
        // bool decision_flag = destination_decision_flag;
        return decision_flag;
        // return 1;
    }

} // namespace PlannerHNS
