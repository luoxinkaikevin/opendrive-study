#include "op_planner/traffic_rule/backside_vehicle.h"

namespace PlannerHNS
{

    void BacksideVehicle::MakeLaneKeepingObstacleDecision(
        const SLBoundary &adc_sl_boundary, PathDecision *path_decision)
    {
        ObjectDecision ignore;
        ignore.decision_type = ObjectDecisionType::ignore;
        const double adc_length_s =
            adc_sl_boundary.end_s - adc_sl_boundary.start_s;
        for (const auto *path_obstacle : path_decision->path_obstacles().Items())
        {
            if (path_obstacle->PerceptionSLBoundary().end_s >=
                adc_sl_boundary.end_s)
            { // don't ignore such vehicles.
                continue;
            }

            if (path_obstacle->reference_line_st_boundary().IsEmpty())
            {
                path_decision->AddLongitudinalDecision("backside_vehicle/no-st-region",
                                                       path_obstacle->Id(), ignore);
                path_decision->AddLateralDecision("backside_vehicle/no-st-region",
                                                  path_obstacle->Id(), ignore);
                continue;
            }
            // Ignore the car comes from back of ADC
            if (path_obstacle->reference_line_st_boundary().min_s() < -adc_length_s)
            {
                path_decision->AddLongitudinalDecision("backside_vehicle/st-min-s < adc",
                                                       path_obstacle->Id(), ignore);
                path_decision->AddLateralDecision("backside_vehicle/st-min-s < adc",
                                                  path_obstacle->Id(), ignore);
                continue;
            }

            const double lane_boundary = backside_lane_width;
            if (path_obstacle->PerceptionSLBoundary().start_s <
                adc_sl_boundary.end_s)
            {
                if (path_obstacle->PerceptionSLBoundary().start_l > lane_boundary ||
                    path_obstacle->PerceptionSLBoundary().end_l < -lane_boundary)
                {
                    continue;
                }
                path_decision->AddLongitudinalDecision("backside_vehicle/sl < adc.end_s",
                                                       path_obstacle->Id(), ignore);
                path_decision->AddLateralDecision("backside_vehicle/sl < adc.end_s",
                                                  path_obstacle->Id(), ignore);
                continue;
            }
        }
    }

    bool BacksideVehicle::ApplyRule(
        Frame *const, ReferenceLineInfo *const reference_line_info)
    {
        auto *path_decision = reference_line_info->path_decision();
        const auto &adc_sl_boundary = reference_line_info->AdcSlBoundary();
        MakeLaneKeepingObstacleDecision(adc_sl_boundary, path_decision);
        return true;
    }

} // namespace PlannerHNS
