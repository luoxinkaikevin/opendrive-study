#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include <limits>

#include "op_planner/reference_line/reference_line_info.h"
#include "op_planner/reference_line/frame.h"
#include "op_planner/traffic_rule/util.h"
#include "op_planner/RoadElement.h"
#include "op_planner/RoadNetwork.h"

namespace PlannerHNS
{

    class SignalLight
    {
    public:
        virtual ~SignalLight() = default;

        bool ApplyRule(Frame *const frame,
                       ReferenceLineInfo *const reference_line_info,
                       const RoadNetwork &map,
                       const std::vector<TrafficState> &traffic_state);

    private:
        void ReadSignals();
        bool FindValidSignalLight(ReferenceLineInfo *const reference_line_info);
        TrafficLight GetSignal(const std::string &signal_id);
        void MakeDecisions(Frame *const frame,
                           ReferenceLineInfo *const reference_line_info, const RoadNetwork &map,
                            const std::vector<TrafficState> &traffic_state);
        bool BuildStopDecision(Frame *const frame,
                               ReferenceLineInfo *const reference_line_info,
                               PathOverlap *const signal_light);
        void SetCreepForwardSignalDecision(
            ReferenceLineInfo *const reference_line_info,
            PathOverlap *const signal_light) const;

    private:
        static constexpr char const *const SIGNAL_LIGHT_VO_ID_PREFIX = "SL_";
        PathOverlap signal_light_from_path_;
        std::unordered_map<std::string, const TrafficLight *>
            detected_signals_;

        // stop distance from stop line
        // meter
        double stop_distance = 1.0;
        // max deceleration
        double max_stop_deceleration = 6.0;
        // min s_distance for adc to be considered have passed signal_light (stop_line_end_s)
        // meter
        double min_pass_s_distance = 4.0;

        // treat yellow light as red when deceleration (abstract value in m/s^2)
        // is less than this threshold; otherwise treated as green light
        double max_stop_deacceleration_yellow_light = 3.0;
        double signal_expire_time_sec = 5.0; // second
    };

} // namespace PlannerHNS
