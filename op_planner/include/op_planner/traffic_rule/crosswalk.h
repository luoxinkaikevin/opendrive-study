#pragma once

#include <string>
#include <vector>
#include <limits>
#include <utility>

#include "op_planner/reference_line/reference_line_info.h"
#include "op_planner/reference_line/frame.h"
#include "op_planner/traffic_rule/util.h"
#include "op_planner/log.h"

namespace PlannerHNS
{

    class Crosswalk
    {
    public:
        virtual ~Crosswalk() = default;

        bool ApplyRule(Frame *const frame,
                       ReferenceLineInfo *const reference_line_info,
                       const RoadNetwork &map);

    private:
        void MakeDecisions(Frame *const frame,
                           ReferenceLineInfo *const reference_line_info,
                           const RoadNetwork &map);
        bool FindCrosswalks(ReferenceLineInfo *const reference_line_info);
        int BuildStopDecision(Frame *frame,
                              ReferenceLineInfo *const reference_line_info,
                              PathOverlap *const crosswalk_overlap,
                              std::vector<std::string> pedestrians);

    private:
        // static constexpr char const *const CROSSWALK_VO_ID_PREFIX = "CW_";
        std::vector<const PathOverlap *> crosswalk_overlaps_;

        // stop distance from stop line of crosswalk
        double stop_distance = 1.0; // meter
        // max deceleration
        double max_stop_deceleration = 6.0;
        // min s_distance for adc to be considered have passed crosswalk (stop_line_end_s)
        double min_pass_s_distance = 1.0; // meter
        // expand s_distance for pedestrian/bicycle detection
        double expand_s_distance = 2.0; // meter
        // strick stop rule within this l_distance
        double stop_strick_l_distance = 4.0; // meter
        // loose stop rule beyond this l_distance
        double stop_loose_l_distance = 5.0; // meter
    };

} // namespace PlannerHNS
