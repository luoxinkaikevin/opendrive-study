#include "op_planner/traffic_rule/signal_light.h"

namespace PlannerHNS
{

    bool SignalLight::ApplyRule(Frame *const frame,
                                ReferenceLineInfo *const reference_line_info,
                                const RoadNetwork &map,
                                const std::vector<TrafficState> &traffic_state)
    {
        if (!FindValidSignalLight(reference_line_info))
        {
            return true;
        }

        MakeDecisions(frame, reference_line_info, map, traffic_state);
        return true;
    }

    //read signal light from vision module

    bool SignalLight::FindValidSignalLight(
        ReferenceLineInfo *const reference_line_info)
    {
        const std::vector<PathOverlap> &signal_lights =
            reference_line_info->reference_line().signal_overlaps();

        if (signal_lights.size() <= 0)
        {
            ADEBUG << "No signal lights from reference line.";
            return false;
        }
        double max_distance = 1000;
        bool find_signal = false;
        ADEBUG << "size " << signal_lights.size();
        for (const PathOverlap &signal_light : signal_lights)
        {
            ADEBUG << "signal_light.start_s " << signal_light.start_s + min_pass_s_distance
                   << " reference_line_info->AdcSlBoundary().end_s " << reference_line_info->AdcSlBoundary().end_s;

            // if signal light in front of adc
            if (signal_light.start_s + min_pass_s_distance >
                reference_line_info->AdcSlBoundary().end_s)
            {
                if (max_distance > signal_light.start_s + min_pass_s_distance)
                {
                    find_signal = true;
                    max_distance = signal_light.start_s + min_pass_s_distance;
                    signal_light_from_path_.end_s = signal_light.end_s;
                    signal_light_from_path_.object_id = signal_light.object_id;
                    signal_light_from_path_.start_s = signal_light.start_s;
                    ADEBUG << "signal_light_from_path_.end_s " << signal_light_from_path_.end_s
                           << " signal_light_from_path_.object_id " << signal_light_from_path_.object_id
                           << " signal_light_from_path_.start_s " << signal_light_from_path_.start_s;
                }
            }
        }
        std::cout << "find_signal " << find_signal << std::endl;
        return find_signal;
    }

    void SignalLight::MakeDecisions(Frame *const frame,
                                    ReferenceLineInfo *const reference_line_info,
                                    const RoadNetwork &map,
                                    const std::vector<TrafficState> &traffic_state)
    {
        bool has_stop = false;

        auto traffic_light_temp = map.trafficLight_map.at(signal_light_from_path_.object_id);
        std::string traffic_light_temp_id = std::to_string(traffic_light_temp.id);

        ADEBUG << "traffic_light_temp_id " << traffic_light_temp_id
               << "traffic_light_temp.roadId" << traffic_light_temp.roadId
               << "traffic_light_temp left_or_right " << traffic_light_temp.left_or_right;

        //read signal light from vision module
        // const TrafficLight signal = GetSignal(signal_light.object_id);
        if (traffic_light_temp.left_or_right == 0)
        {
            for (auto traffic : traffic_state)
            {
                ADEBUG << "traffic.lr " << traffic.left_right
                       << " traffic.color " << traffic.color;
                if (traffic.left_right == 0)
                {
                    // red
                    if (traffic.color == 0)
                    {

                        BuildStopDecision(frame, reference_line_info, &signal_light_from_path_);
                    }
                }
            }
        }
        else if (traffic_light_temp.left_or_right == 1)
        {
            for (auto traffic : traffic_state)
            {
                if (traffic.left_right == 1)
                {
                    if (traffic.color == 0)
                    {
                        BuildStopDecision(frame, reference_line_info, &signal_light_from_path_);
                    }
                }
            }
        }
    }

    bool SignalLight::BuildStopDecision(
        Frame *const frame,
        ReferenceLineInfo *const reference_line_info,
        PathOverlap *const signal_light)
    {
        // check
        const auto &reference_line = reference_line_info->reference_line();
        if (!WithinBound(0.0, reference_line.Length(), signal_light->start_s))
        {
            ADEBUG << "signal_light " << signal_light->object_id
                   << " is not on reference line";
            return true;
        }

        // create virtual stop wall
        std::string virtual_obstacle_id =
            SIGNAL_LIGHT_VO_ID_PREFIX + std::to_string(signal_light->object_id);
        auto *obstacle = frame->CreateStopObstacle(
            reference_line_info, virtual_obstacle_id, signal_light->start_s);
        if (!obstacle)
        {
            AERROR << "Failed to create obstacle[" << virtual_obstacle_id << "]";
            return false;
        }
        PathObstacle *stop_wall = reference_line_info->AddObstacle(obstacle);
        if (!stop_wall)
        {
            AERROR << "Failed to create path_obstacle for " << virtual_obstacle_id;
            return false;
        }

        // build stop decision
        const double stop_s = signal_light->start_s - stop_distance;
        auto stop_point = reference_line.GetNearestReferencePoint(stop_s).pos;

        ObjectDecision stop_decision;
        stop_decision.decision_type = stop;
        stop_decision.reason_code = StopReasonCode::STOP_REASON_SIGNAL;
        stop_decision.distance_s = -stop_distance;
        stop_decision.stop_point = stop_point;

        ADEBUG << "signal " << signal_light->object_id
               << "distance_s " << stop_decision.distance_s;

        auto *path_decision = reference_line_info->path_decision();
        if (!path_decision->MergeWithMainStop(stop_decision, stop_wall->Id(),
                                              reference_line_info->reference_line(),
                                              reference_line_info->AdcSlBoundary()))
        {
            ADEBUG << "signal " << signal_light->object_id
                   << " is not the closest stop.";
            return false;
        }

        path_decision->AddLongitudinalDecision("traffic_light", stop_wall->Id(), stop_decision);

        return true;
    }

} // namespace PlannerHNS
