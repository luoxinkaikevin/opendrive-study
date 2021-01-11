#include "op_planner/traffic_rule/crosswalk.h"

namespace PlannerHNS
{

    // using CrosswalkToStop =
    //     std::vector<std::pair<PathOverlap *, std::vector<std::string>>>;

    bool Crosswalk::ApplyRule(Frame *const frame,
                              ReferenceLineInfo *const reference_line_info,
                              const RoadNetwork &map)
    {
        CHECK_NOTNULL(frame);
        CHECK_NOTNULL(reference_line_info);

        if (!FindCrosswalks(reference_line_info))
        {
            return true;
        }

        MakeDecisions(frame, reference_line_info, map);
        return true;
    }

    void Crosswalk::MakeDecisions(Frame *const frame,
                                  ReferenceLineInfo *const reference_line_info,
                                  const RoadNetwork &map)
    {
        CHECK_NOTNULL(frame);
        CHECK_NOTNULL(reference_line_info);

        auto *path_decision = reference_line_info->path_decision();
        double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s;
        double adc_front_edge_l = reference_line_info->AdcSlBoundary().end_l;

        // CrosswalkToStop crosswalks_to_stop;

        for (auto crosswalk_overlap : crosswalk_overlaps_)
        {
            auto crosswalk_temp = map.crossing_map.at(crosswalk_overlap->object_id);
            std::string crosswalk_id = std::to_string(crosswalk_temp.id);

            // skip crosswalk if master vehicle body already passes the stop line
            double stop_line_end_s = crosswalk_overlap->end_s;
            
            if (adc_front_edge_s - stop_line_end_s > min_pass_s_distance)
            {
                ADEBUG << "skip: crosswalk_id[" << crosswalk_id << "] stop_line_end_s["
                       << stop_line_end_s << "] adc_front_edge_s[" << adc_front_edge_s
                       << "]. adc_front_edge passes stop_line_end_s + buffer.";
                continue;
            }

            std::vector<std::string> pedestrians;
            for (const auto *path_obstacle : path_decision->path_obstacles().Items())
            {

                auto perception_obstacle = path_obstacle->obstacle();
                const std::string &obstacle_id = path_obstacle->Id();
                if (perception_obstacle->IsVirtual())
                {
                    continue;
                }

                const math::Polygon2d crosswalk_exp_poly = crosswalk_temp.crosswalk_box;
                 bool in_expanded_crosswalk = 
                 crosswalk_exp_poly.IsPointIn(perception_obstacle->center.pos);

                // bool in_expanded_crosswalk =
                //     crosswalk_exp_poly.HasOverlap(perception_obstacle->PerceptionPolygon());

                if (in_expanded_crosswalk)
                {
                    pedestrians.push_back(obstacle_id);
                    BuildStopDecision(frame, reference_line_info,
                                      const_cast<PathOverlap *>(crosswalk_overlap),
                                      pedestrians);

                    ADEBUG << "obstacle_id[" << obstacle_id << "] crosswalk_id[" << crosswalk_id
                           << "]: in crosswalk expanded area";
                    break;
                }


                // bool is_path_cross =
                //     !path_obstacle->reference_line_st_boundary().IsEmpty();

                // if (!is_path_cross)
                // {
                //     continue;
                // }

                // check type
                // if (perception_obstacle->label != 0 &&
                //     perception_obstacle->label != 1)
                // {
                //     ADEBUG << "obstacle_id[" << obstacle_id << "] type["
                //            << perception_obstacle->label << "]. skip";
                //     continue;
                // }

                // expand crosswalk polygon
                // note: crosswalk expanded area will include sideway area

                // bool in_expanded_crosswalk = crosswalk_exp_poly.IsPointIn(point);
                // if (!in_expanded_crosswalk)
                // {
                //     ADEBUG << "skip: obstacle_id[" << obstacle_id << "] crosswalk_id[" << crosswalk_id
                //            << "]: not in crosswalk expanded area";
                //     continue;
                // }

                //     auto obstacle_sl_point =
                //         PlanningHelpers::GetPathFrenetCoordinate(
                //             reference_line_info->reference_line().reference_points(),
                //             perception_obstacle->center.pos);

                //     double obstacle_l_distance = std::fabs(obstacle_sl_point.second);

                //     auto obstacle_pose =
                //         path_obstacle->obstacle()->center.pos;
                //     bool is_on_road =
                //         reference_line_info->reference_line().IsOnRoad(obstacle_pose);
                //     bool is_path_cross =
                //         !path_obstacle->reference_line_st_boundary().IsEmpty();

                //     ADEBUG << "obstacle_id[" << obstacle_id << "]crosswalk_id[" << crosswalk_id
                //            << "] obstacle_l["
                //            << obstacle_sl_point.second << "] within_expanded_crosswalk_area["
                //            << in_expanded_crosswalk << "] is_on_road[" << is_on_road
                //            << "] is_path_cross[" << is_path_cross << "]";

                //     bool stop = false;
                //     if (obstacle_l_distance >= stop_loose_l_distance)
                //     {
                //         // (1) when obstacle_l_distance is big enough(>= loose_l_distance),
                //         //     STOP only if path crosses
                //         if (is_path_cross)
                //         {
                //             stop = true;
                //             ADEBUG << "need_stop(>=l2): obstacle_id[" << obstacle_id << " crosswalk_id["
                //                    << crosswalk_id << "]";
                //         }
                //     }
                //     else if (obstacle_l_distance <= stop_strick_l_distance)
                //     {
                //         // (2) when l_distance <= strick_l_distance + on_road(not on sideway),
                //         //     always STOP
                //         // (3) when l_distance <= strick_l_distance + not on_road(on sideway),
                //         //     STOP only if path crosses
                //         if (is_on_road || is_path_cross)
                //         {
                //             stop = true;
                //             ADEBUG << "need_stop(<=11): obstacle_id[" << obstacle_id
                //                    << "]crosswalk_id[" << crosswalk_id << "]";
                //         }
                //     }
                //     else
                //     {
                //         // TODO(all)
                //         // (4) when l_distance is between loose_l and strick_l
                //         //     use history decision of this crosswalk to smooth unsteadiness
                //         stop = true;
                //     }

                //     if (stop)
                //     {
                //         pedestrians.push_back(obstacle_id);
                //         ADEBUG << "wait for: obstacle_id[" << obstacle_id << "]crosswalk_id[" << crosswalk_id
                //                << "]";
                //     }
                //     else
                //     {
                //         ADEBUG << "skip: obstacle_id[" << obstacle_id << "]crosswalk_id[" << crosswalk_id
                //                << "]";
                //     }
                // }

                // if (!pedestrians.empty())
                // {
                //     // stop decision
                //     // double stop_deceleration = util::GetADCStopDecert_s, min_pass_s_distance);
                //     // ADEBUG << "stop_deceleration " << stop_deceleraleration(
                //     //     reference_line_info, crosswalk_overlap->station
                //     //        << " min_pass_s_distance " << min_pass_s_distance
                //     //        << " crosswalk_overlap->start_s " << crosswalk_overlap->start_s;

                //     // if (stop_deceleration < max_stop_deceleration)
                //     {
                //         BuildStopDecision(frame, reference_line_info,
                //                           const_cast<PathOverlap *>(crosswalk_overlap),
                //                           pedestrians);
                //         ADEBUG << "crosswalk_id[" << crosswalk_id << "] STOP";
                //     }
                // }
            }
        }
    }

    bool Crosswalk::FindCrosswalks(ReferenceLineInfo *const reference_line_info)
    {
        CHECK_NOTNULL(reference_line_info);

        crosswalk_overlaps_.clear();
        const std::vector<PathOverlap> &crosswalk_overlaps =
            reference_line_info->reference_line().crosswalk_overlaps();
        for (const PathOverlap &crosswalk_overlap : crosswalk_overlaps)
        {
            // std::cout<<"crosswalk_overlap "<<crosswalk_overlap.object_id<<std::endl;
            crosswalk_overlaps_.push_back(&crosswalk_overlap);
        }
        return crosswalk_overlaps_.size() > 0;
    }

    int Crosswalk::BuildStopDecision(Frame *const frame,
                                     ReferenceLineInfo *const reference_line_info,
                                     PathOverlap *const crosswalk_overlap,
                                     std::vector<std::string> pedestrians)
    {
        CHECK_NOTNULL(frame);
        CHECK_NOTNULL(reference_line_info);
        CHECK_NOTNULL(crosswalk_overlap);

        // // check
        const auto &reference_line = reference_line_info->reference_line();
        // if (!WithinBound(0.0, reference_line.Length(), crosswalk_overlap->start_s))
        // {
        //     ADEBUG << "crosswalk [" << crosswalk_overlap->object_id
        //            << "] reference_line Length " << reference_line.Length()
        //            << "crosswalk_overlap start_s " << crosswalk_overlap->start_s
        //            << "] is not on reference line";
        //     return 0;
        // }

        // create virtual stop wall
        std::string virtual_obstacle_id =
            "CW_" + std::to_string(crosswalk_overlap->object_id);
        ADEBUG << "virtual_obstacle_id " << virtual_obstacle_id;
        auto *obstacle = frame->CreateStopObstacle(
            reference_line_info, virtual_obstacle_id, crosswalk_overlap->start_s);
        if (!obstacle)
        {
            AERROR << "Failed to create obstacle[" << virtual_obstacle_id << "]";
            return -1;
        }
        PathObstacle *stop_wall = reference_line_info->AddObstacle(obstacle);
        if (!stop_wall)
        {
            AERROR << "Failed to create path_obstacle for: " << virtual_obstacle_id;
            return -1;
        }

        // build stop decision
        const double stop_s =
            crosswalk_overlap->start_s - stop_distance;
        auto stop_point = reference_line.GetNearestReferencePoint(stop_s);

        ObjectDecision stop_decision;
        stop_decision.decision_type = stop;
        stop_decision.reason_code = StopReasonCode::STOP_REASON_CROSSWALK;
        stop_decision.stop_point = stop_point.pos;
        stop_decision.distance_s = -stop_distance;

        auto *path_decision = reference_line_info->path_decision();
        path_decision->AddLongitudinalDecision(
            "crosswalk", stop_wall->Id(), stop_decision);

        ADEBUG << "build crosswalk stop wall success, stop_wall id" << stop_wall->Id()
               << " stop_s " << stop_s;
        return 0;
    }

} // namespace PlannerHNS
