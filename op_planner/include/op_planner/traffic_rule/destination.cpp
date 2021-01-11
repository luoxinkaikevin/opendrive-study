#include "op_planner/traffic_rule/destination.h"

namespace PlannerHNS
{

    bool Destination::ApplyRule(Frame *frame,
                                ReferenceLineInfo *const reference_line_info)
    {
        CHECK_NOTNULL(frame);
        CHECK_NOTNULL(reference_line_info);

        MakeDecisions(frame, reference_line_info);

        return true;
    }

    /**
 * @brief: make decision
 */
    void Destination::MakeDecisions(Frame *const frame,
                                    ReferenceLineInfo *const reference_line_info)
    {
        CHECK_NOTNULL(frame);
        CHECK_NOTNULL(reference_line_info);

        if (!frame->is_near_destination())
        {
            return;
        }

        BuildStopDecision(frame, reference_line_info);

        return;
    }

    /**
 * @brief: build stop decision
 */
    int Destination::BuildStopDecision(
        Frame *frame, ReferenceLineInfo *const reference_line_info)
    {
        CHECK_NOTNULL(frame);
        CHECK_NOTNULL(reference_line_info);

        std::size_t end_index =
            reference_line_info->reference_line().reference_points().size() - 1;
        double end_s =
            reference_line_info->reference_line().reference_points().at(end_index).s();


        double dest_lane_s = std::max(
            0.0, end_s - FLAGS_virtual_stop_wall_length - stop_distance);
    
        Stop(frame, reference_line_info, "end", dest_lane_s);

        return 0;
    }

    /**
 * @brief: build on-lane stop decision upon arriving at destination
 */
    int Destination::Stop(Frame *const frame,
                          ReferenceLineInfo *const reference_line_info,
                          const std::string lane_id,
                          const double lane_s)
    {
        CHECK_NOTNULL(frame);
        CHECK_NOTNULL(reference_line_info);

        const auto &reference_line = reference_line_info->reference_line();

        // create virtual stop wall
        std::string stop_wall_id = FLAGS_destination_obstacle_id;

        auto *obstacle =
            frame->CreateStopObstacle(reference_line_info, stop_wall_id, lane_s);


        if (!obstacle)
        {
            AERROR << "Failed to create obstacle [" << stop_wall_id << "]";
            return -1;
        }

        PathObstacle *stop_wall = reference_line_info->AddObstacle(obstacle);
        if (!stop_wall)
        {
            AERROR << "Failed to create path_obstacle for: " << stop_wall_id;
            return -1;
        }

        // std::cout<<"11"<<std::endl;

        // build stop decision
        const auto stop_wall_box = stop_wall->obstacle()->center.pos;
        
        if (!reference_line.IsOnRoad(stop_wall_box))
        {
            ADEBUG << "destination point is not on road";
            return 0;
        }

        auto stop_point = reference_line.GetNearestReferencePoint(
            stop_wall->PerceptionSLBoundary().start_s - stop_distance);

        ObjectDecision stop_decision;
        stop_decision.decision_type = stop;
        stop_decision.reason_code = StopReasonCode::STOP_REASON_DESTINATION;
        stop_decision.distance_s = -stop_distance;
        stop_decision.stop_point = stop_point.pos;

        // std::cout<<"stop_distance "<<stop_distance<<std::endl;

        auto *path_decision = reference_line_info->path_decision();
        path_decision->AddLongitudinalDecision(
            "destination", stop_wall->Id(), stop_decision);

        return 0;
    }

} // namespace PlannerHNS
