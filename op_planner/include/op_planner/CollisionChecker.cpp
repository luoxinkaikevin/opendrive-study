#include "CollisionChecker.h"

namespace PlannerHNS
{

    CollisionChecker::CollisionChecker(
        const std::vector<const DetectedObject *> &obstacles,
        const double ego_vehicle_s,
        const double ego_vehicle_d,
        const std::vector<WayPoint> &discretized_reference_line,
        const ReferenceLineInfo *ptr_reference_line_info,
        const std::shared_ptr<PathTimeGraph> &ptr_path_time_graph)
    {
        ptr_reference_line_info_ = ptr_reference_line_info;
        ptr_path_time_graph_ = ptr_path_time_graph;
        BuildPredictedEnvironment(obstacles, ego_vehicle_s, ego_vehicle_d,
                                  discretized_reference_line);
    }

    bool CollisionChecker::InCollision(
        const DiscretizedTrajectory &discretized_trajectory)
    {

        double ego_length = car_params.length;
        double ego_width = car_params.width;

        for (std::size_t i = 0; i < discretized_trajectory.NumOfPoints(); ++i)
        {
            const auto &trajectory_point = discretized_trajectory.WayPointAt(i);
            double ego_theta = trajectory_point.pos.a;
            GPSPoint temp_wp = GPSPoint(trajectory_point.pos.x, trajectory_point.pos.y, 0, 0);

            math::Box2d ego_box(
                temp_wp, ego_theta, ego_length, ego_width);
            double shift_distance = -car_params.wheel_base / 2.0;
            GPSPoint shift_vec(shift_distance * std::cos(ego_theta),
                               shift_distance * std::sin(ego_theta), 0, 0);
            // ego_box.LongitudinalExtend(2.0 * FLAGS_lon_collision_buffer);
            ego_box.LateralExtend(2.0 * FLAGS_lat_collision_buffer);
            ego_box.Shift(shift_vec);

            for (const DetectedObject *obstacle : obstacles_considered)
            {
                auto obj_heading =
                    ptr_reference_line_info_->reference_line().GetNearestReferencePoint(
                                                                  obstacle->center.pos)
                        .pos.a;

                auto obstacle_box =
                    obstacle->PerceptionPolygon().BoundingBoxWithHeading(obj_heading);
                //for car
                if (obstacle->label == 1 &&
                    (ReferenceLineInfo::lane_change_state == READY_TO_CHANGE_LEFT ||
                     ReferenceLineInfo::lane_change_state == READY_TO_CHANGE_RIGHT))
                {
                    obstacle_box.LongitudinalExtend(2.0 * FLAGS_lon_collision_buffer * 0.1);
                    obstacle_box.LateralExtend(2.0 * FLAGS_lat_collision_buffer * 0.3);
                }
                else if (obstacle->label == 1)
                {
                    obstacle_box.LongitudinalExtend(2.0 * FLAGS_lon_collision_buffer);
                    obstacle_box.LateralExtend(2.0 * FLAGS_lat_collision_buffer);
                }
                else
                {

                    if (ptr_reference_line_info_->reference_line().current_reference_pose().pLane->roadId != 42 ||
                        ptr_reference_line_info_->reference_line().current_reference_pose().pLane->roadId != 89 ||
                        ptr_reference_line_info_->reference_line().current_reference_pose().pLane->roadId != 72)
                    {
                        obstacle_box.LongitudinalExtend(2.0 * 3);
                        obstacle_box.LateralExtend(2.0 * 0.15);
                    }
                    else
                    {
                        obstacle_box.LongitudinalExtend(2.0 * 3);
                        obstacle_box.LateralExtend(2.0 * 0.1);
                    }
                }

                auto overlap = obstacle_box.HasOverlap(ego_box);
                if (overlap)
                {
                    return true;
                }
            }
        }
        return false;
    }

    void CollisionChecker::BuildPredictedEnvironment(
        const std::vector<const DetectedObject *> &obstacles,
        const double ego_vehicle_s,
        const double ego_vehicle_d,
        const std::vector<WayPoint> &discretized_reference_line)
    {
        CHECK(predicted_bounding_rectangles_.empty());
        // If the ego vehicle is in lane,
        // then, ignore all obstacles from the same lane.
        bool ego_vehicle_in_lane = IsEgoVehicleInLane(ego_vehicle_s, ego_vehicle_d);
        obstacles_considered.clear();
        // ADEBUG << "ego_vehicle_in_lane : " << ego_vehicle_in_lane;
        for (const DetectedObject *obstacle : obstacles)
        {
            if (obstacle->IsVirtual())
            {
                continue;
            }
            auto flag_obstacle_behind_ego = IsObstacleBehindEgoVehicle(obstacle, ego_vehicle_s,
                                                                       discretized_reference_line);
            auto flag_IsObstacleInGraph = ptr_path_time_graph_->IsObstacleInGraph(obstacle->Id());
            // ADEBUG << "obstacle : " << obstacle->Id() << "flag_obstacle_behind_ego : " << flag_obstacle_behind_ego;
            // ADEBUG << "flag_IsObstacleInGraph: " << flag_IsObstacleInGraph;
            if ((ego_vehicle_in_lane && flag_obstacle_behind_ego) ||
                !flag_IsObstacleInGraph)
            {
                continue;
            }

            ADEBUG << "Obstacles considered in collision： " << obstacle->Id();
            obstacles_considered.push_back(obstacle);
        }
    }

    bool CollisionChecker::IsEgoVehicleInLane(
        const double ego_vehicle_s, const double ego_vehicle_d)
    {
        double lane_width =
            ptr_reference_line_info_->reference_line().GetLaneWidth(ego_vehicle_s);
        return ego_vehicle_d < lane_width && ego_vehicle_d > -lane_width;
    }

    bool CollisionChecker::IsObstacleBehindEgoVehicle(
        const DetectedObject *obstacle, const double ego_vehicle_s,
        const std::vector<WayPoint> &discretized_reference_line)
    {
        double half_lane_width = FLAGS_default_reference_line_width * 0.5;
        WayPoint point = obstacle->GetPointAtTime(0.0);
        auto obstacle_reference_line_position =
            PlanningHelpers::GetPathFrenetCoordinate(
                discretized_reference_line, obstacle->center.pos);

        if (obstacle_reference_line_position.first < ego_vehicle_s &&
            std::fabs(obstacle_reference_line_position.second) < half_lane_width)
        {
            ADEBUG << "Ignore obstacle [" << obstacle->Id() << "]";
            return true;
        }
        return false;
    }

} // namespace PlannerHNS
