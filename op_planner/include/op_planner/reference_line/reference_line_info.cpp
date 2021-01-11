#include "op_planner/reference_line/reference_line_info.h"

namespace PlannerHNS
{

    LaneChangeState ReferenceLineInfo::lane_change_state;
    ReferenceLineInfo::ReferenceLineInfo(
        const WayPoint &current_state,
        const WayPoint &adc_planning_point,
        const ReferenceLine &reference_line)
        : vehicle_state_(current_state),
          adc_planning_point_(adc_planning_point),
          reference_line_(reference_line) {}

    bool ReferenceLineInfo::Init(
        const std::vector<const DetectedObject *> &obstacles)
    {
        //------------------------get adc sl boundary---------------------------
        const auto &path_point = vehicle_state_.pos;
        GPSPoint position(path_point.x, path_point.y, 0, 0);
        double ego_length = car_params.length;
        double ego_width = car_params.width;
        math::Box2d ego_box(
            position, path_point.a, ego_length, ego_width);
        double shift_distance = -car_params.wheel_base / 2.0;
        GPSPoint shift_vec(shift_distance * std::cos(path_point.a),
                           shift_distance * std::sin(path_point.a), 0, 0);
        ego_box.Shift(shift_vec);

        // -------------------get adc sl boundary
        if (!reference_line_.GetSLBoundary(ego_box, &adc_sl_boundary_))
        {
            AERROR << "Failed to get ADC boundary from box: " << ego_box.DebugString();
            return false;
        }
        ADEBUG << "Vehicle SL " << adc_sl_boundary_.start_s << "," << adc_sl_boundary_.end_s
               << "," << adc_sl_boundary_.start_l << "," << adc_sl_boundary_.end_l;

        //-------------------deal with adc abnormal situation
        if (adc_sl_boundary_.end_s < 0 ||
            adc_sl_boundary_.start_s > reference_line_.Length())
        {
            AWARN << "Vehicle SL " << adc_sl_boundary_.start_s << "," << adc_sl_boundary_.start_l
                  << "," << adc_sl_boundary_.end_s << "," << adc_sl_boundary_.end_l
                  << " is not on reference line:[0, " << reference_line_.Length()
                  << "]";
        }
        constexpr double kOutOfReferenceLineL = 10.0; //meters
        if (adc_sl_boundary_.start_l > kOutOfReferenceLineL ||
            adc_sl_boundary_.end_l < -kOutOfReferenceLineL)
        {
            AERROR << "Ego vehicle is too far away from reference line.";
            return false;
        }

        is_on_reference_line_ = reference_line_.IsOnRoad(adc_sl_boundary_);
        //------------------------------------------
        if (!AddObstacles(obstacles))
        {
            AERROR << "Failed to add obstacles to reference line";
            return false;
        }

        //----------------------set lattice planning target speed limit;
        auto cruise_speed = reference_line_.current_reference_pose().v;
        slamLocationCheck(cruise_speed);
        SetCruiseSpeed(cruise_speed);
        LaneChangeStateProcess(cruise_speed);
        is_inited_ = true;
        return true;
    }

    void ReferenceLineInfo::slamLocationCheck(double &cruise_speed)
    {

        if (reference_line_.speed_limit_1 < 0.12 &&
            reference_line_.speed_limit_2 < 0.12)
        {
            return;
        }
        // else if ((reference_line_.speed_limit_1 > 0.12 && reference_line_.speed_limit_1 < 0.15) ||
        //          (reference_line_.speed_limit_2 > 0.12 && reference_line_.speed_limit_2 < 0.15))
        // {
        //     ADEBUG << "SLAM location not precise, speed_limit_1: " << reference_line_.speed_limit_1
        //            << "speed_limit_2 " << reference_line_.speed_limit_2;
            
        //     if (cruise_speed > 5)
        //         cruise_speed = 5;
        // }
        // else if ((reference_line_.speed_limit_1 > 0.15 && reference_line_.speed_limit_1 < 0.20) ||
        //          (reference_line_.speed_limit_2 > 0.15 && reference_line_.speed_limit_2 < 0.20))
        // {
        //     ADEBUG << "SLAM location not precise, speed_limit_1: " << reference_line_.speed_limit_1
        //            << "speed_limit_2 " << reference_line_.speed_limit_2;
            
        //     if (cruise_speed > 3)
        //         cruise_speed = 3;
        // }
        else
        {
            cruise_speed *= 0.85;
        }
    }

    void ReferenceLineInfo::LaneChangeStateProcess(double cruise_speed)
    {
        ADEBUG << "lane change initial state: " << getLaneChangeState(lane_change_state);
        if (lane_change_state == CHANGE_DISABLE)
        {
            is_safe_to_change_lane_ = CheckChangeLaneState(cruise_speed);
            if (is_safe_to_change_lane_)
            {
                if (is_safe_to_change_left_)
                    lane_change_state = READY_TO_CHANGE_LEFT;
                else if (is_safe_to_change_right_)
                    lane_change_state = READY_TO_CHANGE_RIGHT;
                else
                    lane_change_state = CHANGE_DISABLE;
            }
        }
        else if (lane_change_state == READY_TO_CHANGE_LEFT)
        {
            if (adc_sl_boundary_.start_l > reference_line_.current_reference_pose().laneWidth * 0.5)
                lane_change_state = CHANGE_LEFT_SUCCESS;
        }
        else if (lane_change_state == READY_TO_CHANGE_RIGHT)
        {
            if (adc_sl_boundary_.end_l < -reference_line_.current_reference_pose().laneWidth * 0.5)
                lane_change_state = CHANGE_RIGHT_SUCCESS;
        }
        else if (lane_change_state == CHANGE_LEFT_SUCCESS)
        {
            auto change_back_obsatacle_condition = CheckLaneChangeBackState(true);
            // if change_back_obsatacle_condition satisfied, change lane
            //else go forward and wait in change lane end
            if (change_back_obsatacle_condition)
            {
                lane_change_state = READY_TO_CHANGE_BACK_FROM_LEFT;
            }
            else
            {
                auto current_index_ = reference_line_.GetNearestReferenceIndex(0);
                auto enforce_change_back_index = EnforceChangeBack(current_index_);
                if (enforce_change_back_index != -1)
                {
                    ADEBUG << "current_index: " << current_index_
                           << "enforce_change_back_index: " << enforce_change_back_index;
                    StopPoint stop_point;
                    stop_point.s =
                        reference_line_.reference_points().at(enforce_change_back_index).s() +
                        FLAGS_virtual_stop_wall_length / 2.0;
                    SetStopPoint(stop_point);
                    if (current_index_ >=
                        enforce_change_back_index - 25)
                    {
                        lane_change_state = ENFORCE_CHANGE_BACK_FROM_LEFT;
                    }
                }
            }
        }
        else if (lane_change_state == ENFORCE_CHANGE_BACK_FROM_LEFT)
        {
            auto change_back_obsatacle_condition = CheckLaneChangeBackState(true);
            // if change_back_obsatacle_condition satisfied, change lane
            //else go forward and wait in change lane end
            if (change_back_obsatacle_condition)
            {
                lane_change_state = READY_TO_CHANGE_BACK_FROM_LEFT;
            }
            else
            {
                StopPoint stop_point;
                stop_point.s =
                    reference_line_.current_reference_pose().s() +
                    FLAGS_virtual_stop_wall_length / 2.0;
                SetStopPoint(stop_point);
            }
        }
        else if (lane_change_state == CHANGE_RIGHT_SUCCESS)
        {

            auto change_back_obsatacle_condition = CheckLaneChangeBackState(false);
            // if change_back_obsatacle_condition satisfied, change lane
            //else go forward and wait in change lane end
            if (change_back_obsatacle_condition)
            {
                lane_change_state = READY_TO_CHANGE_BACK_FROM_RIGHT;
            }
            else
            {
                auto current_index_ = reference_line_.GetNearestReferenceIndex(0);
                auto enforce_change_back_index = EnforceChangeBack(current_index_);
                if (enforce_change_back_index != -1)
                {
                    ADEBUG << "current_index: " << current_index_
                           << "enforce_change_back_index: " << enforce_change_back_index;
                    StopPoint stop_point;
                    stop_point.s =
                        reference_line_.reference_points().at(enforce_change_back_index).s() +
                        FLAGS_virtual_stop_wall_length / 2.0;
                    SetStopPoint(stop_point);
                    if (current_index_ >= enforce_change_back_index - 25)
                    {
                        lane_change_state = ENFORCE_CHANGE_BACK_FROM_RIGHT;
                    }
                }
            }
        }
        else if (lane_change_state == ENFORCE_CHANGE_BACK_FROM_RIGHT)
        {

            auto change_back_obsatacle_condition = CheckLaneChangeBackState(false);
            // if change_back_obsatacle_condition satisfied, change lane
            //else go forward and wait in change lane end
            if (change_back_obsatacle_condition)
            {
                lane_change_state = READY_TO_CHANGE_BACK_FROM_RIGHT;
            }
            else
            {
                StopPoint stop_point;
                stop_point.s =
                    reference_line_.current_reference_pose().s() +
                    FLAGS_virtual_stop_wall_length / 2.0;
                SetStopPoint(stop_point);
            }
        }
        else if (lane_change_state == READY_TO_CHANGE_BACK_FROM_LEFT)
        {
            if (adc_sl_boundary_.end_l < reference_line_.current_reference_pose().laneWidth * 0.5)
                lane_change_state = CHANGE_DISABLE;
        }
        else if (lane_change_state == READY_TO_CHANGE_BACK_FROM_RIGHT)
        {
            if (adc_sl_boundary_.start_l > -reference_line_.current_reference_pose().laneWidth * 0.5)
                lane_change_state = CHANGE_DISABLE;
        }
        ADEBUG << "lane change end state: " << getLaneChangeState(lane_change_state);
    }

    bool ReferenceLineInfo::IsInited() const { return is_inited_; }

    int ReferenceLineInfo::EnforceChangeBack(int current_index)
    {
        const double enforce_change_back_s = 5.0;
        int change_lane_end_index = -1;
        for (int i = current_index;
             i < reference_line_.reference_points().size() - 1; i++)
        {
            if (lane_change_state == CHANGE_LEFT_SUCCESS)
            {
                if (reference_line_.reference_points().at(i).pLeft == nullptr &&
                    reference_line_.reference_points().at(i + 1).pLeft == nullptr)
                {
                    ADEBUG << "Find enforce left lane change point: " << i;
                    change_lane_end_index = i;
                    break;
                }
            }
            else if (lane_change_state == CHANGE_RIGHT_SUCCESS)
            {
                if (reference_line_.reference_points().at(i).pRight == nullptr &&
                    reference_line_.reference_points().at(i + 1).pRight == nullptr)
                {
                    ADEBUG << "Find enforce right lane change point: " << i;
                    change_lane_end_index = i;
                    break;
                }
            }
        }
        return change_lane_end_index;
    }

    bool ReferenceLineInfo::CheckLaneChangeBackState(bool left_lane_change_back)
    {
        double front_min_s = std::numeric_limits<double>::max();
        double back_min_s = -1000;
        double front_min_start_l = std::numeric_limits<double>::max();
        double front_min_end_l = std::numeric_limits<double>::max();
        double back_min_start_l = -1000;
        double back_min_end_l = -1000;
        double front_obstacle_speed = std::numeric_limits<double>::max();
        double back_obstacle_speed = std::numeric_limits<double>::max();
        int front_obstacle_type = 0;
        int back_obstacle_type = 0;
        std::string front_obstacle_index;
        std::string back_obstacle_index;
        auto kLateralShift =
            reference_line_.current_reference_pose().laneWidth * 0.5;
        bool has_front_obstacle = false;
        bool has_back_obstacle = false;
        bool front_obstacle_dangerous = false;
        bool back_obstacle_dangerous = false;

        for (const auto *path_obstacle : path_decision_.path_obstacles().Items())
        {
            //------caculate lane change condition in reference lane
            const auto &sl_boundary = path_obstacle->PerceptionSLBoundary();

            if (left_lane_change_back)
            {
                if (sl_boundary.start_l > kLateralShift)
                {
                    continue;
                }

                if (sl_boundary.end_l < -kLateralShift)
                {
                    continue;
                }
            }
            else
            {
                if (sl_boundary.end_l < -kLateralShift)
                {
                    continue;
                }

                if (sl_boundary.start_l > kLateralShift)
                {
                    continue;
                }
            }

            // front
            if (front_min_s > sl_boundary.start_s &&
                sl_boundary.end_s > adc_sl_boundary_.start_s)
            {
                has_front_obstacle = true;
                front_min_s = sl_boundary.start_s;
                front_min_start_l = sl_boundary.start_l;
                front_min_end_l = sl_boundary.end_l;
                front_obstacle_index = path_obstacle->Id();
                front_obstacle_type = path_obstacle->obstacle()->label;
                front_obstacle_speed = path_obstacle->obstacle()->Speed();
            }

            // back
            if (back_min_s < sl_boundary.start_s &&
                sl_boundary.end_s < adc_sl_boundary_.start_s)
            {
                has_back_obstacle = true;
                back_min_s = sl_boundary.start_s;
                back_min_start_l = sl_boundary.start_l;
                back_min_end_l = sl_boundary.start_l;
                back_obstacle_index = path_obstacle->Id();
                back_obstacle_type = path_obstacle->obstacle()->label;
                back_obstacle_speed = path_obstacle->obstacle()->Speed();
            }
        }

        if (has_front_obstacle)
        {
            const double kForwardSafeDistance =
                std::max(FLAGS_forward_min_safe_distance,
                         static_cast<double>((adc_planning_point_.v -
                                              front_obstacle_speed) *
                                             FLAGS_safe_time));
            if (front_min_s < kForwardSafeDistance)
            {
                // disable lane change back for car in both lane
                if (front_obstacle_type == 1)
                {
                    front_obstacle_dangerous = true;
                }
                else
                {
                    if (front_min_start_l < -kLateralShift + car_params.width + 0.2)
                    {
                        front_obstacle_dangerous = true;
                    }
                    else if (front_min_end_l < kLateralShift - car_params.width - 0.2)
                    {
                        front_obstacle_dangerous = true;
                    }
                }
            }

            ADEBUG << "Minimum distance in reference lane, front obstacle id: " << front_obstacle_index
                   << " ;front_min s: " << front_min_s << " ;type: " << front_obstacle_type
                   << " ;obstacle_speed: " << front_obstacle_speed;
        }

        if (has_back_obstacle)
        {
            // const double kBackwardSafeDistance =
            //     std::max(FLAGS_backward_min_safe_distance,
            //              static_cast<double>((back_obstacle_speed -
            //                                   adc_planning_point_.v) *
            //                                  FLAGS_safe_time));
            const double kBackwardSafeDistance =
                std::max(FLAGS_backward_min_safe_distance,
                         static_cast<double>((std::abs(back_obstacle_speed) + 1) * (FLAGS_safe_time + 2)));
            if (back_min_s < -kBackwardSafeDistance)
            {
                back_obstacle_dangerous = true;
            }

            ADEBUG << "Minimum distance in reference lane, back obstacle id: " << back_obstacle_index
                   << " ;back_min s: " << back_min_s
                   << " ;type: " << back_obstacle_type << " ;obstacle_speed: " << back_obstacle_speed;
        }

        if (back_obstacle_dangerous || front_obstacle_dangerous)
        {
            return false;
        }

        return true;
    }

    bool ReferenceLineInfo::CheckChangeLaneState(double cruise_speed)
    {
        //check lane change
        if (reference_line_.current_reference_pose().pLeft == nullptr &&
            reference_line_.current_reference_pose().pRight == nullptr)
        {
            return false;
        }

        double min_s = std::numeric_limits<double>::max();
        double obstacle_speed = std::numeric_limits<double>::max();
        int obstacle_type = 0;
        std::string obstacle_index = "";
        auto kLateralShift =
            reference_line_.current_reference_pose().laneWidth * 0.5;
        bool has_lane_change_obstacle = false;
        for (const auto *path_obstacle : path_decision_.path_obstacles().Items())
        {
            //------caculate lane change condition in reference lane
            const auto &sl_boundary = path_obstacle->PerceptionSLBoundary();

            // find the nearest obstacle in front of adc in reference lane
            if (min_s > sl_boundary.start_s &&
                sl_boundary.start_s > adc_sl_boundary_.end_s &&
                sl_boundary.start_l > -kLateralShift &&
                sl_boundary.end_l < kLateralShift)
            {
                has_lane_change_obstacle = true;
                min_s = sl_boundary.start_s;
                obstacle_index = path_obstacle->Id();
                obstacle_type = path_obstacle->obstacle()->label;
                obstacle_speed = path_obstacle->obstacle()->Speed();
            }
        }

        if (!has_lane_change_obstacle)
        {
            return false;
        }

        ADEBUG << "Minimum distance in reference lane, obstacle id: " << obstacle_index
               << " ;min s: " << min_s
               << " ;type: " << obstacle_type
               << " ;obstacle_speed: " << obstacle_speed;
        const double kForwardMinSafeDistance = 6;
        const double kForwardSafeDistance =
            std::max(FLAGS_forward_min_safe_distance,
                     static_cast<double>((adc_planning_point_.v -
                                          obstacle_speed) *
                                         FLAGS_safe_time));

        bool lane_change_flag = false;
        //only consider car in front of adc

        if (min_s < kForwardSafeDistance &&
            min_s > kForwardMinSafeDistance &&
            obstacle_type == 1)
        {
            // if current adc speed if slow or obstacle speed is high disable lane change action
            if (obstacle_speed >= cruise_speed * 0.5)
            {
                return false;
            }

            ADEBUG << "Enable lane change, obstacle id: " << obstacle_index
                   << " ;min s: " << min_s
                   << " ;type: " << obstacle_type
                   << " ;obstacle_speed: " << obstacle_speed;
            lane_change_flag = true;
        }

        if (!lane_change_flag)
        {
            return false;
        }

        //enable left lane change
        if (reference_line_.current_reference_pose().pLeft)
        {
            //------check  the situation of other lane to keep changing safe
            auto left_lane_state = checkLeftAnaRightLane(3, 1);

            if (left_lane_state)
            {
                is_safe_to_change_left_ = true;
                ADEBUG << "left lane safe. enable left lane change";
                return true;
            }
        }
        else
        {
            ADEBUG << "left lane is nullptr";
        }

        if (reference_line_.current_reference_pose().pRight)
        {
            auto right_lane_state = checkLeftAnaRightLane(-1, -3);

            if (right_lane_state)
            {
                is_safe_to_change_right_ = true;
                ADEBUG << "right lane safe. enable right lane change";
                return true;
            }
        }
        else
        {
            ADEBUG << "right lane is nullptr";
        }

        return false;
    }

    bool ReferenceLineInfo::checkLeftAnaRightLane(double left_l_range, double right_l_range)
    {
        for (const auto *path_obstacle : path_decision_.path_obstacles().Items())
        {
            auto kLateralShift =
                reference_line_.current_reference_pose().laneWidth * 0.5;
            const auto &sl_boundary = path_obstacle->PerceptionSLBoundary();
            // if obstacle  in referenec line
            if (sl_boundary.start_l > -kLateralShift &&
                sl_boundary.end_l < kLateralShift)
            {
                continue;
            }
            // if obastcle out of road
            if (sl_boundary.start_l > kLateralShift * left_l_range ||
                sl_boundary.end_l < kLateralShift * right_l_range)
            {
                continue;
            }

            const double kForwardSafeDistance =
                std::max(FLAGS_forward_min_safe_distance,
                         static_cast<double>((adc_planning_point_.v -
                                              path_obstacle->obstacle()->center.v) *
                                             FLAGS_safe_time));
            const double kBackwardSafeDistance =
                std::max(FLAGS_backward_min_safe_distance,
                         static_cast<double>((path_obstacle->obstacle()->center.v -
                                              adc_planning_point_.v) *
                                             FLAGS_safe_time));

            //如果障碍物在其他车道上，并且在adv前后方一定区域内
            if (sl_boundary.end_s >
                    adc_sl_boundary_.start_s - kBackwardSafeDistance &&
                sl_boundary.start_s <
                    adc_sl_boundary_.end_s + kForwardSafeDistance * 2)
            {

                ADEBUG << "lane not safe, obstacle in lane change range,id: "
                       << path_obstacle->Id() << " sl_boundary.end_s " << sl_boundary.end_s
                       << " kBackwardSafeDistance " << kBackwardSafeDistance
                       << " start_s " << sl_boundary.start_s
                       << " kForwardSafeDistance " << kForwardSafeDistance;
                return false;
            }
        }
        return true;
    }

    // AddObstacle is thread safe
    PathObstacle *ReferenceLineInfo::AddObstacle(const DetectedObject *obstacle)
    {
        if (!obstacle)
        {
            AERROR << "The provided obstacle is empty";
            return nullptr;
        }
        // create PathObstacle and add it into path_decision_
        auto *path_obstacle = path_decision_.AddPathObstacle(PathObstacle(obstacle));

        if (!path_obstacle)
        {
            AERROR << "failed to add obstacle " << obstacle->Id();
            return nullptr;
        }

        SLBoundary perception_sl;
        if (!reference_line_.GetSLBoundary(obstacle->PerceptionPolygon(), &perception_sl))
        {
            AERROR << "Failed to get sl boundary for obstacle: " << obstacle->Id();
            return path_obstacle;
        }
        // only obstacles accepted by perception have PerceptionSlBoundary
        path_obstacle->SetPerceptionSlBoundary(perception_sl);

        // ignore back and far away obstacle
        if (IsUnrelaventObstacle(path_obstacle))
        {
            ObjectDecision ignore_decision;
            ignore_decision.decision_type = ignore;
            path_decision_.AddLateralDecision("reference_line_filter", obstacle->Id(),
                                              ignore_decision);
            path_decision_.AddLongitudinalDecision("reference_line_filter",
                                                   obstacle->Id(), ignore_decision);
            // ADEBUG << "NO build reference line st boundary. id:" << obstacle->Id();
        }
        else
        {
            // ADEBUG << "Build reference line st boundary. id:" << obstacle->Id();
            path_obstacle->BuildReferenceLineStBoundary(reference_line_,
                                                        adc_sl_boundary_.start_s);

            // ADEBUG << "reference line st boundary: "
            //        << path_obstacle->reference_line_st_boundary().min_t() << ", "
            //        << path_obstacle->reference_line_st_boundary().max_t()
            //        << ", s_max: " << path_obstacle->reference_line_st_boundary().max_s()
            //        << ", s_min: " << path_obstacle->reference_line_st_boundary().min_s()
            //        << ", start_l: " << perception_sl.start_l
            //        << ", end_l: " << perception_sl.end_l;
        }
        return path_obstacle;
    }

    bool ReferenceLineInfo::AddObstacles(
        const std::vector<const DetectedObject *> &obstacles)
    {
        ADEBUG << "Perception obstacles size: " << obstacles.size();
        for (const auto *obstacle : obstacles)
        {
            if (!AddObstacle(obstacle))
            {
                AERROR << "Failed to add obstacle " << obstacle->Id();
                return false;
            }
        }
        return true;
    }

    bool ReferenceLineInfo::IsUnrelaventObstacle(PathObstacle *path_obstacle)
    {
        // if obstacle far from adc
        if (path_obstacle->PerceptionSLBoundary().end_s > reference_line_.Length())
        {
            return true;
        }

        // if (!reference_line_.IsOnRoad(path_obstacle->PerceptionSLBoundary()) &&
        //     !path_obstacle->obstacle()->HasTrajectory())
        // {
        //     return true;
        // }

        // if adc is on the road, and obstacle is on the same lane and behind adc, ignore
        if (is_on_reference_line_ &&
            path_obstacle->PerceptionSLBoundary().end_s < adc_sl_boundary_.start_s &&
            reference_line_.IsOnRoad(path_obstacle->PerceptionSLBoundary()))
        {
            return true;
        }
        return false;
    }

    const DiscretizedTrajectory &ReferenceLineInfo::trajectory() const
    {
        return discretized_trajectory_;
    }

    double ReferenceLineInfo::TrajectoryLength() const
    {
        const auto &tps = discretized_trajectory_.trajectory_points();
        if (tps.empty())
        {
            return 0.0;
        }
        return tps.back().s();
    }

    void ReferenceLineInfo::SetStopPoint(const StopPoint &stop_point)
    {
        planning_target_.has_stop_point = true;
        planning_target_.stop_point = stop_point;
    }

    void ReferenceLineInfo::SetCruiseSpeed(double speed)
    {
        planning_target_.cruise_speed = speed;
    }

    bool ReferenceLineInfo::IsStartFrom(
        const ReferenceLineInfo &previous_reference_line_info) const
    {
        if (reference_line_.reference_points().empty())
        {
            return false;
        }
        auto start_point = reference_line_.reference_points().front();
        const auto &prev_reference_line =
            previous_reference_line_info.reference_line();

        auto sl_point =
            PlanningHelpers::GetPathFrenetCoordinate(
                previous_reference_line_info.reference_line_.reference_points(), start_point.pos);

        return previous_reference_line_info.reference_line_.IsOnRoad(sl_point);
    }

    void ReferenceLineInfo::SetDrivable(bool drivable) { is_drivable_ = drivable; }

    bool ReferenceLineInfo::IsDrivable() const { return is_drivable_; }

    const SLBoundary &ReferenceLineInfo::AdcSlBoundary() const
    {
        return adc_sl_boundary_;
    }

    PathDecision *ReferenceLineInfo::path_decision() { return &path_decision_; }

    const PathDecision &ReferenceLineInfo::path_decision() const
    {
        return path_decision_;
    }
    const WayPoint &ReferenceLineInfo::AdcPlanningPoint() const
    {
        return adc_planning_point_;
    }

    const ReferenceLine &ReferenceLineInfo::reference_line() const
    {
        return reference_line_;
    }

    void ReferenceLineInfo::SetTrajectory(const DiscretizedTrajectory &trajectory)
    {
        discretized_trajectory_ = trajectory;
    }

    void ReferenceLineInfo::AddObstacleHelper(const DetectedObject *obstacle, int *ret)
    {
        auto *path_obstacle = AddObstacle(obstacle);
        *ret = path_obstacle == nullptr ? 0 : 1;
    }

    bool ReferenceLineInfo::ReachedDestination() const
    {
    }

    void ReferenceLineInfo::ExportDecision(DecisionResult *decision_result) const
    {
        MakeDecision(decision_result);
    }

    void ReferenceLineInfo::MakeDecision(DecisionResult *decision_result) const
    {
        CHECK_NOTNULL(decision_result);
        // cruise by default
        decision_result->main_decision.main_decision_type = MISSION_CRUISE;

        // // check stop decision
        // int error_code = MakeMainStopDecision(decision_result);
        // if (error_code < 0)
        // {
        //     MakeEStopDecision(decision_result);
        // }
        MakeMainMissionCompleteDecision(decision_result);
        SetObjectDecisions(decision_result->object_decision);
    }

    void ReferenceLineInfo::MakeMainMissionCompleteDecision(
        DecisionResult *decision_result) const
    {
        if (!(decision_result->main_decision.main_decision_type == STOP))
        {
            return;
        }
        auto main_stop = decision_result->main_decision.stop;
        if (main_stop.reason_code != STOP_REASON_DESTINATION)
        {
            return;
        }
        const auto &adc_pos = AdcPlanningPoint().pos;
        if (adc_pos.DistanceTo(main_stop.stop_point) > FLAGS_destination_check_distance)
        {
            return;
        }

        auto mission_complete = decision_result->main_decision.mission_complete;
        if (!ReachedDestination())
        {
            decision_result->main_decision.mission_complete.stop_point = main_stop.stop_point;
        }
    }

    int ReferenceLineInfo::MakeMainStopDecision(
        DecisionResult *decision_result) const
    {
        double min_stop_line_s = std::numeric_limits<double>::infinity();
        const DetectedObject *stop_obstacle = nullptr;
        const ObjectDecision *stop_decision = nullptr;

        for (const auto path_obstacle : path_decision_.path_obstacles().Items())
        {
            const auto &obstacle = path_obstacle->obstacle();
            const auto &object_decision = path_obstacle->LongitudinalDecision();
            if (!object_decision.decision_type == stop)
            {
                continue;
            }

            GPSPoint stop_point = object_decision.stop_point;
            auto stop_line_sl =
                PlanningHelpers::GetPathFrenetCoordinate(
                    reference_line_.reference_points(), stop_point);

            double stop_line_s = stop_line_sl.first;
            if (stop_line_s < 0 || stop_line_s > reference_line_.Length())
            {
                AERROR << "Ignore object:" << obstacle->Id() << " fence route_s["
                       << stop_line_s << "] not in range[0, " << reference_line_.Length()
                       << "]";
                continue;
            }

            // check stop_line_s vs adc_s
            if (stop_line_s < min_stop_line_s)
            {
                min_stop_line_s = stop_line_s;
                stop_obstacle = obstacle;
                stop_decision = &object_decision;
            }
        }

        if (stop_obstacle != nullptr)
        {
            decision_result->main_decision.main_decision_type = STOP;
            MainStop *main_stop = &(decision_result->main_decision.stop);
            main_stop->stop_point = stop_obstacle->center.pos;
            // main_stop->set_reason_code(stop_decision->reason_code());
            // main_stop->set_reason("stop by " + stop_obstacle->Id());

            // ADEBUG << " main stop obstacle id:" << stop_obstacle->Id()
            //        << " stop_line_s:" << min_stop_line_s << " stop_point: ("
            //        << stop_decision->stop_point().x() << stop_decision->stop_point().y()
            //        << " ) stop_heading: " << stop_decision->stop_heading();

            return 1;
        }

        return 0;
    }

    void ReferenceLineInfo::SetObjectDecisions(
        std::vector<ObjectDecision> &object_decisions) const
    {
        for (const auto path_obstacle : path_decision_.path_obstacles().Items())
        {
            if (!path_obstacle->HasNonIgnoreDecision())
            {
                continue;
            }
            // auto *object_decision = object_decisions->add_decision();

            const auto &obstacle = path_obstacle->obstacle();

            // object_decision->set_id(obstacle->Id());
            // object_decision->set_perception_id(obstacle->PerceptionId());
            if (path_obstacle->HasLateralDecision() &&
                !path_obstacle->IsLateralIgnore())
            {
                object_decisions.emplace_back(path_obstacle->LateralDecision());
                // object_decision->add_object_decision()->CopyFrom(
                //     path_obstacle->LateralDecision());
            }
            if (path_obstacle->HasLongitudinalDecision() &&
                !path_obstacle->IsLongitudinalIgnore())
            {
                object_decisions.emplace_back(path_obstacle->LongitudinalDecision());
                // object_decision->add_object_decision()->CopyFrom(
                //     path_obstacle->LongitudinalDecision());
            }
        }
    }
    std::string ReferenceLineInfo::getLaneChangeState(LaneChangeState lane_change_state)
    {
        switch (lane_change_state)
        {
        case READY_TO_CHANGE_LEFT:
            return "ready to change left";
            break;
        case READY_TO_CHANGE_RIGHT:
            return "ready to change right";
            break;
        case CHANGE_LEFT_SUCCESS:
            return "change left success";
            break;
        case CHANGE_RIGHT_SUCCESS:
            return "change right success";
            break;
        case READY_TO_CHANGE_BACK_FROM_LEFT:
            return "ready to change back from left";
            break;
        case READY_TO_CHANGE_BACK_FROM_RIGHT:
            return "ready to change back from right";
            break;
        case ENFORCE_CHANGE_BACK_FROM_LEFT:
            return "enforce change back from left";
            break;
        case ENFORCE_CHANGE_BACK_FROM_RIGHT:
            return "enfore change back from right";
            break;
        default:
            return "Disable lane change";
            break;
        }
    }

} // namespace PlannerHNS