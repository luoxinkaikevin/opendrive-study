#include "op_planner/reference_line/frame.h"

namespace PlannerHNS
{

    // constexpr double kMathEpsilon = 1e-8;

    Frame::Frame(const WayPoint &planning_start_point,
                 const WayPoint &vehicle_state,
                 const ReferenceLine &reference_line)
        : planning_start_point_(planning_start_point),
          vehicle_state_(vehicle_state),
          reference_line_info_(
              vehicle_state_, planning_start_point_, reference_line)
    {
    }

    const WayPoint &Frame::PlanningStartPoint() const
    {
        return planning_start_point_;
    }

    const WayPoint &Frame::vehicle_state() const
    {
        return vehicle_state_;
    }

    bool Frame::Rerouting()
    {
        return true;
    }

    ReferenceLineInfo &Frame::reference_line_info()
    {
        return reference_line_info_;
    }


   //该函数被Planning::InitFrame函数调用，用于完成当前帧各类信息的初始化，
    //包括获取高精地图、车辆状态，创建预测障碍、目的地障碍，查找碰撞障碍，初始化参考线信息等。
    bool Frame::Init(const std::shared_ptr<std::vector<DetectedObject>> ptr_objects_in_lane)
    {

        const double start_timestamp = Clock::NowInSeconds();
        double current_time = start_timestamp;

        const auto &point =
            GPSPoint(vehicle_state_.pos.x, vehicle_state_.pos.y, 0, vehicle_state_.pos.a);
        if (std::isnan(point.x) || std::isnan(point.y))
        {
            AERROR << "init point is not set";
            return false;
        }

        for (std::size_t i = 0; i < ptr_objects_in_lane->size(); i++)
        {
            AddObstacle(ptr_objects_in_lane->at(i));
        }

        ADEBUG << "AddObstacle Time = "
               << (Clock::NowInSeconds() - current_time) * 1000<<" ms.";
        current_time = Clock::NowInSeconds();

        is_near_destination_ = reference_line_info_.reference_line().is_near_destination();
        //----------------------- 判断碰撞
        // Step B.1 检查当前时刻(relative_time=0.0s)，无人车位置和障碍物位置是否重叠(相撞)，如果是，可以直接退出
        const auto *collision_obstacle = FindCollisionObstacle();
        if (collision_obstacle)
        {
            std::string err_str =
                "Found collision with DetectedObject: " + collision_obstacle->Id();
            ADEBUG << err_str;
            return false;
        }

        ADEBUG << "FindCollisionObstacle Time = "
               << (Clock::NowInSeconds() - current_time) * 1000<<" ms.";
        current_time = Clock::NowInSeconds();

        // Step B.2 如果当前时刻不冲突，检查未来时刻无人车可以在参考线上前进的位置，ReferenceLineInfo生成
        //----------------------- 获取参考线信息
        if (!CreateReferenceLineInfo())
        {
            AERROR << "Failed to init reference line info";
            return false;
        }

        ADEBUG << "CreateReferenceLineInfo Time = "
               << (Clock::NowInSeconds() - current_time) * 1000<<" ms.";
        current_time = Clock::NowInSeconds();

        return true;
    }

    bool Frame::CreateReferenceLineInfo()
    {

        // 将障碍物投影到参考线上
        bool has_valid_reference_line = false;
        ADEBUG << "    ---------------------------------------------";
        if (!reference_line_info_.Init(obstacles()))
        {
            AERROR << "Failed to create reference line";
        }
        else
        {
            has_valid_reference_line = true;
        }
        ADEBUG << "    ---------------------------------------------";
        return has_valid_reference_line;
    }

    /**
 * @brief: create static virtual object with lane width,
 *         mainly used for virtual stop wall
 */
    const DetectedObject *Frame::CreateStopObstacle(
        ReferenceLineInfo *const reference_line_info,
        const std::string &obstacle_id, const double obstacle_s)
    {
        if (reference_line_info == nullptr)
        {
            AERROR << "reference_line_info nullptr";
            return nullptr;
        }

        const auto &reference_line = reference_line_info->reference_line();
        const double box_center_s = obstacle_s + FLAGS_virtual_stop_wall_length / 2.0;
        auto box_center = reference_line.GetNearestReferencePoint(box_center_s);
        double heading = reference_line.GetNearestReferencePoint(obstacle_s).pos.a;
        double lane_width = reference_line.GetLaneWidth(obstacle_s);


        math::Box2d stop_wall_box{
            box_center.pos, heading, FLAGS_virtual_stop_wall_length, lane_width};
        return CreateStaticVirtualObstacle(obstacle_id, stop_wall_box);
    }

    const DetectedObject *Frame::CreateStaticVirtualObstacle(const std::string &id,
                                                             const math::Box2d &box)
    {
        const auto *object = obstacles_.Find(id);
        if (object)
        {
            AWARN << "obstacle " << id << " already exist.";
            return object;
        }

        auto *ptr =
            obstacles_.Add(id, *CreateStaticVirtualObstacles(id, box));
        if (!ptr)
        {
            AERROR << "Failed to create virtual obstacle " << id;
        }
        return ptr;
    }

    std::unique_ptr<DetectedObject> Frame::CreateStaticVirtualObstacles(
        const std::string &id, const math::Box2d &obstacle_box)
    {
        // create a "virtual" perception_obstacle
        DetectedObject perception_obstacle;
        // simulator needs a valid integer
        int32_t negative_id = std::hash<std::string>{}(id);
        ADEBUG << "negative_id:" << negative_id;
        // set the first bit to 1 so negative_id became negative number
        negative_id |= (0x1 << 31);
        perception_obstacle.id = negative_id;
        perception_obstacle.center.pos = obstacle_box.center();
        perception_obstacle.center.pos.a = obstacle_box.heading();
        perception_obstacle.center.v = 0;
        perception_obstacle.l = obstacle_box.length();
        perception_obstacle.w = obstacle_box.width();
        perception_obstacle.h = FLAGS_virtual_stop_wall_height;

        std::vector<GPSPoint> corner_points;
        obstacle_box.GetAllCorners(&corner_points);
        perception_obstacle.contour.clear();
        for (std::size_t i = 0; i < corner_points.size(); i++)
        {
            perception_obstacle.contour.emplace_back(corner_points.at(i));
        }
        // perception_obstacle.set_perception_bounding_box(obstacle_box);
        perception_obstacle.set_Id(negative_id);
        perception_obstacle.set_polygon();

        auto *obstacle = new DetectedObject(perception_obstacle);
        obstacle->is_virtual_ = true;
        return std::unique_ptr<DetectedObject>(obstacle);
    }

 

    void Frame::AddObstacle(const DetectedObject &obstacle)
    {
        obstacles_.Add(obstacle.Id(), obstacle);
    }

    const DetectedObject *Frame::FindCollisionObstacle() const
    {
        if (obstacles_.Items().empty())
        {
            return nullptr;
        }

        math::Box2d adc_box(
            vehicle_state_.pos, vehicle_state_.pos.a, car_params.length, car_params.width);
        const double adc_half_diagnal = adc_box.diagonal() / 2.0;
        for (const auto &obstacle : obstacles_.Items())
        {
            if (obstacle->IsVirtual())
            {
                continue;
            }

            double center_dist = adc_box.center().DistanceTo(obstacle->center.pos);

            math::Box2d obj_box(
                obstacle->center.pos, obstacle->center.pos.a, obstacle->l, obstacle->w);

            if (center_dist > obj_box.diagonal() / 2.0 +
                                  adc_half_diagnal + FLAGS_max_collision_distance)
            {
                // ADEBUG << "Obstacle : " << obstacle->Id() << " is too far to collide";
                continue;
            }

            math::Polygon2d obj_polygon2d(obstacle->contour);
            double distance = obj_polygon2d.DistanceTo(adc_box);
            if (FLAGS_ignore_overlapped_obstacle && distance < kMathEpsilon)
            {
                bool all_points_in = true;
                for (const auto &point : obj_polygon2d.points())
                {
                    if (!adc_box.IsPointIn(point))
                    {
                        all_points_in = false;
                        break;
                    }
                }
                if (all_points_in)
                {
                    ADEBUG << "Skip overlapped obstacle, which is often caused by lidar "
                              "calibration error";
                    continue;
                }
            }
            if (distance < FLAGS_max_collision_distance)
            {
                AERROR << "Found collision with obstacle " << obstacle->Id();
                return obstacle;
            }
        }
        return nullptr;
    }

    std::string Frame::DebugString() const
    {
        // return "Frame: " + std::to_string(sequence_num_);
    }

    const std::vector<const DetectedObject *> Frame::obstacles() const
    {
        return obstacles_.Items();
    }

} // namespace PlannerHNS
