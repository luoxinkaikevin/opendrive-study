#pragma once

#include <cstdint>
#include <list>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>

#include "op_planner/reference_line/reference_line_info.h"
#include "op_planner/RoadNetwork.h"
#include "op_planner/RoadElement.h"
#include "op_planner/planning_gflags.h"
#include "op_planner/box2d.h"
#include "op_planner/reference_line/reference_line_base.h"
#include "op_planner/reference_line/path_decision.h"
#include "op_planner/log.h"
#include "op_planner/time.h"


namespace PlannerHNS
{
    /**
 * @class Frame
 *
 * @brief Frame holds all data for one planning cycle.
 */

    class Frame
    {
    public:
        //  frame_.reset(new Frame(sequence_num, planning_start_point, start_time,
        //                        vehicle_state, reference_line_provider_.get()));
        explicit Frame(const WayPoint &planning_start_point,
                       const WayPoint &vehicle_state,
                       const ReferenceLine &reference_line);

        const WayPoint &PlanningStartPoint() const;
        //该函数被Planning::InitFrame函数调用，用于完成当前帧各类信息的初始化，
        //包括获取高精地图、车辆状态，创建预测障碍、目的地障碍，查找碰撞障碍，初始化参考线信息等。
        bool Init(const std::shared_ptr<std::vector<DetectedObject>> ptr_objects_in_lane);

        std::string DebugString() const;

        const PublishableTrajectory &ComputedTrajectory() const;

        // void RecordInputDebug(planning_internal::Debug *debug);

        ReferenceLineInfo &reference_line_info();

        DetectedObject *Find(const std::string &id);

        const DetectedObject *CreateStopObstacle(
            ReferenceLineInfo *const reference_line_info,
            const std::string &obstacle_id, const double obstacle_s);

        bool Rerouting();

        const WayPoint &vehicle_state() const;

        const bool is_near_destination() const { return is_near_destination_; }
        const std::vector<const DetectedObject *> obstacles() const;

        IndexedList<std::string, DetectedObject> *GetObstacleList() { return &obstacles_; }

    private:
        bool CreateReferenceLineInfo();
        void ChangeLaneDecider();

        /**
   * Find an obstacle that collides with ADC (Autonomous Driving Car) if
   * such
   * obstacle exists.
   * @return pointer to the obstacle if such obstacle exists, otherwise
   * @return false if no colliding obstacle.
   */
        const DetectedObject *FindCollisionObstacle() const;

        /**
           * @brief create a static virtual obstacle
           */
        const DetectedObject *CreateStaticVirtualObstacle(const std::string &id,
                                                          const math::Box2d &box);

        void AddObstacle(const DetectedObject &obstacle);

        static std::unique_ptr<DetectedObject> CreateStaticVirtualObstacles(
            const std::string &id, const math::Box2d &obstacle_box);

    private:
        //轨迹起始点
        WayPoint planning_start_point_;
        //车辆状态
        WayPoint vehicle_state_;
        //参考线信息列表，存储多条候选路径
        ReferenceLine reference_line_;
        ReferenceLineInfo reference_line_info_;
        //接近终点标志位
        bool is_near_destination_ = false;
        //障碍物列表
        IndexedList<std::string, DetectedObject> obstacles_;

        CAR_BASIC_INFO car_params;
    };

} // namespace PlannerHNS