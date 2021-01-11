#pragma once

#include <algorithm>
#include <limits>
#include <list>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>
#include <functional>
#include <utility>

#include "op_planner/RoadNetwork.h"
#include "op_planner/box2d.h"
#include "op_planner/reference_line/reference_line_base.h"
#include "op_planner/reference_line/path_obstacle.h"
#include "op_planner/reference_line/path_decision.h"
#include "op_planner/PolynomialCurve.h"
#include "op_planner/planning_gflags.h"
#include "op_planner/RoadElement.h"
#include "op_planner/log.h"

namespace PlannerHNS
{

    enum LaneChangeState
    {
        CHANGE_DISABLE,
        READY_TO_CHANGE_LEFT,
        READY_TO_CHANGE_RIGHT,
        CHANGE_LEFT_SUCCESS,
        CHANGE_RIGHT_SUCCESS,
        CHANGE_LEFT_FAILURE,
        CHANGE_RIGHT_FAILURE,
        READY_TO_CHANGE_BACK_FROM_LEFT,
        READY_TO_CHANGE_BACK_FROM_RIGHT,
        ENFORCE_CHANGE_BACK_FROM_LEFT,
        ENFORCE_CHANGE_BACK_FROM_RIGHT,
    };

    /**
 * @class ReferenceLineInfo
 * @brief ReferenceLineInfo holds all data for one reference line.
 */
    class ReferenceLineInfo
    {
    public:
        explicit ReferenceLineInfo(
            const WayPoint &current_state,
            const WayPoint &adc_planning_point,
            const ReferenceLine &reference_line);

        std::string getLaneChangeState(LaneChangeState lane_change_state);

        bool Init(
            const std::vector<const DetectedObject *> &obstacles);

        bool IsInited() const;

        bool AddObstacles(const std::vector<const DetectedObject *> &obstacles);
        PathObstacle *AddObstacle(const DetectedObject *obstacle);
        void AddObstacleHelper(const DetectedObject *obstacle, int *ret);

        PathDecision *path_decision();
        const PathDecision &path_decision() const;
        const ReferenceLine &reference_line() const;
        const WayPoint &AdcPlanningPoint() const;

        bool ReachedDestination() const;

        void SetTrajectory(const DiscretizedTrajectory &trajectory);

        const DiscretizedTrajectory &trajectory() const;
        double TrajectoryLength() const;

        // For lattice planner'speed planning target
        void SetStopPoint(const StopPoint &stop_point);
        void SetCruiseSpeed(double speed);
        const PlanningTarget &planning_target() const { return planning_target_; }

        /**
   * @brief check if current reference line is started from another reference
   *line info line. The method is to check if the start point of current
   *reference line is on previous reference line info.
   * @return returns true if current reference line starts on previous reference
   *line, otherwise false.
   **/
        bool IsStartFrom(const ReferenceLineInfo &previous_reference_line_info) const;

        const SLBoundary &AdcSlBoundary() const;
        std::string PathSpeedDebugString() const;

        /**
   * Check if the current reference line is a change lane reference line, i.e.,
   * ADC's current position is not on this reference line.
   */
        bool IsChangeLanePath() const;

        /**
   * Check if the current reference line is the neighbor of the vehicle 
   * current position
   */
        bool IsNeighborLanePath() const;

        /**
   * Set if the vehicle can drive following this reference line
   * A planner need to set this value to true if the reference line is OK
   */
        void SetDrivable(bool drivable);
        bool IsDrivable() const;

        // void ExportEngageAdvice(common::EngageAdvice *engage_advice) const;

        bool IsSafeToChangeLane() const { return is_safe_to_change_lane_; }
        bool IsSafeToChangeLeft() const { return is_safe_to_change_left_; }
        bool IsSafeToChangeRight() const { return is_safe_to_change_right_; }

        void ExportDecision(DecisionResult *decision_result) const;

        bool IsRightTurnPath() const;

        double OffsetToOtherReferenceLine() const
        {
            return offset_to_other_reference_line_;
        }
        void SetOffsetToOtherReferenceLine(const double offset)
        {
            offset_to_other_reference_line_ = offset;
        }

        void set_is_on_reference_line() { is_on_reference_line_ = true; }

        bool CheckChangeLaneState(double cruise_speed);
        bool CheckLaneChangeBackState(bool left_lane_change_back);
        int EnforceChangeBack(int current_index);
        void LaneChangeStateProcess(double cruise_speed);
        static LaneChangeState lane_change_state;

        const WayPoint vehicle_state() const { return vehicle_state_; }

    private:
        bool CheckChangeLane() const;

        bool checkLeftAnaRightLane(double left_l_range, double right_l_range);

        bool IsUnrelaventObstacle(PathObstacle *path_obstacle);

        void MakeDecision(DecisionResult *decision_result) const;
        int MakeMainStopDecision(DecisionResult *decision_result) const;
        void MakeMainMissionCompleteDecision(DecisionResult *decision_result) const;
        void MakeEStopDecision(DecisionResult *decision_result) const;
        void SetObjectDecisions(std::vector<ObjectDecision> &object_decisions) const;
        void slamLocationCheck(double &cruise_speed);
        //车辆状态
        const WayPoint vehicle_state_;
        //局部轨迹
        WayPoint adc_planning_point_;
        //参考线
        ReferenceLine reference_line_;

        /**
   * @brief this is the number that measures the goodness of this reference
   * line. The lower the better.
   */
        double cost_ = 0.0;
        //是否初始化
        bool is_inited_ = false;
        //是否可驾驶
        bool is_drivable_ = true;
        //change lane state machine
        // int change_lane_state_ = 0;
        //路径决策
        PathDecision path_decision_;
        //离散化轨迹
        DiscretizedTrajectory discretized_trajectory_;
        //ls边界
        SLBoundary adc_sl_boundary_;
        //是否在参考线上
        bool is_on_reference_line_ = false;
        //换道是否安全
        bool is_safe_to_change_lane_ = false;
        bool is_safe_to_change_left_ = false;
        bool is_safe_to_change_right_ = false;

        //到其他参考线的补偿
        double offset_to_other_reference_line_ = 0.0;
        //优先级代价
        double priority_cost_ = 0.0;
        //规划目标
        PlanningTarget planning_target_;

        CAR_BASIC_INFO car_params;
    };

} // namespace PlannerHNS
