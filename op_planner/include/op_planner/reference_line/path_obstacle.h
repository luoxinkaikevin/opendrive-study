#pragma once

#include <list>
#include <string>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <limits>
#include <utility>

#include "op_planner/planning_gflags.h"
#include "op_planner/RoadNetwork.h"
#include "op_planner/PlanningHelpers.h"
#include "op_planner/RoadElement.h"
#include "op_planner/reference_line/reference_line_base.h"
#include "op_planner/reference_line/st_boundary.h"

namespace PlannerHNS
{
    /**
 * @class PathObstacle
 * @brief This is the class that associates an Obstacle with its path
 * properties. An obstacle's path properties relative to a path.
 * The `s` and `l` values are examples of path properties.
 * The decision of an obstacle is also associated with a path.
 *
 * The decisions have two categories: lateral decision and longitudinal
 * decision.
 * Lateral decision includes: nudge, ignore.
 * Lateral decision saftey priority: nudge > ignore.
 * Longitudinal decision includes: stop, yield, follow, overtake, ignore.
 * Decision saftey priorities order: stop > yield >= follow > overtake > ignore
 *
 * Ignore decision belongs to both lateral decision and longitudinal decision,
 * and it has the lowest priority.
 */
    class PathObstacle
    {
    public:
        PathObstacle() = default;
        explicit PathObstacle(const DetectedObject *obstacle);

        const std::string &Id() const;

        const DetectedObject *obstacle() const;

        /**
   * return the merged lateral decision
   * Lateral decision is one of {Nudge, Ignore}
   **/
        const ObjectDecision &LateralDecision() const;

        /**
   * @brief return the merged longitudinal decision
   * Longitudinal decision is one of {Stop, Yield, Follow, Overtake, Ignore}
   **/
        const ObjectDecision &LongitudinalDecision() const;

        const std::string DebugString() const;

        const SLBoundary &PerceptionSLBoundary() const;

        const StBoundary &reference_line_st_boundary() const;

        const StBoundary &st_boundary() const;

        const std::vector<std::string> &decider_tags() const;

        const std::vector<ObjectDecision> &decisions() const;

        void AddLongitudinalDecision(const std::string &decider_tag,
                                     const ObjectDecision &decision);

        void AddLateralDecision(const std::string &decider_tag,
                                const ObjectDecision &decision);
        bool HasLateralDecision() const;

        void SetStBoundary(const StBoundary &boundary);

        void SetStBoundaryType(const StBoundary::BoundaryType type);

        void EraseStBoundary();

        void SetReferenceLineStBoundary(const StBoundary &boundary);

        void SetReferenceLineStBoundaryType(const StBoundary::BoundaryType type);

        void EraseReferenceLineStBoundary();

        bool HasLongitudinalDecision() const;

        bool HasNonIgnoreDecision() const;

        /**
   * @brief Calculate stop distance with the obstacle using the ADC's minimum
   * turning radius
   */
        double MinRadiusStopDistance() const;

        /**
   * @brief Check if this object can be safely ignored.
   * The object will be ignored if the lateral decision is ignore and the
   * longitudinal decision is ignore
   *  return longitudinal_decision_ == ignore && lateral_decision == ignore.
   */
        bool IsIgnore() const;
        bool IsLongitudinalIgnore() const;
        bool IsLateralIgnore() const;

        void BuildReferenceLineStBoundary(const ReferenceLine &reference_line,
                                          const double adc_start_s);

        void SetPerceptionSlBoundary(const SLBoundary &sl_boundary);

        /**
   * @brief check if a ObjectDecisionType is a longitudinal decision.
   **/
        static bool IsLongitudinalDecision(const ObjectDecision &decision);

        /**
   * @brief check if a ObjectDecisionType is a lateral decision.
   **/
        static bool IsLateralDecision(const ObjectDecision &decision);

        void SetBlockingObstacle(bool blocking) { is_blocking_obstacle_ = blocking; }
        bool IsBlockingObstacle() const { return is_blocking_obstacle_; }

    private:
        FRIEND_TEST(MergeLongitudinalDecision, AllDecisions);
        static ObjectDecision MergeLongitudinalDecision(
            const ObjectDecision &lhs, const ObjectDecision &rhs);
        FRIEND_TEST(MergeLateralDecision, AllDecisions);
        static ObjectDecision MergeLateralDecision(const ObjectDecision &lhs,
                                                   const ObjectDecision &rhs);

        static int get_longitudinal_vaule(ObjectDecisionType tag);
        static int get_lateral_vaule(ObjectDecisionType tag);

        bool BuildTrajectoryStBoundary(const ReferenceLine &reference_line,
                                       const double adc_start_s,
                                       StBoundary *const st_boundary);
        bool IsValidObstacle(
            const DetectedObject &perception_obstacle);
        std::string id_;
        const DetectedObject *obstacle_ = nullptr;
        std::vector<ObjectDecision> decisions_;
        std::vector<std::string> decider_tags_;
        SLBoundary perception_sl_boundary_;

        StBoundary reference_line_st_boundary_;
        StBoundary st_boundary_;

        ObjectDecision lateral_decision_;
        ObjectDecision longitudinal_decision_;

        bool is_blocking_obstacle_ = false;

        double min_radius_stop_distance_ = -1.0;

        CAR_BASIC_INFO car_params;
    };

} // namespace PlannerHNS
