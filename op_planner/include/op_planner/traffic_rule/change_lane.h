#pragma once

#include <string>
#include <vector>
#include "op_planner/reference_line/reference_line_info.h"
#include "op_planner/reference_line/path_obstacle.h"
#include "op_planner/reference_line/reference_line_base.h"
#include "op_planner/reference_line/path_decision.h"
#include "op_planner/reference_line/frame.h"

namespace PlannerHNS
{

    /**
 * @brief This class defines rule-based lane change behaviors for the ADC.
 */
    class ChangeLane
    {
    public:
         ChangeLane()= default;
        virtual ~ChangeLane() = default;

        bool ApplyRule(Frame *const frame,
                                 ReferenceLineInfo *const reference_line_info);

    private:
        /**
   * @brief This function will filter obstacles based on change lane strategy.
   **/
        bool FilterObstacles(ReferenceLineInfo *reference_line_info);
        /**
   * @brief This function will extend the prediction of the guard obstacle to
   *guard lane change action. Due to the ST path may drive on the forward lane
   *first, then slowly move to the target lane when making lane change, we need
   *to make sure the vehicle is aware that it actually occupies the target lane,
   *even when it is not on the target lane yet.
   **/
        bool CreateGuardObstacle(const ReferenceLineInfo *reference_line_info,
                                 DetectedObject *obstacle);

        /**
   * @brief create overtake decision for the give path obstacle
   */
        ObjectDecision CreateOvertakeDecision(
            const ReferenceLine &reference_line,
            const PathObstacle *path_obstacle) const;

        std::vector<const PathObstacle *> guard_obstacles_;
        std::vector<const PathObstacle *> overtake_obstacles_;

    private:
        double min_overtake_distance = 10.0;
        double min_overtake_time = 2.0;
        bool enable_guard_obstacle = false;
        double guard_distance = 100;
        double min_guard_speed = 1.0;
    };

} // namespace PlannerHNS
