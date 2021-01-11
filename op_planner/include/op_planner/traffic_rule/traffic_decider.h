#pragma once

#include <string>
#include <vector>
#include <limits>

#include "op_planner/reference_line/reference_line_info.h"
#include "op_planner/reference_line/frame.h"

#include "op_planner/traffic_rule/change_lane.h"
#include "op_planner/traffic_rule/destination.h"
#include "op_planner/traffic_rule/crosswalk.h"
#include "op_planner/traffic_rule/signal_light.h"

namespace PlannerHNS
{

    /**
 * @class TrafficDecider
 * @brief Create traffic related decision in this class.
 * The created obstacles is added to obstacles_, and the decision is added to
 * path_obstacles_
 * Traffic obstacle examples include:
 *  * Traffic Light
 *  * End of routing
 *  * Select the drivable reference line.
 */
    class TrafficDecider
    {
    public:
        TrafficDecider() = default;
        //注册交通规则，从配置文件获取当前应用的交通规则
        virtual ~TrafficDecider() = default;
        //首先判断是否启用信号灯决策（FLAGS_enable_traffic_light），若未启动，则跳过该决策。
        //之后调用rule_factory_.CreateObject函数动态创建配置文件中指定的规则对象
        bool Execute(
            Frame *frame, ReferenceLineInfo *reference_line_info,
            const RoadNetwork &map);

        void readTrafficLight(std::vector<TrafficState> &traffic_state)
        {
            traffic_state_ = traffic_state;
        }

    private:
        void BuildPlanningTarget(ReferenceLineInfo *reference_line_info);

        ChangeLane change_lane_decision;
        Destination destination_decision;
        Crosswalk crosswalk;
        SignalLight signal_light;
        std::vector<TrafficState> traffic_state_;
    };

} // namespace PlannerHNS
