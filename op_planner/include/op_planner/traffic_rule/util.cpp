#include "op_planner/traffic_rule/util.h"

namespace PlannerHNS
{
    namespace util
    {
        //output deceleration for stop
        double GetADCStopDeceleration(ReferenceLineInfo *const reference_line_info,
                                      const double stop_line_s,
                                      const double min_pass_s_distance)
        {
            double adc_speed =reference_line_info->AdcPlanningPoint().v;
            if (adc_speed < FLAGS_max_stop_speed)
            {
                return 0.0;
            }

            double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s;
            double stop_distance = 0;

            if (stop_line_s > adc_front_edge_s)
            {
                stop_distance = stop_line_s - adc_front_edge_s;
            }
            else
            {
                stop_distance = stop_line_s + min_pass_s_distance - adc_front_edge_s;
            }
            if (stop_distance < 1e-5)
            {
                return std::numeric_limits<double>::max();
            }
            return (adc_speed * adc_speed) / (2 * stop_distance);
        }

    } // namespace util
} // namespace PlannerHNS
