#include "ConstraintChecker.h"
#include "planning_gflags.h"

namespace PlannerHNS
{

    bool ConstraintChecker::ValidTrajectory(
        const DiscretizedTrajectory &trajectory,
        const ReferenceLineInfo *ptr_reference_line_info,
        const Curve1d &lat_trajectory,
        std::size_t &num_speed_lower_bound,
        std::size_t &num_longitudinal_acceleration_lower_bound,
        std::size_t &num_kappa_bound,
        std::size_t &num_lane_border_limit)
    {
        const double kMaxCheckRelativeTime = FLAGS_trajectory_time_length;

        // for every trajectory point
        for (const auto &p : trajectory.trajectory_points())
        {
            double t = p.timeCost;
            if (t > kMaxCheckRelativeTime)
            {
                break;
            }
            // check trajectory point speed
            // double lon_v = p.v;
            // -0.1-50
            // if (!WithinRange(lon_v, FLAGS_speed_lower_bound, FLAGS_speed_upper_bound))
            // {
            //     num_speed_lower_bound++;
            //     ADEBUG << "Velocity at relative time " << t
            //            << " exceeds bound, value: " << lon_v << ", bound ["
            //            << FLAGS_speed_lower_bound << ", " << FLAGS_speed_upper_bound
            //            << "].";
            //     return false;
            // }
            // check acceleration
            //-4.5-4.0
            // double lon_a = p.acceleration;
            // if (!WithinRange(lon_a, FLAGS_longitudinal_acceleration_lower_bound,
            //                  FLAGS_longitudinal_acceleration_upper_bound))
            // {
            //     num_longitudinal_acceleration_lower_bound++;
            //     ADEBUG << "Longitudinal acceleration at relative time " << t
            //            << " exceeds bound, value: " << lon_a << ", bound ["
            //            << FLAGS_longitudinal_acceleration_lower_bound << ", "
            //            << FLAGS_longitudinal_acceleration_upper_bound << "].";
            //     return false;
            // }
            // check kappa
            // 0.15     // min radius 7m
            double kappa = p.kappa();
            if (!WithinRange(kappa, -FLAGS_kappa_bound, FLAGS_kappa_bound))
            {
                num_kappa_bound++;
                ADEBUG << "Kappa at relative time " << t
                       << " exceeds bound, value: " << kappa << ", bound ["
                       << -FLAGS_kappa_bound << ", " << FLAGS_kappa_bound << "].";
                return false;
            }

            double l = lat_trajectory.Evaluate(0, t);

            auto lane_width =
                ptr_reference_line_info->reference_line().current_reference_pose().laneWidth;
            constexpr float car_width = 1.8;
            double in_lane_range = (lane_width - car_width) * 0.5 + 0.2;

            if (ReferenceLineInfo::lane_change_state == CHANGE_DISABLE)
            {
                if (!ptr_reference_line_info->reference_line()
                         .current_reference_pose()
                         .enable_road_occupation_left)
                {
                    // std::cout<<"ee"<<std::endl;
                    if (!WithinRange(l, -in_lane_range, in_lane_range))
                    {
                        num_lane_border_limit++;
                        // ADEBUG << "Ego center Lateral distance at relative time " << t
                        //        << " exceeds bound, value: " << l << ", bound ["
                        //        << -in_lane_range << ", " << in_lane_range << "].";

                        // ADEBUG << "trajectory out of lane border " << l;
                        return false;
                    }
                }
                else
                {
                    if (!WithinRange(l, -in_lane_range, lane_width + car_width * 0.5))
                    {
                        num_lane_border_limit++;
                        // ADEBUG << "Ego center Lateral distance at relative time " << t
                        //        << " exceeds bound, value: " << l << ", bound ["
                        //        << -in_lane_range << ", " << in_lane_range << "].";

                        // ADEBUG << "trajectory out of lane border " << l;
                        return false;
                    }
                }
            }
            else if (ReferenceLineInfo::lane_change_state == READY_TO_CHANGE_LEFT ||
                     ReferenceLineInfo::lane_change_state == READY_TO_CHANGE_BACK_FROM_LEFT ||
                     ReferenceLineInfo::lane_change_state == ENFORCE_CHANGE_BACK_FROM_LEFT)
            {

                if (!WithinRange(l, -FLAGS_change_lane_lat_bound, lane_width * 1.5))
                {
                    num_lane_border_limit++;
                    ADEBUG << "Lateral distance at relative time " << t
                           << " exceeds bound, value: " << l << ", bound ["
                           << -FLAGS_change_lane_lat_bound << ", " << lane_width * 1.5 << "].";

                    ADEBUG << "trajectory out of lane border " << l;
                    return false;
                }
            }
            else if (ReferenceLineInfo::lane_change_state == READY_TO_CHANGE_RIGHT ||
                     ReferenceLineInfo::lane_change_state == READY_TO_CHANGE_BACK_FROM_RIGHT ||
                     ReferenceLineInfo::lane_change_state == ENFORCE_CHANGE_BACK_FROM_RIGHT)
            {
                if (!WithinRange(l, -lane_width * 1.5, FLAGS_change_lane_lat_bound))
                {
                    num_lane_border_limit++;
                    ADEBUG << "Lateral distance at relative time " << t
                           << " exceeds bound, value: " << l << ", bound ["
                           << -lane_width * 1.5 << ", " << FLAGS_change_lane_lat_bound << "].";

                    ADEBUG << "trajectory out of lane border " << l;
                    return false;
                }
            }
            else if (ReferenceLineInfo::lane_change_state == CHANGE_LEFT_SUCCESS)
            {
                if (!WithinRange(l, lane_width - in_lane_range, lane_width + in_lane_range))
                {
                    num_lane_border_limit++;
                    ADEBUG << "Lateral distance at relative time " << t
                           << " exceeds bound, value: " << l << ", bound ["
                           << lane_width - in_lane_range << ", " << lane_width + in_lane_range << "].";

                    ADEBUG << "trajectory out of lane border " << l;
                    return false;
                }
            }
            else if (ReferenceLineInfo::lane_change_state == CHANGE_RIGHT_SUCCESS)
            {
                if (!WithinRange(l, -lane_width - in_lane_range, -lane_width + in_lane_range))
                {
                    num_lane_border_limit++;
                    ADEBUG << "Lateral distance at relative time " << t
                           << " exceeds bound, value: " << l << ", bound ["
                           << -lane_width - in_lane_range << ", " << -lane_width + in_lane_range << "].";

                    ADEBUG << "trajectory out of lane border " << l;
                    return false;
                }
            }
        }

        for (std::size_t i = 1; i < trajectory.NumOfPoints(); ++i)
        {
            const auto &p0 = trajectory.WayPointAt(i - 1);
            const auto &p1 = trajectory.WayPointAt(i);
            double t = p0.timeCost;
            if (p1.timeCost > kMaxCheckRelativeTime)
            {
                break;
            }
            // 1.1 (10km/h)
            double lat_a = p1.v * p1.v * p1.kappa();
            if (!WithinRange(lat_a, -FLAGS_lateral_acceleration_bound,
                             FLAGS_lateral_acceleration_bound))
            {
                ADEBUG << "Lateral acceleration at relative time " << t
                       << " exceeds bound, value: " << lat_a << ", bound ["
                       << -FLAGS_lateral_acceleration_bound << ", "
                       << FLAGS_lateral_acceleration_bound << "].";
                return false;
            }


            // double dt = p1.timeCost - p0.timeCost;
            // double d_lon_a = p1.acceleration - p0.acceleration;
            // double lon_jerk = d_lon_a / dt;
            // // -4,4
            // if (!WithinRange(lon_jerk, FLAGS_longitudinal_jerk_lower_bound,
            //                  FLAGS_longitudinal_jerk_upper_bound))
            // {
            //     ADEBUG << "Longitudinal jerk at relative time " << t
            //            << " exceeds bound, value: " << lon_jerk << ", bound ["
            //            << FLAGS_longitudinal_jerk_lower_bound << ", "
            //            << FLAGS_longitudinal_jerk_upper_bound << "].";
            //     return false;
            // }

            // double d_lat_a = p1.v * p1.v * p1.kappa() -
            //                  p0.v * p0.v * p0.kappa();
            // // 4
            // double lat_jerk = d_lat_a / dt;
            // if (!WithinRange(lat_jerk, -FLAGS_lateral_jerk_bound,
            //                  FLAGS_lateral_jerk_bound))
            // {
            //     ADEBUG << "Lateral jerk at relative time " << t
            //            << " exceeds bound, value: " << lat_jerk << ", bound ["
            //            << -FLAGS_lateral_jerk_bound << ", " << FLAGS_lateral_jerk_bound
            //            << "].";
            //     return false;
            // }
        }

        return true;
    }

    namespace
    {

        inline bool fuzzy_within(const double v, const double lower, const double upper,
                                 const double e = 1.0e-4)
        {
            return v > lower - e && v < upper + e;
        }
    } // namespace

    bool ConstraintChecker1d::IsValidLongitudinalTrajectory(
        const Curve1d &lon_trajectory)
    {
        double t = 0.0;
        while (t < lon_trajectory.ParamLength())
        {
            double v = lon_trajectory.Evaluate(1, t); // evalute_v
            if (!fuzzy_within(v, FLAGS_speed_lower_bound, FLAGS_speed_upper_bound))
            {
                return false;
            }

            double a = lon_trajectory.Evaluate(2, t); // evaluat_a
            if (!fuzzy_within(a, FLAGS_longitudinal_acceleration_lower_bound,
                              FLAGS_longitudinal_acceleration_upper_bound))
            {
                return false;
            }

            double j = lon_trajectory.Evaluate(3, t);
            if (!fuzzy_within(j, FLAGS_longitudinal_jerk_lower_bound,
                              FLAGS_longitudinal_jerk_upper_bound))
            {
                return false;
            }
            t += FLAGS_trajectory_time_resolution;
        }
        return true;
    }

    bool ConstraintChecker1d::IsValidLateralTrajectory(
        const Curve1d &lat_trajectory, const Curve1d &lon_trajectory)
    {
        double t = 0.0;
        while (t < lon_trajectory.ParamLength())
        {
            double s = lon_trajectory.Evaluate(0, t);
            double dd_ds = lat_trajectory.Evaluate(1, s);
            double ds_dt = lon_trajectory.Evaluate(1, t);

            double d2d_ds2 = lat_trajectory.Evaluate(2, s);
            double d2s_dt2 = lon_trajectory.Evaluate(2, t);

            double a = 0.0;
            if (s < lat_trajectory.ParamLength())
            {
                a = d2d_ds2 * ds_dt * ds_dt + dd_ds * d2s_dt2;
            }

            if (!fuzzy_within(a, -FLAGS_lateral_acceleration_bound,
                              FLAGS_lateral_acceleration_bound))
            {
                return false;
            }

            // this is not accurate, just an approximation...
            double j = 0.0;
            if (s < lat_trajectory.ParamLength())
            {
                j = lat_trajectory.Evaluate(3, s) * lon_trajectory.Evaluate(3, t);
            }

            if (!fuzzy_within(j, -FLAGS_lateral_jerk_bound, FLAGS_lateral_jerk_bound))
            {
                return false;
            }
            t += FLAGS_trajectory_time_resolution;
        }
        return true;
    }

} // namespace PlannerHNS
