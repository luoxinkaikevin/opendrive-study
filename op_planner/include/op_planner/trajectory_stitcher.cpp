#include "op_planner/trajectory_stitcher.h"

namespace PlannerHNS
{

    std::vector<WayPoint> TrajectoryStitcher::ComputeReinitStitchingTrajectory(
        const WayPoint &vehicle_state)
    {
        WayPoint init_point;
        init_point.pos = vehicle_state.pos;
        init_point.set_kappa(vehicle_state.kappa());
        init_point.v = vehicle_state.v;
        init_point.acceleration = vehicle_state.acceleration;
        init_point.timeCost = 0;
        init_point.task=vehicle_state.task;

        return std::vector<WayPoint>(1, init_point);
    }

    // Planning from current vehicle state:
    // if 1. the auto-driving mode is off or
    //    2. we don't have the trajectory from last planning cycle or
    //    3. the position deviation from actual and target is too high
    std::vector<WayPoint> TrajectoryStitcher::ComputeStitchingTrajectory(
        const WayPoint &vehicle_state, const double current_timestamp,
        const double planning_cycle_time,
        const PublishableTrajectory *prev_trajectory, bool *is_replan)
    {
        *is_replan = true;
        if (!FLAGS_enable_trajectory_stitcher)
        {
            return ComputeReinitStitchingTrajectory(vehicle_state);
        }
        if (!prev_trajectory)
        {
            return ComputeReinitStitchingTrajectory(vehicle_state);
        }

        std::size_t prev_trajectory_size = prev_trajectory->NumOfPoints();

        if (prev_trajectory_size == 0)
        {
            ADEBUG << "Projected trajectory at time [" << prev_trajectory->header_time()
                   << "] size is zero! Previous planning not exist or failed. Use "
                      "origin car status instead.";
            return ComputeReinitStitchingTrajectory(vehicle_state);
        }

        const double veh_rel_time =
            current_timestamp - prev_trajectory->header_time();

        std::size_t matched_index = prev_trajectory->QueryNearestPoint(veh_rel_time);

        if (matched_index == 0 &&
            veh_rel_time < prev_trajectory->StartPoint().timeCost)
        {
            AWARN << "current time smaller than the previous trajectory's first time";
            return ComputeReinitStitchingTrajectory(vehicle_state);
        }
        if (matched_index + 1 >= prev_trajectory_size)
        {
            AWARN << "current time beyond the previous trajectory's last time";
            return ComputeReinitStitchingTrajectory(vehicle_state);
        }

        auto matched_point = prev_trajectory->Evaluate(veh_rel_time);

        auto frenet_sd = ComputePositionProjection(
            vehicle_state.pos.x, vehicle_state.pos.y, *prev_trajectory);

        auto lon_diff = std::fabs(frenet_sd.first);
        auto lat_diff = std::fabs(frenet_sd.second);

        ADEBUG << "Control lateral diff: " << lat_diff
               << ", longitudinal diff: " << lon_diff;

        if (lat_diff > FLAGS_replan_lateral_distance_threshold ||
            lon_diff > FLAGS_replan_longitudinal_distance_threshold)
        {
            // AERROR << "the distance between matched point and actual position is too "
            //           "large. Replan is triggered. lat_diff = "
            //        << lat_diff << ", lon_diff = " << lon_diff;
            return ComputeReinitStitchingTrajectory(vehicle_state);
        }
        // 当前轨迹点下一循环后的位置
        double forward_rel_time =
            prev_trajectory->WayPointAt(matched_index).timeCost +
            planning_cycle_time;

        std::size_t forward_index =
            prev_trajectory->QueryNearestPoint(forward_rel_time);

        ADEBUG << "matched_index: " << matched_index;
        std::vector<WayPoint> stitching_trajectory(
            prev_trajectory->trajectory_points().begin() +
                std::max(0, static_cast<int>(matched_index - 1)),
            prev_trajectory->trajectory_points().begin() + forward_index + 1);

        const double zero_s = matched_point.s();

        if(stitching_trajectory.empty())
        {
            return ComputeReinitStitchingTrajectory(vehicle_state);
        }

        for (auto &tp : stitching_trajectory)
        {
            tp.timeCost=tp.timeCost+prev_trajectory->header_time()-current_timestamp;
            tp.set_s(tp.s() - zero_s);
        }
        *is_replan = false;
        return stitching_trajectory;
    }

    std::pair<double, double> TrajectoryStitcher::ComputePositionProjection(
        const double x, const double y,
        const PublishableTrajectory &prev_trajectory)
    {
        auto index = prev_trajectory.QueryNearestPoint({x, y, 0, 0});
        auto p = prev_trajectory.WayPointAt(index);

        GPSPoint v(x - p.pos.x, y - p.pos.y, 0, 0);
        GPSPoint n(std::cos(p.pos.a), std::sin(p.pos.a), 0, 0);

        std::pair<double, double> frenet_sd;
        frenet_sd.first = v.InnerProd(n) + p.s();
        frenet_sd.second = v.CrossProd(n);
        return frenet_sd;
    }

} // namespace PlannerHNS
