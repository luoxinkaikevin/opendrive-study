#include "op_planner/traffic_rule/change_lane.h"

#include <algorithm>

namespace PlannerHNS
{
    namespace
    {
        constexpr double kMinGuardVehicleSpeed = 1.0;
    } // namespace

    // This function will filter obstacles based on change lane strategy.
    // put obstacle which trajectory could influence lane change action
    bool ChangeLane::FilterObstacles(ReferenceLineInfo *reference_line_info)
    {
        const auto &reference_line = reference_line_info->reference_line();
        const auto &adc_sl_boundary = reference_line_info->AdcSlBoundary();
        const auto &path_decision = reference_line_info->path_decision();
        const PathObstacle *first_guard_vehicle = nullptr;
        constexpr double kGuardForwardDistance = 60;
        double max_s = 0.0;
        for (const auto *path_obstacle : path_decision->path_obstacles().Items())
        {
            const auto *obstacle = path_obstacle->obstacle();
            // skip static obstacle
            if (!obstacle->HasTrajectory())
            {
                continue;
            }
            if (path_obstacle->PerceptionSLBoundary().start_s > adc_sl_boundary.end_s)
            {
                continue;
            }
            // consider path obstacle behind adc
            if (path_obstacle->PerceptionSLBoundary().end_s <
                adc_sl_boundary.start_s -
                    std::max(min_overtake_distance, obstacle->center.v * min_overtake_time))
            {
                overtake_obstacles_.push_back(path_obstacle);
            }
            const auto last_point = obstacle->predTrajectory.back();

            if (last_point.v < min_guard_speed)
            {
                continue;
            }
            if (!reference_line.IsOnRoad(last_point.pos))
            {
                continue;
            }
            auto last_sl =
                PlanningHelpers::GetPathFrenetCoordinate(
                    reference_line.reference_points(), last_point.pos);

            if (last_sl.first < 0 ||
                last_sl.first > adc_sl_boundary.end_s + kGuardForwardDistance)
            {
                continue;
            }
            if (last_sl.first > max_s)
            {
                max_s = last_sl.first;
                first_guard_vehicle = path_obstacle;
            }
        }
        if (first_guard_vehicle)
        {
            guard_obstacles_.push_back(first_guard_vehicle);
        }
        return true;
    }

    /**
   * @brief This function will extend the prediction of the guard obstacle to
   *guard lane change action. Due to the ST path may drive on the forward lane
   *first, then slowly move to the target lane when making lane change, we need
   *to make sure the vehicle is aware that it actually occupies the target lane,
   *even when it is not on the target lane yet.
   **/
    bool ChangeLane::CreateGuardObstacle(
        const ReferenceLineInfo *reference_line_info, DetectedObject *obstacle)
    {
        if (!obstacle || !obstacle->HasTrajectory())
        {
            return false;
        }
        const auto last_point = obstacle->predTrajectory.back();

        const double kStepDistance = obstacle->l;
        double extend_v = std::max(last_point.v, min_guard_speed);
        const double time_delta = kStepDistance / extend_v;
        const auto &reference_line = reference_line_info->reference_line();
        const double end_s = std::min(reference_line.Length(),
                                      reference_line_info->AdcSlBoundary().end_s + guard_distance);
        auto sl_point =
            PlanningHelpers::GetPathFrenetCoordinate(
                reference_line.reference_points(), last_point.pos);
        double s = last_point.s() + kStepDistance;
        double ref_s = sl_point.first + kStepDistance;
        for (double t = last_point.timeCost + time_delta; ref_s < end_s;
             ref_s += kStepDistance, s += kStepDistance, t += time_delta)
        {
            auto ref_point = reference_line.GetNearestReferencePoint(ref_s);

            // Vec2d xy_point;
            // if (!reference_line.SLToXY(common::util::MakeSLPoint(ref_s, sl_point.l()),
            //                            &xy_point))
            // {
            //     return false;
            // }

            // auto *tp = obstacle->AddTrajectoryPoint();
            // tp->set_a(0.0);
            // tp->set_v(extend_v);
            // tp->set_relative_time(t);
            // tp->mutable_path_point()->set_x(xy_point.x());
            // tp->mutable_path_point()->set_y(xy_point.y());
            // tp->mutable_path_point()->set_theta(ref_point.heading());

            // // this is an approximate estimate since we do not use it.
            // tp->mutable_path_point()->set_s(s);
            // tp->mutable_path_point()->set_kappa(ref_point.kappa());
        }
        return true;
    }

    bool ChangeLane::ApplyRule(Frame *const frame,
                               ReferenceLineInfo *const reference_line_info)
    {
        guard_obstacles_.clear();
        overtake_obstacles_.clear();
        if (!FilterObstacles(reference_line_info))
        {
            return false;
        }
        if (enable_guard_obstacle &&
            !guard_obstacles_.empty())
        {
            for (const auto path_obstacle : guard_obstacles_)
            {
                auto *guard_obstacle =
                    frame->GetObstacleList()->Find(path_obstacle->Id());

                if (guard_obstacle &&
                    CreateGuardObstacle(reference_line_info, guard_obstacle))
                {
                    AINFO << "Created guard obstacle: " << guard_obstacle->Id();
                }
            }
        }

        if (!overtake_obstacles_.empty())
        {
            auto *path_decision = reference_line_info->path_decision();
            const auto &reference_line = reference_line_info->reference_line();
            for (const auto *path_obstacle : overtake_obstacles_)
            {
                auto overtake = CreateOvertakeDecision(reference_line, path_obstacle);
                path_decision->AddLongitudinalDecision(
                    "overtake", path_obstacle->Id(), overtake);
            }
        }
        return true;
    }

    ObjectDecision ChangeLane::CreateOvertakeDecision(
        const ReferenceLine &reference_line,
        const PathObstacle *path_obstacle) const
    {
        ObjectDecision overtake_decision;
        const double speed = path_obstacle->obstacle()->center.v;
        double distance = std::max(speed * min_overtake_time, min_overtake_distance);
        overtake_decision.distance_s = distance;
        double fence_s = path_obstacle->PerceptionSLBoundary().end_s + distance;
        auto point = reference_line.GetNearestReferencePoint(fence_s);
        overtake_decision.stop_point = point.pos;
        return overtake_decision;
    }

} // namespace PlannerHNS
