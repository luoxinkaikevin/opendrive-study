
#include "op_planner/reference_line/path_decision.h"

namespace PlannerHNS
{

    using IndexedPathObstacles = IndexedList<std::string, PathObstacle>;

    PathObstacle *PathDecision::AddPathObstacle(const PathObstacle &path_obstacle)
    {
        // std::lock_guard<std::mutex> lock(obstacle_mutex_);
        return path_obstacles_.Add(path_obstacle.Id(), path_obstacle);
    }

    const IndexedPathObstacles &PathDecision::path_obstacles() const
    {
        return path_obstacles_;
    }

    PathObstacle *PathDecision::Find(const std::string &object_id)
    {
        return path_obstacles_.Find(object_id);
    }

    const PathObstacle *PathDecision::Find(const std::string &object_id) const
    {
        return path_obstacles_.Find(object_id);
    }

    void PathDecision::SetStBoundary(const std::string &id,
                                     const StBoundary &boundary)
    {
        auto *obstacle = path_obstacles_.Find(id);

        if (!obstacle)
        {
            AERROR << "Failed to find obstacle : " << id;
            return;
        }
        else
        {
            obstacle->SetStBoundary(boundary);
        }
    }

    bool PathDecision::AddLateralDecision(const std::string &tag,
                                          const std::string &object_id,
                                          const ObjectDecision &decision)
    {
        auto *path_obstacle = path_obstacles_.Find(object_id);
        if (!path_obstacle)
        {
            AERROR << "failed to find obstacle";
            return false;
        }
        path_obstacle->AddLateralDecision(tag, decision);
        return true;
    }

    void PathDecision::EraseStBoundaries()
    {
        for (const auto *path_obstacle : path_obstacles_.Items())
        {
            auto *obstacle_ptr = path_obstacles_.Find(path_obstacle->Id());
            obstacle_ptr->EraseStBoundary();
        }
    }

    bool PathDecision::AddLongitudinalDecision(const std::string &tag,
                                               const std::string &object_id,
                                               const ObjectDecision &decision)
    {
        auto *path_obstacle = path_obstacles_.Find(object_id);
        if (!path_obstacle)
        {
            AERROR << "failed to find obstacle";
            return false;
        }
        path_obstacle->AddLongitudinalDecision(tag, decision);
        return true;
    }

    bool PathDecision::MergeWithMainStop(const ObjectDecision &obj_stop,
                                         const std::string &obj_id,
                                         const ReferenceLine &reference_line,
                                         const SLBoundary &adc_sl_boundary)
    {
        GPSPoint stop_point = obj_stop.stop_point;
        auto stop_line_sl =
            PlanningHelpers::GetPathFrenetCoordinate(reference_line.reference_points(), stop_point);

        double stop_line_s = stop_line_sl.first;
        if (stop_line_s < 0 || stop_line_s > reference_line.Length())
        {
            AERROR << "Ignore object:" << obj_id << " fence route_s[" << stop_line_s
                   << "] not in range[0, " << reference_line.Length() << "]";
            return false;
        }

        // check stop_line_s vs adc_s, ignore if it is further way than main stop
        const double kStopBuff = 1.0;
        stop_line_s = std::fmax(stop_line_s, adc_sl_boundary.end_s - kStopBuff);

        if (stop_line_s >= stop_reference_line_s_)
        {
            ADEBUG << "stop point is further than current main stop point.";
            return false;
        }

        main_stop_ = obj_stop;

        // ADEBUG << " main stop obstacle id:" << obj_id
        //        << " stop_line_s:" << stop_line_s << " stop_point: ("
        //        << obj_stop.stop_point().x() << obj_stop.stop_point().y()
        //        << " ) stop_heading: " << obj_stop.stop_heading();
        return true;
    }

} // namespace PlannerHNS
