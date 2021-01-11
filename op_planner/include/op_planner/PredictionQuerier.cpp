#include "PredictionQuerier.h"

namespace PlannerHNS
{

    PredictionQuerier::PredictionQuerier(
        const std::vector<const DetectedObject *> &obstacles,
        const std::shared_ptr<std::vector<WayPoint>> &ptr_reference_line)
        : ptr_reference_line_(ptr_reference_line)
    {
        for (const auto ptr_obstacle : obstacles)
        {
            if(ptr_obstacle->IsVirtual())
            {
                return;
            }

            if (id_obstacle_map_.find(ptr_obstacle->Id()) == id_obstacle_map_.end())
            {
                id_obstacle_map_[ptr_obstacle->Id()] = ptr_obstacle;
                obstacles_.push_back(ptr_obstacle);
            }
            else
            {
                AWARN << "Duplicated obstacle found [" << ptr_obstacle->Id() << "]";
            }
        }
    }

    std::vector<const DetectedObject *> PredictionQuerier::GetObstacles() const
    {
        return obstacles_;
    }
 
    std::unordered_map<std::string, const DetectedObject *> PredictionQuerier::GetObstaclesMap() const
    {
        return id_obstacle_map_;
    }

    double PredictionQuerier::ProjectVelocityAlongReferenceLine(
        const std::string &obstacle_id, const double s, const double t) const
    {
        CHECK(id_obstacle_map_.find(obstacle_id) != id_obstacle_map_.end());
        if (id_obstacle_map_.find(obstacle_id) == id_obstacle_map_.end())
            return 0.0;
        if (id_obstacle_map_.at(obstacle_id)->predTrajectory.empty())
            return 0.0;
        const auto &trajectory = id_obstacle_map_.at(obstacle_id)->predTrajectory;

        if (trajectory.empty())
            return 0.0;
        int num_traj_point = trajectory.size();
        if (num_traj_point < 2)
        {
            return 0.0;
        }

        if (t < trajectory.at(0).timeCost ||
            t > trajectory.at(num_traj_point - 1).timeCost)
        {
            return 0.0;
        } 

        auto matched_it =
            std::lower_bound(trajectory.begin(),
                             trajectory.end(), t,
                             [](const WayPoint &p, const double t) { return p.timeCost < t; });

        double v = matched_it->v;
        double theta = matched_it->pos.a;
        double v_x = v * std::cos(theta);
        double v_y = v * std::sin(theta);

        auto obstacle_point_on_ref_line = PlanningHelpers::MatchToPath(*ptr_reference_line_, s);
        auto ref_theta = obstacle_point_on_ref_line.pos.a;

        return std::cos(ref_theta) * v_x + std::sin(ref_theta) * v_y;
    }

} // namespace PlannerHNS
