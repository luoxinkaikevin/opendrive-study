
#ifndef PATHTIMEGRAPH_H_
#define PATHTIMEGRAPH_H_

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <string>
#include "op_planner/RoadNetwork.h"
#include "op_planner/PlanningHelpers.h"
#include "op_planner/MatrixOperations.h"
#include "op_planner/PolynomialCurve.h"
#include "op_planner/CartesianFrenetConversion.h"
#include "op_planner/reference_line/reference_line_info.h"
#include "op_planner/log.h"
#include "op_planner/reference_line/path_obstacle.h"

namespace PlannerHNS
{
    using State = std::array<double, 3>;
    using Condition = std::pair<State, double>;

    class PathTimeGraph
    {
    public:
        PathTimeGraph(const std::vector<const DetectedObject *> &obj_list,
                      const std::vector<WayPoint> &discretized_ref_points,
                      const ReferenceLineInfo *ptr_reference_line_info,
                      const double s_start, const double s_end,
                      const double t_start, const double t_end);
        ~PathTimeGraph();

        const std::vector<PathTimeObstacle> &GetPathTimeObstacles() const;

        std::pair<double, double> get_path_range() const;

        std::pair<double, double> get_time_range() const;
        bool GetPathTimeObstacle(const std::string &obstacle_id,
                                 PathTimeObstacle *path_time_obstacle);

        std::vector<std::pair<double, double>> GetPathBlockingIntervals(
            const double t) const;
        std::vector<std::vector<std::pair<double, double>>> GetPathBlockingIntervals(
            const double t_start, const double t_end, const double t_resolution);

        std::vector<PathTimePoint> GetObstacleSurroundingPoints(
            const std::string &obstacle_id, const double s_dist,
            const double t_density) const;

        bool IsObstacleInGraph(const std::string &obstacle_id);

        const ReferenceLineInfo *ptg_reference_info() { return ptr_reference_line_info_; }

    private:
        void SetupObstacles(const std::vector<const DetectedObject *> &obstacles,
                            const std::vector<WayPoint> &discretized_ref_points);
        PathTimePoint SetPathTimePoint(const std::string &obstacle_id, const double s,
                                       const double t) const;

        void SetStaticObstacle(
            const PathObstacle *obstacle,
            const std::vector<WayPoint> &discretized_ref_points);
        void SetDynamicObstacle(
            const DetectedObject *obstacle,
            const std::vector<WayPoint> &discretized_ref_points);

    private:
        std::pair<double, double> time_range_;
        // 起始s和终止s
        std::pair<double, double> path_range_;

        const ReferenceLineInfo *ptr_reference_line_info_;

        std::unordered_map<std::string, PathTimeObstacle> path_time_obstacle_map_;
        std::vector<PathTimeObstacle> path_time_obstacles_;
        std::vector<SLBoundary> static_obs_sl_boundaries_;

        double trajectory_time_length_;
        double trajectory_time_resolution_;
        std::vector<DetectedObject> obstacle_list_;
        std::vector<WayPoint> discretized_ref_points_;
        // FeasibleRegion feasible_region_(init_s_);
    };

} /* namespace PlannerHNS */

#endif
