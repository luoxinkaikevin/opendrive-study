#include "op_planner/PathTimeGraph.h"
#include "planning_gflags.h"
// #include <ros/ros.h>

namespace PlannerHNS
{
    // using namespace PlannerHNS::PlanningHelpers;
    // double trajectory_time_resolution = 0.1;
    PathTimeGraph::PathTimeGraph(const std::vector<const DetectedObject *> &obj_list,
                                 const std::vector<WayPoint> &discretized_ref_points,
                                 const ReferenceLineInfo *ptr_reference_line_info,
                                 const double s_start, const double s_end,
                                 const double t_start, const double t_end)
    {
        path_range_.first = s_start;
        path_range_.second = s_end;
        time_range_.first = t_start;
        time_range_.second = t_end;
        trajectory_time_length_ = t_end;
        ptr_reference_line_info_ = ptr_reference_line_info;
        // 在构造函数中，给S-T graph添加所有的动静态障碍物
        SetupObstacles(obj_list, discretized_ref_points);
    }
    PathTimeGraph::~PathTimeGraph()
    {
    }

    void PathTimeGraph::SetupObstacles(
        const std::vector<const DetectedObject *> &obstacles,
        const std::vector<WayPoint> &discretized_ref_points)
    {

        for (const auto *path_obstacle :
             ptr_reference_line_info_->path_decision().path_obstacles().Items())
        {
            //------caculate lane change condition in reference lane
            const auto &sl_boundary = path_obstacle->PerceptionSLBoundary();

            // don't consider decision virtual obstacle
            if (path_obstacle->obstacle()->IsVirtual())
            {
                continue;
            }

            if (!path_obstacle->obstacle()->HasTrajectory())
            {
                SetStaticObstacle(path_obstacle, discretized_ref_points);
            }
            else
            {
                //plot a parallelogram on s-t graph, representing a dynamic obstacle
                SetDynamicObstacle(path_obstacle->obstacle(), discretized_ref_points);
            }
        }

        //对静态障碍物的占据范围，按s坐标升序排列
        std::sort(static_obs_sl_boundaries_.begin(), static_obs_sl_boundaries_.end(),
                  [](const SLBoundary &sl0, const SLBoundary &sl1) {
                      return sl0.start_s < sl1.start_s;
                  });
        // 对于动态障碍物 选定s-t范围
        for (auto &path_time_obstacle : path_time_obstacle_map_)
        {
            double s_upper = std::max(path_time_obstacle.second.bottom_right.s,
                                      path_time_obstacle.second.upper_right.s);
            double s_lower = std::min(path_time_obstacle.second.bottom_left.s,
                                      path_time_obstacle.second.upper_left.s);
            path_time_obstacle.second.path_lower = s_lower;
            path_time_obstacle.second.path_upper = s_upper;

            double t_upper = std::max(path_time_obstacle.second.bottom_right.t,
                                      path_time_obstacle.second.upper_right.t);
            double t_lower = std::min(path_time_obstacle.second.bottom_left.t,
                                      path_time_obstacle.second.upper_left.t);
            path_time_obstacle.second.time_lower = t_lower;
            path_time_obstacle.second.time_upper = t_upper;
            path_time_obstacles_.push_back(path_time_obstacle.second);
        }
    }

    /**
 * @brief plot a rectangle on s-t graph, representing the static obstacle
 * @param obstacle
 * @param discretized_ref_points, used to transform the obstacle's polygon(in x-y)
 *        to frenet
 */
    void PathTimeGraph::SetStaticObstacle(
        const PathObstacle *obstacle,
        const std::vector<WayPoint> &discretized_ref_points)
    {
        std::string obstacle_id = obstacle->Id();

        const auto &sl_boundary = obstacle->PerceptionSLBoundary();

        double lane_width =
            ptr_reference_line_info_->reference_line().current_reference_pose().laneWidth;

        double half_lane_width = lane_width * 0.5;

        double lane_param = 0;
        double lane_left_change = 0;
        double lane_right_change = 0;

        bool road_block_ = (ReferenceLineInfo::lane_change_state == CHANGE_DISABLE &&
                            ptr_reference_line_info_->reference_line().current_reference_pose().enable_road_occupation_left);

        std::cout << "road_block_ " << road_block_ << std::endl;

        if (!road_block_)
        {
            if (sl_boundary.start_s > path_range_.second ||
                sl_boundary.end_s < path_range_.first ||
                sl_boundary.start_l > (half_lane_width) ||
                sl_boundary.end_l < -half_lane_width)
            {
                // ADEBUG << "Obstacle [" << obstacle_id << "] is out of range.current half_width" << half_width
                //        << "," << sl_boundary.start_s << ","
                //        << sl_boundary.end_s << "," << sl_boundary.start_l << "," << sl_boundary.end_l;
                return;
            }
        }
        else if (road_block_ ||
                 ReferenceLineInfo::lane_change_state == READY_TO_CHANGE_BACK_FROM_LEFT ||
                 ReferenceLineInfo::lane_change_state == ENFORCE_CHANGE_BACK_FROM_LEFT)
        {
            if (sl_boundary.start_s > path_range_.second ||
                sl_boundary.end_s < path_range_.first ||
                sl_boundary.start_l > (half_lane_width + lane_width) ||
                sl_boundary.end_l < -half_lane_width)
            {
                ADEBUG << "Obstacle [" << obstacle_id << "] is out of range.current half_width" << half_lane_width
                       << "," << sl_boundary.start_s << ","
                       << sl_boundary.end_s << "," << sl_boundary.start_l << "," << sl_boundary.end_l;
                return;
            }
        }
        else if (ReferenceLineInfo::lane_change_state == READY_TO_CHANGE_BACK_FROM_RIGHT ||
                 ReferenceLineInfo::lane_change_state == ENFORCE_CHANGE_BACK_FROM_RIGHT)
        {
            if (sl_boundary.start_s > path_range_.second ||
                sl_boundary.end_s < path_range_.first ||
                sl_boundary.start_l > (half_lane_width) ||
                sl_boundary.end_l < -half_lane_width - lane_width)
            {
                ADEBUG << "Obstacle [" << obstacle_id << "] is out of range.current half_width" << half_lane_width
                       << "," << sl_boundary.start_s << ","
                       << sl_boundary.end_s << "," << sl_boundary.start_l << "," << sl_boundary.end_l;
                return;
            }
        }

        else if (ReferenceLineInfo::lane_change_state == READY_TO_CHANGE_LEFT)
        {
            lane_param = 0;
            lane_left_change = 1;
            lane_right_change = 0;

            if (sl_boundary.start_s > path_range_.second ||
                sl_boundary.end_s < path_range_.first ||
                sl_boundary.start_l > (half_lane_width + lane_param * lane_width +
                                       lane_left_change * lane_width) ||
                sl_boundary.end_l < (-half_lane_width + lane_param * lane_width +
                                     lane_right_change * lane_width))
            {

                // ADEBUG << "Obstacle [" << obstacle_id << "] is out of range.current half_width" << half_width
                //        << "," << sl_boundary.start_s << ","
                //        << sl_boundary.end_s << "," << sl_boundary.start_l << "," << sl_boundary.end_l;
                return;
            }
        }
        else if (ReferenceLineInfo::lane_change_state == READY_TO_CHANGE_RIGHT)
        {
            lane_param = 0;
            lane_left_change = 0;
            lane_right_change = -1;

            if (sl_boundary.start_s > path_range_.second ||
                sl_boundary.end_s < path_range_.first ||
                sl_boundary.start_l > (half_lane_width + lane_param * lane_width +
                                       lane_left_change * lane_width) ||
                sl_boundary.end_l < (-half_lane_width + lane_param * lane_width +
                                     lane_right_change * lane_width))
            {

                // ADEBUG << "Obstacle [" << obstacle_id << "] is out of range.current half_width" << half_width
                //        << "," << sl_boundary.start_s << ","
                //        << sl_boundary.end_s << "," << sl_boundary.start_l << "," << sl_boundary.end_l;
                return;
            }
        }
        else if (ReferenceLineInfo::lane_change_state == CHANGE_LEFT_SUCCESS)
        {
            lane_param = 1;
            lane_left_change = 0;
            lane_right_change = 0;

            if (sl_boundary.start_s > path_range_.second ||
                sl_boundary.end_s < path_range_.first ||
                sl_boundary.start_l > (half_lane_width + lane_param * lane_width +
                                       lane_left_change * lane_width) ||
                sl_boundary.end_l < (-half_lane_width + lane_param * lane_width +
                                     lane_right_change * lane_width))
            {

                // ADEBUG << "Obstacle [" << obstacle_id << "] is out of range.current half_width" << half_width
                //        << "," << sl_boundary.start_s << ","
                //        << sl_boundary.end_s << "," << sl_boundary.start_l << "," << sl_boundary.end_l;
                return;
            }
        }
        else
        {
            lane_param = -1;
            lane_left_change = 0;
            lane_right_change = 0;

            if (sl_boundary.start_s > path_range_.second ||
                sl_boundary.end_s < path_range_.first ||
                sl_boundary.start_l > (half_lane_width + lane_param * lane_width +
                                       lane_left_change * lane_width) ||
                sl_boundary.end_l < (-half_lane_width + lane_param * lane_width +
                                     lane_right_change * lane_width))
            {

                // ADEBUG << "Obstacle [" << obstacle_id << "] is out of range.current half_width" << half_width
                //        << "," << sl_boundary.start_s << ","
                //        << sl_boundary.end_s << "," << sl_boundary.start_l << "," << sl_boundary.end_l;
                return;
            }
        }

        //plot a static obstacle's occupancy graph on s-t graph, actually a rectangle,
        //so it's ok if the 4 corners are confirmed
        ADEBUG << "Obstacle [" << obstacle_id << "] in pt graph.";
        PathTimeObstacle path_time_obs;
        path_time_obs.obstacle_id = obstacle_id;
        path_time_obs.bottom_left = SetPathTimePoint(obstacle_id, sl_boundary.start_s, 0.0);
        path_time_obs.upper_left = SetPathTimePoint(obstacle_id, sl_boundary.end_s, 0.0);
        path_time_obs.bottom_right = SetPathTimePoint(
            obstacle_id, sl_boundary.start_s, trajectory_time_length_);
        path_time_obs.upper_right = SetPathTimePoint(
            obstacle_id, sl_boundary.end_s, trajectory_time_length_);
        // 数值形式插入
        path_time_obstacle_map_[obstacle_id] = path_time_obs;

        static_obs_sl_boundaries_.push_back(std::move(sl_boundary));
    }

    /**
 * @brief plot a parallelogram on s-t graph, representing a dynamic obstacle
 * @param obstacle
 * @param discretized_ref_points
 */

    void PathTimeGraph::SetDynamicObstacle(
        const DetectedObject *obstacle,
        const std::vector<WayPoint> &discretized_ref_points)
    {
        double relative_time = time_range_.first;
        //从起始点到终点

        while (relative_time < time_range_.second)
        {
            auto obj_trj_point = obstacle->GetPointAtTime(relative_time);
            math::Box2d box = obstacle->GetBoundingBox(obj_trj_point);
            SLBoundary sl_boundary;
            ptr_reference_line_info_->reference_line().GetSLBoundary(
                box, &sl_boundary);

            double lane_width =
                ptr_reference_line_info_->reference_line().GetLaneWidth(sl_boundary.start_s);

            double half_lane_width = lane_width * 0.5;
            double lane_param = 0;

            if (ReferenceLineInfo::lane_change_state == CHANGE_DISABLE ||
                ReferenceLineInfo::lane_change_state == READY_TO_CHANGE_BACK_FROM_LEFT ||
                ReferenceLineInfo::lane_change_state == READY_TO_CHANGE_BACK_FROM_RIGHT ||
                ReferenceLineInfo::lane_change_state == ENFORCE_CHANGE_BACK_FROM_LEFT ||
                ReferenceLineInfo::lane_change_state == ENFORCE_CHANGE_BACK_FROM_RIGHT)
            {
                lane_param = 0;
            }
            else if (ReferenceLineInfo::lane_change_state == CHANGE_LEFT_SUCCESS ||
                     ReferenceLineInfo::lane_change_state == READY_TO_CHANGE_LEFT)
            {
                lane_param = 1;
            }
            else
            {
                lane_param = -1;
            }

            // the obstacle is not shown on the region to be considered.
            if (sl_boundary.start_s > path_range_.second ||
                sl_boundary.end_s < path_range_.first ||
                sl_boundary.start_l > (half_lane_width + lane_param * lane_width) ||
                sl_boundary.end_l < (-half_lane_width + lane_param * lane_width))
            {
                //if the obstacle was in the obstacle_map_ before, now it disappear or go to
                //another lane, break and return, so plotting parallelogram is done
                if (path_time_obstacle_map_.find(obstacle->Id()) !=
                    path_time_obstacle_map_.end())
                {
                    break;
                }
                else
                {
                    //if the obstacle has never been observed in the obstacle_map_ before,
                    //time added to see it would appear or not later
                    relative_time += FLAGS_trajectory_time_resolution;
                    continue;
                }
            }
            PathTimeObstacle path_time_obs;
            //if object never appear before,set start t and s
            if (path_time_obstacle_map_.find(obstacle->Id()) ==
                path_time_obstacle_map_.end())
            {
                path_time_obs.bottom_left =
                    SetPathTimePoint(obstacle->Id(), sl_boundary.start_s, relative_time);
                path_time_obs.upper_left =
                    SetPathTimePoint(obstacle->Id(), sl_boundary.end_s, relative_time);
            }
            path_time_obs.obstacle_id = obstacle->Id();
            path_time_obs.bottom_right =
                SetPathTimePoint(obstacle->Id(), sl_boundary.start_s, relative_time);
            path_time_obs.upper_right =
                SetPathTimePoint(obstacle->Id(), sl_boundary.end_s, relative_time);
            relative_time += FLAGS_trajectory_time_resolution;
        }
    }

    PathTimePoint PathTimeGraph::SetPathTimePoint(const std::string &obstacle_id,
                                                  const double s,
                                                  const double t) const
    {
        PathTimePoint path_time_point;
        path_time_point.s = s;
        path_time_point.t = t;
        path_time_point.obstacle_id = obstacle_id;

        return path_time_point;
    }

    const std::vector<PathTimeObstacle> &PathTimeGraph::GetPathTimeObstacles()
        const
    {
        return path_time_obstacles_;
    }

    bool PathTimeGraph::GetPathTimeObstacle(const std::string &obstacle_id,
                                            PathTimeObstacle *path_time_obstacle)
    {
        if (path_time_obstacle_map_.find(obstacle_id) ==
            path_time_obstacle_map_.end())
        {
            return false;
        }
        *path_time_obstacle = path_time_obstacle_map_[obstacle_id];
        return true;
    }
    /**
 * @brief Given time t, check obstacles' occupied s region
 * @param t
 * @return
 */
    std::vector<std::pair<double, double>> PathTimeGraph::GetPathBlockingIntervals(
        const double t) const
    {
        CHECK(time_range_.first <= t && t <= time_range_.second);

        std::vector<std::pair<double, double>> intervals;
        for (const auto &pt_obstacle : path_time_obstacles_)
        {
            if (t > pt_obstacle.time_upper || t < pt_obstacle.time_lower)
            {
                continue;
            }
            double s_upper =
                lerp(pt_obstacle.upper_left.s, pt_obstacle.upper_left.t,
                     pt_obstacle.upper_right.s, pt_obstacle.upper_right.t, t);

            double s_lower =
                lerp(pt_obstacle.bottom_left.s, pt_obstacle.bottom_left.t,
                     pt_obstacle.bottom_right.s, pt_obstacle.bottom_right.t, t);

            intervals.emplace_back(s_lower, s_upper);
        }
        return intervals;
    }
    //获取所有时刻、所有障碍物的占据范围（即外层vector）
    //内层vector是某个时刻、所有障碍物的占据范围
    std::vector<std::vector<std::pair<double, double>>>
    PathTimeGraph::GetPathBlockingIntervals(const double t_start,
                                            const double t_end,
                                            const double t_resolution)
    {
        std::vector<std::vector<std::pair<double, double>>> intervals;
        for (double t = t_start; t <= t_end; t += t_resolution)
        {
            intervals.push_back(GetPathBlockingIntervals(t));
        }
        return intervals;
    }

    std::pair<double, double> PathTimeGraph::get_path_range() const
    {
        return path_range_;
    }

    std::pair<double, double> PathTimeGraph::get_time_range() const
    {
        return time_range_;
    }
    /**
 * @brief Sampling expanded boundary points around obstacle, below or above the
 *        obstacle
 * @param obstacle_id
 * @param s_dist a very small value, keep the sample points close to obstacle but not
 *        overlapped, also control sampling below or above the obstacle
 * @param t_min_density time sample interval
 * @return sample points upper or lower than obstacle
 */
    std::vector<PathTimePoint> PathTimeGraph::GetObstacleSurroundingPoints(
        const std::string &obstacle_id, const double s_dist, const double t_min_density) const
    {
        CHECK(t_min_density > 0.0);
        std::vector<PathTimePoint> pt_pairs;
        //if the obstacle never show up in obstacle_map_, then return null
        if (path_time_obstacle_map_.find(obstacle_id) ==
            path_time_obstacle_map_.end())
        {
            return pt_pairs;
        }

        const auto &pt_obstacle = path_time_obstacle_map_.at(obstacle_id);

        double s0 = 0.0;
        double s1 = 0.0;

        double t0 = 0.0;
        double t1 = 0.0;
        //sampling above the obstacle
        if (s_dist > 0.0)
        {
            s0 = pt_obstacle.upper_left.s;
            s1 = pt_obstacle.upper_right.s;

            t0 = pt_obstacle.upper_left.t;
            t1 = pt_obstacle.upper_right.t;
        }
        //sampling below the obstacle
        else
        {
            s0 = pt_obstacle.bottom_left.s;
            s1 = pt_obstacle.bottom_right.s;

            t0 = pt_obstacle.bottom_left.t;
            t1 = pt_obstacle.bottom_right.t;
        }

        double time_gap = t1 - t0;
        if (time_gap <= -FLAGS_lattice_epsilon)
            return pt_pairs;
        time_gap = std::fabs(time_gap);

        std::size_t num_sections = std::size_t(time_gap / t_min_density) + 1;
        double t_interval = time_gap / num_sections;

        for (std::size_t i = 0; i <= num_sections; ++i)
        {
            double t = t_interval * i + t0;
            // add s_dist to ensure the sample point don't locate in the obstacle
            double s = lerp(s0, t0, s1, t1, t) + s_dist;

            PathTimePoint ptt;
            ptt.obstacle_id = obstacle_id;
            ptt.t = t;
            ptt.s = s;
            pt_pairs.push_back(std::move(ptt));
        }

        return pt_pairs;
    }

    bool PathTimeGraph::IsObstacleInGraph(const std::string &obstacle_id)
    {
        return path_time_obstacle_map_.find(obstacle_id) !=
               path_time_obstacle_map_.end();
    }

} // namespace PlannerHNS