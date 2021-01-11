#include "op_planner/reference_line/reference_line_base.h"

namespace PlannerHNS
{
    std::fstream a;

    bool ReferenceLine::Shrink(
        const std::vector<WayPoint> &global_path, WayPoint &current_pose,
        const double &lookBack_Distance, const double &lookAhand_Distance)
    {
        if (global_path.size() < 2)
            return false;

        close_index =
            PlanningHelpers::GetClosestNextPointIndexFast(global_path, current_pose, last_index);

        current_reference_pose_ = global_path.at(close_index);
        current_reference_pose_.set_s(0);
        current_pose.laneWidth = global_path.at(close_index).laneWidth;
        current_pose.task = global_path.at(close_index).task;
        current_road_state.road_id = global_path.at(close_index).pLane->roadId;
        if (global_path.at(close_index).pLane->num > 0)
        {
            current_road_state.bRight = false;
        }
        else
        {
            current_road_state.bRight = true;
        }

        ADEBUG << "current_road_state id: " << current_road_state.road_id
               << " current_road_state lane direction: " << current_road_state.bRight;

        ADEBUG << "Current pose index in global path:  " << close_index;
        if (close_index + 1 >= global_path.size())
            close_index = global_path.size() - 2;

        reference_points_.clear();
        accumulated_s.clear();

        if (global_path.size() - close_index < 180)
        {
            near_destination_ = true;
        }

        WayPoint temp_p;
        double d = 0;
        for (int i = close_index; i >= 0; i--)
        {
            temp_p = global_path.at(i);
            temp_p.set_s(global_path.at(i).s() - global_path.at(close_index).s());
            accumulated_s.insert(accumulated_s.begin(), temp_p.s());
            // 在extractedPath.begin()增加元素originalPath.at(i)
            reference_points_.insert(reference_points_.begin(), temp_p);
            if (i < global_path.size())
                d += global_path.at(i).pos.DistanceTo(global_path.at(i + 1).pos);
            if (d > lookBack_Distance)
                break;
        }
        // 提取全局路径上车前方lookAhand_Distance的路点
        d = 0;
        road_list.road_info_list.clear();
        for (int i = close_index + 1; i < (int)global_path.size(); i++)
        {
            temp_p = global_path.at(i);
            temp_p.set_s(global_path.at(i).s() - global_path.at(close_index).s());
            reference_points_.emplace_back(temp_p);
            accumulated_s.emplace_back(temp_p.s());

            if ((global_path.at(i).pLane->roadId != current_road_state.road_id) &&
                (!road_list.findRoad(global_path.at(i).pLane->roadId, global_path.at(i).pLane->num)))
            {
                RoadState road_state;
                road_state.road_id = global_path.at(i).pLane->roadId;
                if (global_path.at(i).pLane->num > 0)
                    road_state.bRight = false;
                else
                    road_state.bRight = true;
                // ADEBUG << "road id: " << road_state.road_id
                //        << " lane direction: " << road_state.bRight;
                road_list.road_info_list.emplace_back(road_state);
            }

            if (i > 0)
                d += global_path.at(i).pos.DistanceTo(global_path.at(i - 1).pos);
            if (d > lookAhand_Distance)
                break;
        }

        last_index = close_index;
        // 如果提取出来的路径点过少
        if (reference_points_.size() < 2)
        {
            ADEBUG << "### Extracted reference line is too Small, Size = "
                   << reference_points_.size();
            return false;
        }

        return true;
    }

    int ReferenceLine::getBlockInfo(RoadBlockInfo current_road_info)
    {
        for (auto current_road_i : current_road_info.road_info_list)
        {

            ADEBUG << "current_road_state id: " << current_road_state.road_id
                   << " current_road_state lane direction: " << current_road_state.bRight;

            ADEBUG << "current_road_i id: " << current_road_i.road_id
                   << " current_road_i lane direction: " << current_road_i.bRight;

            if ((current_road_state.road_id == current_road_i.road_id) &&
                (current_road_state.bRight == current_road_i.bRight))
            {
                ADEBUG << "current road blocked";
                return -1;
            }
            else
            {
                for (auto road_index : road_list.road_info_list)
                {
                    ADEBUG << "road_index id: " << road_index.road_id
                           << " road_index lane direction: " << road_index.bRight;

                    ADEBUG << "current_road_i id: " << current_road_i.road_id
                           << " current_road_i lane direction: " << current_road_i.bRight;

                    if ((road_index.road_id == current_road_i.road_id) &&
                        (road_index.bRight == current_road_i.bRight))
                    {
                        return 1;
                    }
                }
            }
        }
        return 0;
    }

    void ReferenceLine::getTrafficState()
    {
        crosswalk_overlaps_.clear();
        for (std::size_t i = close_index; i < reference_points_.size(); i++)
        {
            auto path_point = reference_points_.at(i);
            if (path_point.crosswalkID > 0)
            {

                auto stopline_index = GetNearStopLine(i);
                ADEBUG << "Find crosswalk: " << path_point.s()
                       << " stopline_index: " << stopline_index;
                if (stopline_index != -1)
                {
                    PathOverlap crosswalk_temp;
                    crosswalk_temp.end_s = reference_points_.at(stopline_index).s()+3;
                    crosswalk_temp.object_id = path_point.crosswalkID;
                    //停止线前30m开始减速
                    crosswalk_temp.start_s =
                        crosswalk_temp.end_s - FLAGS_stop_line_look_backward_distance;
                    // std::max(crosswalk_temp.end_s - FLAGS_stop_line_look_backward_distance, 0.0);

                    ADEBUG << "crosswalk_temp.start_s: " << crosswalk_temp.start_s
                           << " crosswalk_temp.end_s: " << crosswalk_temp.end_s;

                    crosswalk_overlaps_.emplace_back(crosswalk_temp);
                }
                else
                {
                    PathOverlap crosswalk_temp;
                    // 对于十字路口没有停止线的情况，虚拟一个斑马线中心点位置前2m的停止线
                    crosswalk_temp.end_s = path_point.s() - 4.0;
                    crosswalk_temp.object_id = path_point.crosswalkID;
                    //停止线前5m开始减速
                    // crosswalk_temp.start_s = crosswalk_temp.end_s - 5;

                    crosswalk_temp.start_s =
                        crosswalk_temp.end_s - FLAGS_stop_line_look_backward_distance;
                    // std::max(crosswalk_temp.end_s - 5, 0.0);

                    ADEBUG << "crosswalk_temp.start_s: " << crosswalk_temp.start_s
                           << " crosswalk_temp.end_s: " << crosswalk_temp.end_s;
                    crosswalk_overlaps_.emplace_back(crosswalk_temp);
                }
            }

            if (path_point.trafficLigntID > 0)
            {
                auto stopline_index = GetTrafficNearStopLine(i);
                if (stopline_index >= 0)
                {
                    if (i - close_index < 90)
                    {
                        enable_traffic_light = true;
                        // std::cout<<"enable_traffic_light:  true"<<std::endl;
                    }

                    PathOverlap trafficlight_temp;
                    trafficlight_temp.end_s = reference_points_.at(stopline_index).s()+3;
                    trafficlight_temp.object_id = path_point.trafficLigntID;
                    //停止线前30m开始减速
                    trafficlight_temp.start_s =
                        trafficlight_temp.end_s - FLAGS_stop_line_look_backward_distance;
                    signal_light_overlaps_.emplace_back(trafficlight_temp);

                    ADEBUG << "traffic light index: " << i
                           << " close_index: " << close_index
                           << " trafficLigntID: " << path_point.trafficLigntID
                           << " trafficlight_temp.end_s: " << trafficlight_temp.end_s;
                }
            }
        }
    }

    int ReferenceLine::GetNearStopLine(std::size_t crossWalkIndex)
    {
        if (crossWalkIndex < 1)
            return -1;
        for (std::size_t i = crossWalkIndex; i > 0; i--)
        {
            // look back 8m(0.5m density)
            if (crossWalkIndex - i > 25)
            {
                return -1;
            }

            if (reference_points_.at(i).stopLineID != -1)
            {
                return i;
            }
        }
        return -1;
    }

    int ReferenceLine::GetTrafficNearStopLine(std::size_t crossWalkIndex)
    {
        if (crossWalkIndex < 1)
            return -1;
        for (std::size_t i = crossWalkIndex; i > 0; i--)
        {
            // look back 8m(0.5m density)
            if (crossWalkIndex - i > 45)
            {
                return -1;
            }

            if (reference_points_.at(i).stopLineID != -1)
            {
                return i;
            }
        }
        return -1;
    }

    WayPoint ReferenceLine::GetNearestReferencePoint(const GPSPoint &xy) const
    {
        double min_dist = std::numeric_limits<double>::max();
        int min_index = 0;
        for (std::size_t i = 0; i < reference_points_.size(); ++i)
        {
            const double distance = xy.DistanceTo(reference_points_.at(i).pos);
            if (distance < min_dist)
            {
                min_dist = distance;
                min_index = i;
            }
        }
        return reference_points_[min_index];
    }

    std::size_t ReferenceLine::GetNearestReferenceIndex(const GPSPoint &xy) const
    {
        double min_dist = std::numeric_limits<double>::max();
        std::size_t min_index = 0;
        for (std::size_t i = 0; i < reference_points_.size(); ++i)
        {
            const double distance = xy.DistanceTo(reference_points_.at(i).pos);
            if (distance < min_dist)
            {
                min_dist = distance;
                min_index = i;
            }
        }
        return min_index;
    }

    bool ReferenceLine::IsBlockRoad(const math::Box2d &box,
                                    double gap) const
    {
        const GPSPoint center = box.center();
        const double radius = (box.diagonal() / 2.0 + gap) + kMathEpsilon;
        auto path_wp = GetNearestReferencePoint(center);
        // for long obj
        if (box.length() / box.width() > 5)
        {
            auto obj_distance = center.DistanceTo(path_wp.pos) * 2;
            if (obj_distance < gap + box.width())
            {
                ADEBUG << "Bolcked----for long obj: " << obj_distance;
                return true;
            }
            else
            {
                return false;
            }
        }

        GPSPoint left;
        GPSPoint right;
        double half_lane = path_wp.laneWidth / 2;
        double a = path_wp.pos.a + M_PI_2;
        left.x = path_wp.pos.x + half_lane * cos(a);
        left.y = path_wp.pos.y + half_lane * sin(a);
        a = path_wp.pos.a - M_PI_2;
        right.x = path_wp.pos.x + half_lane * cos(a);
        right.y = path_wp.pos.y + half_lane * sin(a);

        double border =
            std::max(center.DistanceTo(left), center.DistanceTo(right));
        if (border < radius)
        {
            ADEBUG << "Bolcked----box border: " << border << " radius: " << radius;
            return true;
        }

        return false;
    }

    WayPoint ReferenceLine::GetNearestReferencePoint(const double s) const
    {
        if (s < accumulated_s.front() - 1e-2)
        {
            AWARN << "The requested s " << s << " < 0";
            return reference_points_.front();
        }
        if (s > accumulated_s.back() + 1e-2)
        {
            AWARN << "The requested s " << s << " > reference line length "
                  << accumulated_s.back();
            return reference_points_.back();
        }
        auto it_lower =
            std::lower_bound(accumulated_s.begin(), accumulated_s.end(), s);
        if (it_lower == accumulated_s.begin())
        {
            return reference_points_.front();
        }
        else
        {
            auto index = std::distance(accumulated_s.begin(), it_lower);
            if (std::fabs(accumulated_s[index - 1] - s) <
                std::fabs(accumulated_s[index] - s))
            {
                return reference_points_[index - 1];
            }
            else
            {
                return reference_points_[index];
            }
        }
    }

    std::size_t ReferenceLine::GetNearestReferenceIndex(double s) const
    {
        if (s < accumulated_s.front() - 1e-2)
        {
            AWARN << "The requested s " << s << " < 0";
            return 0;
        }
        if (s > reference_points_.back().s() + 1e-2)
        {
            AWARN << "The requested s " << s << " > reference line length "
                  << reference_points_.back().s();
            return reference_points_.size() - 1;
        }
        auto it_lower =
            std::lower_bound(accumulated_s.begin(), accumulated_s.end(), s);
        return std::distance(accumulated_s.begin(), it_lower);
    }

    std::vector<WayPoint> ReferenceLine::GetReferencePoints(
        double start_s, double end_s) const
    {
        if (start_s < 0.0)
        {
            start_s = 0.0;
        }
        if (end_s > Length())
        {
            end_s = Length();
        }
        std::vector<WayPoint> ref_points;
        auto start_index = GetNearestReferenceIndex(start_s);
        auto end_index = GetNearestReferenceIndex(end_s);
        if (start_index < end_index)
        {
            ref_points.assign(reference_points_.begin() + start_index,
                              reference_points_.begin() + end_index);
        }
        return ref_points;
    }

    const std::vector<WayPoint> &ReferenceLine::reference_points() const
    {
        return reference_points_;
    }

    double ReferenceLine::GetLaneWidth(const double s) const
    {
        if (reference_points_.empty())
        {
            return -1;
        }

        return GetNearestReferencePoint(s).laneWidth;
    }

    bool ReferenceLine::IsOnRoad(const GPSPoint &vec2d_point) const
    {
        auto sl_point =
            PlanningHelpers::GetPathFrenetCoordinate(reference_points_, vec2d_point);

        return IsOnRoad(sl_point);
    }

    bool ReferenceLine::IsOnRoad(const SLBoundary &sl_boundary) const
    {
        if (sl_boundary.end_s < 0 || sl_boundary.start_s > Length() + 0.5)
        {
            return false;
        }

        double lane_width = current_reference_pose_.laneWidth;

        return !(sl_boundary.start_l > lane_width ||
                 sl_boundary.end_l < -lane_width);
    }

    bool ReferenceLine::IsOnRoad(const std::pair<double, double> &sl_point) const
    {
        if (sl_point.first <= 0 || sl_point.first > reference_points_.back().s())
        {
            return false;
        }
        double lane_width = GetLaneWidth(sl_point.first);

        return !(sl_point.second < -lane_width || sl_point.second > lane_width);
    }

    bool ReferenceLine::IsOnRoad(const math::Box2d &box) const
    {
        SLBoundary sl_boundary;
        if (!GetSLBoundary(box, &sl_boundary))
        {
            AERROR << "Failed to get sl boundary for box " << box.DebugString();
            return false;
        }

        return IsOnRoad(sl_boundary);
    }

    bool ReferenceLine::GetSLBoundary(const math::Box2d &box,
                                      SLBoundary *const sl_boundary) const
    {
        double start_s(std::numeric_limits<double>::max());
        double end_s(std::numeric_limits<double>::lowest());
        double start_l(std::numeric_limits<double>::max());
        double end_l(std::numeric_limits<double>::lowest());
        std::vector<GPSPoint> corners;
        box.GetAllCorners(&corners);
        for (const auto &point : corners)
        {
            auto sl_point =
                PlanningHelpers::GetPathFrenetCoordinate(reference_points_, point);

            start_s = std::fmin(start_s, sl_point.first);
            end_s = std::fmax(end_s, sl_point.first);
            start_l = std::fmin(start_l, sl_point.second);
            end_l = std::fmax(end_l, sl_point.second);
        }
        sl_boundary->start_s = start_s;
        sl_boundary->end_s = end_s;
        sl_boundary->start_l = start_l;
        sl_boundary->end_l = end_l;
        return true;
    }

    bool ReferenceLine::GetSLBoundary(const math::Polygon2d &polygon,
                                      SLBoundary *const sl_boundary) const
    {
        double start_s(std::numeric_limits<double>::max());
        double end_s(std::numeric_limits<double>::lowest());
        double start_l(std::numeric_limits<double>::max());
        double end_l(std::numeric_limits<double>::lowest());
        for (const auto &point : polygon.points())
        {
            auto sl_point =
                PlanningHelpers::GetPathFrenetCoordinate(reference_points_, point);

            start_s = std::fmin(start_s, sl_point.first);
            end_s = std::fmax(end_s, sl_point.first);
            start_l = std::fmin(start_l, sl_point.second);
            end_l = std::fmax(end_l, sl_point.second);
        }
        sl_boundary->start_s = start_s;
        sl_boundary->end_s = end_s;
        sl_boundary->start_l = start_l;
        sl_boundary->end_l = end_l;
        return true;
    }

    bool ReferenceLine::HasOverlap(const math::Box2d &box) const
    {
        SLBoundary sl_boundary;
        if (!GetSLBoundary(box, &sl_boundary))
        {
            AERROR << "Failed to get sl boundary for box " << box.DebugString();
            return false;
        }

        ADEBUG << "sl_boundary.start_s " << sl_boundary.start_s
               << " sl_boundary.end_s " << sl_boundary.end_s
               << " sl_boundary.start_l " << sl_boundary.start_l
               << " sl_boundary.end_l " << sl_boundary.end_l;

        if (sl_boundary.end_s < 0 || sl_boundary.start_s > Length())
        {
            return false;
        }
        if (sl_boundary.start_l * sl_boundary.end_l < 0)
        {
            return false;
        }

        const double mid_s = (sl_boundary.start_s + sl_boundary.end_s) / 2.0;
        if (mid_s < 0 || mid_s > Length())
        {
            ADEBUG << "ref_s out of range:" << mid_s;
            return false;
        }
        double lane_width = GetLaneWidth(mid_s);

        if (sl_boundary.start_l > 0)
        {
            return sl_boundary.start_l < lane_width;
        }
        else
        {
            return sl_boundary.end_l > -lane_width;
        }
    }

    std::string ReferenceLine::DebugString() const
    {
        // const auto limit =
        //     std::min(reference_points_.size(),
        //              static_cast<size_t>(FLAGS_trajectory_point_num_for_debug));
        // return apollo::common::util::StrCat(
        //     "point num:", reference_points_.size(),
        //     apollo::common::util::PrintDebugStringIter(
        //         reference_points_.begin(), reference_points_.begin() + limit, ""));
    }

} // namespace PlannerHNS
