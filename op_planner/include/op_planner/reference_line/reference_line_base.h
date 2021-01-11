#pragma once

#include <algorithm>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "boost/math/tools/minima.hpp"
#include "op_planner/planning_gflags.h"
#include "op_planner/RoadNetwork.h"
#include "op_planner/PlanningHelpers.h"
#include "op_planner/RoadElement.h"
#include "op_planner/OpendriveMapLoader.h"
#include "op_planner/log.h"

namespace PlannerHNS
{

    class ReferenceLine
    {
    public:
        ReferenceLine() = default;

        double speed_limit_1;
        double speed_limit_2;
        WayPoint GetNearestReferencePoint(const GPSPoint &xy) const;
        std::size_t GetNearestReferenceIndex(const GPSPoint &xy) const;

        int getBlockInfo(RoadBlockInfo current_road_info);

        bool Shrink(
            const std::vector<WayPoint> &global_path, WayPoint &current_pose,
            const double &lookBack_Distance, const double &lookAhand_Distance);

        WayPoint GetNearestReferencePoint(const double s) const;
        std::size_t GetNearestReferenceIndex(const double s) const;
        std::vector<WayPoint> GetReferencePoints(
            double start_s, double end_s) const;
        double FindMinDistancePoint(const WayPoint &p0,
                                    const double s0,
                                    const WayPoint &p1,
                                    const double s1, const double x,
                                    const double y);
        const std::vector<WayPoint> &reference_points() const;
        double GetLaneWidth(const double s) const;
        bool IsOnRoad(const GPSPoint &vec2d_point) const;
        bool IsOnRoad(const SLBoundary &sl_boundary) const;
        bool IsOnRoad(const std::pair<double, double> &sl_point) const;
        bool IsOnRoad(const math::Box2d &box) const;
        bool GetSLBoundary(const math::Box2d &box,
                           SLBoundary *const sl_boundary) const;
        bool GetSLBoundary(const math::Polygon2d &polygon,
                           SLBoundary *const sl_boundary) const;

        /**
   * @brief check if any part of the box has overlap with the road.
   */
        bool HasOverlap(const math::Box2d &box) const;
        double Length() const
        {
            return reference_points_.back().s() - reference_points_.front().s();
        }
        std::string DebugString() const;

        bool GetApproximateSLBoundary(
            const math::Box2d &box, const double start_s, const double end_s,
            SLBoundary *const sl_boundary) const;
        bool is_near_destination() const { return near_destination_; }

        WayPoint current_reference_pose() const { return current_reference_pose_; }

        /**
   * @brief Check if a box is blocking the road surface. The crieria is to check
   * whether the remaining space on the road surface is larger than the provided
   * gap space.
   * @param boxed the provided box
   * @param gap check the gap of the space
   * @return true if the box blocks the road.
   */
        bool IsBlockRoad(const math::Box2d &box,
                         double gap) const;
        void getTrafficState();
        const std::vector<PathOverlap> &crosswalk_overlaps() const
        {
            return crosswalk_overlaps_;
        }

        const std::vector<PathOverlap> &signal_overlaps() const
        {
            return signal_light_overlaps_;
        }

        const RoadState &Current_road_state() const
        {
            return current_road_state;
        }

        const bool &Enable_traffic_light() const
        {
            return enable_traffic_light;
        }

        const int &Close_index() const
        {
            return close_index;
        }


    private:
        int GetNearStopLine(std::size_t crossWalkIndex);
        int GetTrafficNearStopLine(std::size_t crossWalkIndex);

    private:
        std::vector<WayPoint> reference_points_;
        std::vector<PathOverlap> crosswalk_overlaps_;
        std::vector<PathOverlap> signal_light_overlaps_;
        WayPoint current_reference_pose_;
        int last_index = 0;

        std::vector<double> accumulated_s;
        bool near_destination_ = false;
        RoadBlockInfo road_list;
        RoadState current_road_state;
        int close_index=0;
        bool enable_traffic_light=false;
    };

} // namespace PlannerHNS
