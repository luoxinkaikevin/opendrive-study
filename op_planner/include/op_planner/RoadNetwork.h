#ifndef ROADNETWORK_H_
#define ROADNETWORK_H_

#include <string>
#include <vector>
#include <sstream>
#include "op_utility/UtilityH.h"
#include <cmath>
#include "box2d.h"
#include "op_planner/RoadElement.h"

namespace PlannerHNS
{

    class Lane;
    class TrafficLight;
    class RoadSegment;

    enum TASK_TYPE
    {
        ASTAR,
        PARKING,
        CROSS_ROAD,
        CRUISE
    };

    enum TrafficSignTypes
    {
        UNKNOWN_SIGN,
        STOP_SIGN,
        MAX_SPEED_SIGN,
        MIN_SPEED_SIGN
    };

    // enum GOAL_TYPE
    // {
    //     PARKSPOT,
    //     STRUCT_ROAD,
    //     HYBRID_ASTAR
    // };

    class GoalPoint
    {
    public:
        int goal_type;
        GPSPoint goal_pose;
        GoalPoint()
        {
            goal_type = 1;
        }
    };

    enum ACTION_TYPE
    {
        FORWARD_ACTION,
        BACKWARD_ACTION,
        STOP_ACTION,
        LEFT_TURN_ACTION,
        RIGHT_TURN_ACTION,
        U_TURN_ACTION,
        SWERVE_ACTION,
        OVERTACK_ACTION,
        START_ACTION,
        SLOWDOWN_ACTION,
        CHANGE_DESTINATION,
        WAITING_ACTION,
        DESTINATION_REACHED,
        UNKOWN_ACTION
    };

    class WayPoint
    {
    public:
        WayPoint();
        WayPoint(const double &x, const double &y, const double &z, const double &a);
        WayPoint(const double &x, const double &y, const double &z,
                 const double &a, const double &speed);
        WayPoint(const double &x, const double &y, const double &z, const double &a, const double &speed, const int &geartmp);

        double kappa() const { return kappa_; }
        double dkappa() const { return dkappa_; }
        double s() const { return s_; }
        double l() const { return l_; }
        //! Setter for s component
        void set_s(const double s) { s_ = s; }
        //! Setter for l component
        void set_l(const double l) { l_ = l; }
        void set_kappa(const double kappa) { kappa_ = kappa; }
        void set_dkappa(const double dkappa) { dkappa_ = dkappa; }

    public:
        GPSPoint pos;
        //档位
        int gear;
        double v;
        double acceleration;
        // 代价
        double cost;
        // relative time
        double timeCost;
        //lane width in this waypoint
        double laneWidth;

        TASK_TYPE task;
        std::vector<std::pair<ACTION_TYPE, double>> actionCost;

        int laneId;
        int id;
        int stopLineID;
        int crosswalkID;
        int trafficLigntID;
        bool enable_left_change;
        bool enable_right_change;
        bool enable_road_occupation_left = false;

        Lane *pLane;
        WayPoint *pLeft;
        WayPoint *pRight;
        int LeftPointId;
        int RightPointId;
        int LeftLnId;
        int RightLnId;
        std::vector<int> toIds;
        std::vector<int> fromIds;
        std::vector<WayPoint *> pFronts;
        std::vector<WayPoint *> pBacks;

    private:
        double s_;
        double l_;
        double kappa_;
        double dkappa_;
        void Init();
    };

    class RelativeInfo
    {
    public:
        double perp_distance;
        double to_front_distance; //negative
        double from_back_distance;
        int iFront;
        int iBack;
        int iGlobalPath;
        WayPoint perp_point;
        double angle_diff; // degrees
        bool bBefore;
        bool bAfter;
        double after_angle;

        RelativeInfo()
        {
            after_angle = 0;
            bBefore = false;
            bAfter = false;
            perp_distance = 0;
            to_front_distance = 0;
            from_back_distance = 0;
            iFront = 0;
            iBack = 0;
            iGlobalPath = 0;
            angle_diff = 0;
        }
    };

    class ParkSpot
    {
    public:
        int id;
        // int laneId;
        int roadId;
        math::Box2d spot_box;

        ParkSpot()
        {
            id = 0;
            // laneId = 0;
            roadId = 0;
        }
    };

    class Crossing
    {
    public:
        int id;
        int roadId;
        std::vector<int> laneIds;
        math::Polygon2d crosswalk_box;

        Crossing()
        {
            id = 0;
            roadId = 0;
        }
    };

    class StopLine
    {
    public:
        int id;
        std::vector<int> laneIds;
        int roadId;
        int trafficLightID;
        int stopSignID;
        GPSPoint point;
        Lane *pLane;
        int linkID;
        int8_t orientation_;

        StopLine()
        {
            id = 0;
            roadId = 0;
            pLane = 0;
            trafficLightID = -1;
            stopSignID = -1;
            linkID = 0;
            orientation_ = 0;
        }
    };

    class TrafficSign
    {
    public:
        int id;
        int laneId;
        int roadId;

        GPSPoint pos;
        TrafficSignTypes signType;
        double value;
        double fromValue;
        double toValue;
        std::string strValue;
        timespec timeValue;
        timespec fromTimeValue;
        timespec toTimeValue;

        Lane *pLane;

        TrafficSign()
        {
            id = 0;
            laneId = 0;
            roadId = 0;
            signType = UNKNOWN_SIGN;
            value = 0;
            fromValue = 0;
            toValue = 0;
            //		timeValue	= 0;
            //		fromTimeValue = 0;
            //		toTimeValue	= 0;
            pLane = 0;
        }
    };


    class TrafficLight
    {
    public:
        int id;
        int roadId;
        GPSPoint pos;
        int left_or_right;//0 left,1-right

        TrafficLight()
        {
            id = 0;
            left_or_right=-1;

        }

        // bool CheckLane(const int &laneId)
        // {
        //     for (unsigned int i = 0; i < laneIds.size(); i++)
        //     {
        //         if (laneId == laneIds.at(i))
        //             return true;
        //     }
        //     return false;
        // }
    };

    class RoadSegment
    {
    public:
        int id;
        std::vector<Lane> Lanes;

        RoadSegment()
        {
            id = 0;
        }
    };

    class Lane
    {
    public:
        int id;
        int roadId;
        int laneSectionId;
        int type;
        int num; //lane number in the road segment from left to right
        double speed;
        double length;
        double dir;
        double width;
        std::string roadtype_;
        std::vector<WayPoint> points;
        std::vector<TrafficLight> trafficlights;
        std::vector<StopLine> stopLines;
        std::vector<Crossing> crossings;

        std::vector<int> fromIds;
        std::vector<int> toIds;
        std::vector<Lane *> fromLanes;
        std::vector<Lane *> toLanes;

        int LeftLaneId;
        int RightLaneId;
        Lane *pLeftLane;
        Lane *pRightLane;
        RoadSegment *pRoad;

        Lane()
        {
            LeftLaneId = 0;
            RightLaneId = 0;
            id = 0;
            num = 0;
            speed = 0;
            length = 0;
            dir = 0;
            width = 0;
            pLeftLane = nullptr;
            pRightLane = nullptr;
            pRoad = nullptr;
            roadId = 0;
            laneSectionId = 0;
            roadtype_ = "town";
        }
    };

    class RoadNetwork
    {
    public:
        std::vector<RoadSegment> roadSegments;
        std::unordered_map<int, Crossing> crossing_map;
        std::unordered_map<int, StopLine> stopLine_map;
        std::unordered_map<int, TrafficLight> trafficLight_map;
        std::unordered_map<int, ParkSpot> ParkSpot_map;
    };

    class DetectedObject
    {
    public:
        DetectedObject();

        void set_Id(int id) { Id_ = std::to_string(id); }
        const std::string &Id() const { return Id_; }
        bool IsVirtual() const { return is_virtual_; }
        bool HasTrajectory() const
        {
            if (predTrajectory.empty())
                return false;
            return true;
        }
        double Speed() const { return center.v; }

        const math::Polygon2d &PerceptionPolygon() const
        {
            return perception_polygon_;
        }
        void set_polygon()
        {
            math::Polygon2d::ComputeConvexHull(contour, &perception_polygon_);
        }

        math::Box2d GetBoundingBox(const WayPoint &point) const
        {
            return math::Box2d({point.pos.x, point.pos.y, 0, 0},
                               point.pos.a, l, w);
        }

        math::Box2d PerceptionBoundingBox() const
        {
            return perception_bounding_box_;
        }

        void set_box()
        {
            math::Box2d box_({center.pos.x, center.pos.y, 0, 0},
                             center.pos.a, l, w);
            perception_bounding_box_ = box_;
        }

        void set_perception_bounding_box(math::Box2d box_)
        {
            perception_bounding_box_ = box_;
        }

        WayPoint GetPointAtTime(const double relative_time) const;

    public:
        int id;
        uint8_t label;
        WayPoint center;
        std::vector<GPSPoint> contour;
        std::vector<WayPoint> predTrajectory;
        WayPoint *pClosestWaypoints;
        double w;
        double l;
        double h;
        bool bDirection;
        bool bVelocity;
        bool is_virtual_;
        bool is_static;

    private:
        std::string Id_;
        math::Box2d perception_bounding_box_;
        math::Polygon2d perception_polygon_;
    };

    class CAR_BASIC_INFO
    {
    public:
        // 转弯半径
        double turning_radius;
        //   轴距
        double wheel_base;
        //   最大前进速度
        double max_speed_forward;
        //   最小前进速度
        double min_speed_forward;
        //   最大后退速度
        double max_speed_backword;
        //   最大转向值
        double max_steer_value;
        //   最小转向值
        double min_steer_value;
        //   最大刹车值
        double max_brake_value;
        //   最小刹车值
        double min_brake_value;
        //   最大转向角度
        double max_steer_angle;
        //   最小的转向角度
        double min_steer_angle;
        // 车辆长度
        double length;
        //   车辆宽度
        double width;
        //   最大加速度
        double max_acceleration;
        //   最大减速度
        double max_deceleration;
        //
        double mass;

        double wheel_radius;
        double steer_ratio;

        double front_axis_to_car_front;
        CAR_BASIC_INFO()
        {
            turning_radius = 6.2;
            wheel_base = 2.56;
            max_speed_forward = 80.0; //km/h
            min_speed_forward = 0.0;
            max_speed_backword = 20.0;
            max_steer_value = 500; //角度
            min_steer_value = -500;
            max_brake_value = 1;
            min_brake_value = 0;
            length = 4.35; //m
            width = 1.82;
            max_acceleration = 5.0;  // m/s2
            max_deceleration = -5.0; // 1/3 G
            mass = 1700;             //kg
            wheel_radius = 0.34;
            steer_ratio = 14.5;
            max_steer_angle = max_steer_value / steer_ratio * DEG2RAD; //弧度
            min_steer_angle = -max_steer_value / steer_ratio * DEG2RAD;
            front_axis_to_car_front = length / 2 - length / 2;
        }

        double CalcMaxSteeringAngle()
        {
            return max_steer_angle; //asin(wheel_base/turning_radius);
        }

        double BoundSpeed(double s)
        {
            if (s > 0 && s > max_speed_forward)
                return max_speed_forward;
            if (s < 0 && s < max_speed_backword)
                return max_speed_backword;
            return s;
        }

        double BoundSteerAngle(double a)
        {
            if (a > max_steer_angle)
                return max_steer_angle;
            if (a < min_steer_angle)
                return min_steer_angle;

            return a;
        }

        double BoundSteerValue(double v)
        {
            if (v >= max_steer_value)
                return max_steer_value;
            if (v <= min_steer_value)
                return min_steer_value;

            return v;
        }
    };

    class PlanningTarget
    {
    public:
        bool has_stop_point;
        double cruise_speed;
        StopPoint stop_point;
        PlanningTarget()
        {
            has_stop_point = false;
            cruise_speed = 80 / 3.6;
        }
    };

    enum STATE_TYPE
    {
        SLOW,
        BLOCK,
        PASS
    };

    class RoadState
    {
    public:
        int road_id;
        bool bRight;
        STATE_TYPE state_type;
        RoadState()
        {
            road_id = -1;
            state_type = PASS;
            bRight = true;
        }
    };

    //Must keep opendrive map  right lane num to be negative
    class RoadBlockInfo
    {
    public:
        std::vector<RoadState> road_info_list;
        bool findRoad(int roadid, int lane_num)
        {
            for (auto road_ids_i : road_info_list)
            {
                if (roadid == road_ids_i.road_id)
                {
                    if ((lane_num < 0 && road_ids_i.bRight) ||
                        (lane_num > 0 && !road_ids_i.bRight))
                        return true;
                }
            }
            return false;
        }
    };

    class TrafficState
    {
    public:
        int left_right;
        int color;

        TrafficState()
        {
            left_right = -1;
            color = -1;
        }

    };

    template <typename T>
    bool WithinRange(const T v, const T lower, const T upper)
    {
        return lower <= v && v <= upper;
    }

    template <typename T>
    bool WithinBound(T start, T end, T value)
    {
        return value >= start && value <= end;
    }

} // namespace PlannerHNS

#endif /* ROADNETWORK_H_ */
