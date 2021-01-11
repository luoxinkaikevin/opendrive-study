#pragma once

#include <string>
#include <vector>
#include <sstream>
#include <utility>
#include "op_utility/UtilityH.h"
#include <cmath>
#include "op_planner/vec2d.h"
#include <unordered_map>

namespace PlannerHNS
{

    template <typename I, typename T>
    class IndexedList
    {
    public:
        /**
   * @brief copy object into the container. If the id is already exist,
   * overwrite the object in the container.
   * @param id the id of the object
   * @param object the const reference of the objected to be copied to the
   * container.
   * @return The pointer to the object in the container.
   */
        T *Add(const I id, const T &object)
        {
            // auto obs = Find(id);
            // if (obs)
            // {
            //     // AWARN << "object " << id << " is already in container";
            //     *obs = object;
            //     return obs;
            // }
            // else
            {
                object_dict_.insert({id, object});
                auto *ptr = &object_dict_.at(id);
                object_list_.push_back(ptr);
                return ptr;
            }
        }

        /**
   * @brief Find object by id in the container
   * @param id the id of the object
   * @return the raw pointer to the object if found.
   * @return nullptr if the object is not found.
   */
        T *Find(const I id)
        {
            // return apollo::common::util::FindOrNull(object_dict_, id);
            for(auto i: object_dict_)
            {
                if(i.first==id)
                {
return &object_dict_.at(id);
                }

            }
            return nullptr;
        }

        /**
   * @brief Find object by id in the container
   * @param id the id of the object
   * @return the raw pointer to the object if found.
   * @return nullptr if the object is not found.
   */
        const T *Find(const I id) const
        {
            for(auto i: object_dict_)
            {
                if(i.first==id)
                {
return &object_dict_.at(id);
                }

            }
            return nullptr;
        }

        /**
   * @brief List all the items in the container.
   * @return the list of const raw pointers of the objects in the container.
   */
        const std::vector<const T *> &Items() const { return object_list_; }

    private:
        std::vector<const T *> object_list_;
        std::unordered_map<I, T> object_dict_;
    };

    /////////////////////////////////////////////////////////////////
    // The start_s and end_s are longitudinal values.
    // start_s <= end_s.
    //
    //              end_s
    //                ^
    //                |
    //          S  direction
    //                |
    //            start_s
    //
    // The start_l and end_l are lateral values.
    // start_l <= end_l. Left side of the reference line is positive,
    // and right side of the reference line is negative.
    //  end_l  <-----L direction---- start_l
    /////////////////////////////////////////////////////////////////
    class GPSPoint : public Vec2d
    {
    public:
        GPSPoint();
        virtual ~GPSPoint() = default;
        GPSPoint(const double &x, const double &y, const double &z, const double &a);

        //! Creates a unit-vector with a given angle to the positive x semi-axis
        static GPSPoint CreateUnitVec2d(const double angle);

        double Length() const { return std::hypot(x, y); }
        //! Gets the length of the vector
        double LengthToOrigin() const;

        //! Gets the squared length of the vector
        double LengthSquare() const;

        //! Gets the angle between the vector and the positive x semi-axis
        double AngleWithOrigin() const;

        //! Returns the unit vector that is co-linear with this vector
        void Normalize();

        //! Returns the distance to the given vector
        double DistanceTo(const GPSPoint &other) const;

        //! Returns the squared distance to the given vector
        double DistanceSquareTo(const GPSPoint &other) const;

        //! Returns the "cross" product between these two GPSPoint (non-standard).
        double CrossProd(const GPSPoint &other) const;

        //! Returns the inner product between these two GPSPoint.
        double InnerProd(const GPSPoint &other) const;

        //! rotate the vector by angle.
        GPSPoint rotate(const double angle) const;

        //! Sums two GPSPoint
        GPSPoint operator+(const GPSPoint &other) const;

        //! Subtracts two GPSPoint
        GPSPoint operator-(const GPSPoint &other) const;

        //! Multiplies GPSPoint by a scalar
        GPSPoint operator*(const double ratio) const;

        //! Divides GPSPoint by a scalar
        GPSPoint operator/(const double ratio) const;

        //! Sums another GPSPoint to the current one
        GPSPoint &operator+=(const GPSPoint &other);

        //! Subtracts another GPSPoint to the current one
        GPSPoint &operator-=(const GPSPoint &other);

        //! Multiplies this GPSPoint by a scalar
        GPSPoint &operator*=(const double ratio);

        //! Divides this GPSPoint by a scalar
        GPSPoint &operator/=(const double ratio);

        //! Compares two GPSPoint
        bool operator==(const GPSPoint &other) const;

        //! Returns a human-readable string representing this object
        std::string ToString() const;

        double x;
        double y;
        double z;
        double a;
    };
    struct SLBoundary
    {
        double start_s;
        double end_s;
        double start_l;
        double end_l;
    };

    struct PathTimePoint
    {
        double t;
        double s;
        std::string obstacle_id;
    };

    struct SamplePoint
    {
        PathTimePoint path_time_point;
        double ref_v;
    };

    struct PathTimeObstacle
    {
        std::string obstacle_id;
        PathTimePoint bottom_left;
        PathTimePoint upper_left;
        PathTimePoint upper_right;
        PathTimePoint bottom_right;
        double time_lower;
        double time_upper;
        double path_lower;
        double path_upper;
    };

    enum StopType
    {
        HARD,
        SOFT
    };
    class StopPoint
    {
    public:
        double s = std::numeric_limits<double>::infinity();
        StopType type = HARD;
    };

    enum ObjectDecisionType
    {
        ignore,
        stop,
        follow,
        yield,
        overtake,
        nudge,
        sidepass,
        avoid,
        unknow
    };

    // This message is deprecated

    enum StopReasonCode
    {
        STOP_REASON_HEAD_VEHICLE,
        STOP_REASON_DESTINATION,
        STOP_REASON_PEDESTRIAN,
        STOP_REASON_OBSTACLE,
        STOP_REASON_PREPARKING,
        STOP_REASON_SIGNAL,
        STOP_REASON_STOP_SIGN,
        STOP_REASON_YIELD_SIGN,
        STOP_REASON_CLEAR_ZONE,
        STOP_REASON_CROSSWALK,
        STOP_REASON_CREEPER,
        STOP_REASON_REFERENCE_END,
        STOP_REASON_YELLOW_SIGNAL,
        STOP_REASON_PULL_OVER
    };

    class ObjectDecision
    {
    public:
        ObjectDecisionType decision_type = unknow;
        StopReasonCode reason_code;
        double distance_s = 0;
        double distance_l = 0;
        GPSPoint stop_point;
    };

    class MainStop
    {
    public:
        StopReasonCode reason_code;
        std::string reason;
        // When stopped, the front center of vehicle should be at this point.
        GPSPoint stop_point;
    };

    enum MainChangeLaneType
    {
        LEFT,
        RIGHT
    };
    class MainChangeLane
    {
    public:
        MainChangeLaneType type;
        MainStop default_lane_stop;
        MainStop target_lane_stop;
    };

    class MainMissionComplete
    {
    public:
        // arrived at routing destination
        // When stopped, the front center of vehicle should be at this point.
        GPSPoint stop_point;
    };

    enum DecisionType
    {
        STOP,
        CHANGE_LANE,
        MISSION_CRUISE,
        MISSION_COMPLETE
    };
    class MainDecision
    {
    public:
        DecisionType main_decision_type;
        MainStop stop;
        MainChangeLane change_lane;
        MainMissionComplete mission_complete;
    };

    class DecisionResult
    {
    public:
        MainDecision main_decision;
        std::vector<ObjectDecision> object_decision;
    };

    struct PathOverlap
    {
        PathOverlap() = default;
        PathOverlap(int object_id, const double start_s, const double end_s)
            : object_id(std::move(object_id)), start_s(start_s), end_s(end_s) {}

        int object_id;
        double start_s = 0.0;
        double end_s = 0.0;

        // std::string DebugString() const;
    };

} // namespace PlannerHNS
