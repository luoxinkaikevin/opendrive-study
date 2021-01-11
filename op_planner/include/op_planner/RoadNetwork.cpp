#include "op_planner/RoadNetwork.h"

namespace PlannerHNS
{
    void WayPoint::Init()
    {
        enable_left_change=false;
        enable_right_change=false;
        trafficLigntID=-1;
        this->id = 0;
        this->cost = 0;
        this->laneId = -1;
        this->pLane = 0;
        this->pLeft = 0;
        this->pRight = 0;
        this->LeftPointId = 0;
        this->RightPointId = 0;
        this->LeftLnId = 0;
        this->RightLnId = 0;
        this->timeCost = 0;
        this->laneWidth = 0;
        this->stopLineID = -1;
        this->crosswalkID = -1;
        this->task = CRUISE;
    }
    WayPoint::WayPoint()
    {
        pos.x = 0;
        pos.y = 0;
        pos.z = 0;
        pos.a = 0;
        v = 0;
        Init();
    }
    WayPoint::WayPoint(const double &x, const double &y, const double &z, const double &a)
    {
        pos.x = x;
        pos.y = y;
        pos.z = z;
        pos.a = a;
        v = 0;
        Init();
    }

    WayPoint::WayPoint(const double &x, const double &y, const double &z, const double &a, const double &speed)
    {
        pos.x = x;
        pos.y = y;
        pos.z = z;
        pos.a = a;
        v = speed;
        Init();
    }

    WayPoint::WayPoint(const double &x, const double &y, const double &z, const double &a, const double &speed, const int &geartmp)
    {
        pos.x = x;
        pos.y = y;
        pos.z = z;
        pos.a = a;
        v = speed;
        gear = geartmp;
        Init();
        
    }

    DetectedObject::DetectedObject()
    {
        is_virtual_ = false;
        bDirection = false;
        bVelocity = false;
        id = 0;
        w = 0;
        l = 0;
        h = 0;
        predTrajectory.clear();
    }

    WayPoint DetectedObject::GetPointAtTime(const double relative_time) const
    {
        if (predTrajectory.empty())
        {
            return center;
        }
        else
        {
            const auto &points = predTrajectory;
            auto comp = [](const WayPoint p, const double time) {
                return p.timeCost < time;
            };

            auto it_lower =
                std::lower_bound(points.begin(), points.end(), relative_time, comp);

            if (it_lower == points.begin())
            {
                return *points.begin();
            }
            else if (it_lower == points.end())
            {
                return *points.rbegin();
            }
            return *(it_lower);
        }
    }
} // namespace PlannerHNS