
/// \file PlanningHelpers.h
/// \brief Helper functions for planning algorithms
/// \author Hatem Darweesh
/// \date Jun 16, 2016

#ifndef PLANNINGHELPERS_H_
#define PLANNINGHELPERS_H_

#include "RoadNetwork.h"
#include "op_utility/UtilityH.h"
#include "op_utility/DataRW.h"
#include "op_planner/planning_gflags.h"
#include "tinyxml.h"
#include "math.h"
#include <algorithm>
#include <utility>
#include <vector>
#include <cmath>
#include <memory>

namespace PlannerHNS
{

// #define distance2points(from, to) sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2))
// #define distance2pointsSqr(from, to) pow(to.x - from.x, 2) + pow(to.y - from.y, 2)
#define pointNorm(v) sqrt(v.x *v.x + v.y * v.y)

    inline double distance2pointsSqr(const GPSPoint &p1, const GPSPoint &p2)
    {
        return pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2);
    }

    inline double distance2points(const GPSPoint &p1, const GPSPoint &p2)
    {
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    }


    constexpr double kMathEpsilon = 1e-10;
    /**
     * @brief Linear interpolation between two points of type T.
     * @param x0 The coordinate of the first point.
     * @param t0 The interpolation parameter of the first point.
     * @param x1 The coordinate of the second point.
     * @param t1 The interpolation parameter of the second point.
     * @param t The interpolation parameter for interpolation.
     * @param x The coordinate of the interpolated point.
     * @return Interpolated point.
     */
    template <typename T>
    T lerp(const T &x0, const double t0, const T &x1, const double t1,
           const double t)
    {
        if (std::abs(t1 - t0) <= 1.0e-6)
        {
            ADEBUG << "input time difference is too small";
            return x0;
        }
        const double r = (t - t0) / (t1 - t0);
        const T x = x0 + r * (x1 - x0);
        return x;
    }

    class PlanningHelpers
    {
    public:
        PlanningHelpers() = default;
        virtual ~PlanningHelpers() = default;

        static int MatchIndex(const std::vector<PlannerHNS::WayPoint> &reference_line,
                       const double x, const double y);

        static void ConvertFrameclockwise(
            const double yaw, const double x0, const double y0, double &xWorld, double &yWrold);

        static Lane *GetClosestLaneFromMap(
            const WayPoint &pos, RoadNetwork &map, const double &distance, const bool bDirectionBased);
        static WayPoint *GetClosestWaypointFromMap(
            const WayPoint &pos, RoadNetwork &map, const bool bDirectionBased = true);

        static double slerp(const double a0, const double t0, const double a1, const double t1,
                            const double t);

        static WayPoint MatchToPath(const std::vector<WayPoint> &reference_line,
                                    const double x, const double y);

        static WayPoint FindProjectionPoint(const WayPoint &p0,
                                            const WayPoint &p1, const double x,
                                            const double y);

        static std::pair<double, double> GetPathFrenetCoordinate(
            const std::vector<WayPoint> &reference_line, const GPSPoint contourPoint);

        static WayPoint MatchToPath(const std::vector<WayPoint> &reference_line,
                                    const double s);
        static WayPoint InterpolateUsingLinearApproximation(
            const WayPoint &p0, const WayPoint &p1, const double s);

        static void DeleteWaypoints(std::vector<WayPoint *> &wps);
        static bool GetRelativeInfo(const std::vector<WayPoint> &trajectory, const WayPoint &p, RelativeInfo &info, const int &prevIndex = 0);

        static double GetExactDistanceOnTrajectory(const std::vector<WayPoint> &trajectory, const RelativeInfo &p1, const RelativeInfo &p2);

        static int GetClosestNextPointIndexFast(const std::vector<WayPoint> &trajectory, const WayPoint &p, const int &prevIndex = 0);

        static int GetClosestNextPointIndexFastV2(const std::vector<WayPoint> &trajectory, const WayPoint &p, const int &prevIndex = 0);

        static int GetClosestNextPointIndexDirectionFast(const std::vector<WayPoint> &trajectory, const WayPoint &p, const int &prevIndex = 0);
        

        //CD add at 20201112
		static bool GetRelativeInfo_forwardback(const std::vector<WayPoint> &trajectory, const WayPoint &p, RelativeInfo &info, const int &prevIndex, const int &turnIndex);
		static int GetClosestNextPointIndexDirectionFast_forwardback(const std::vector<WayPoint> &trajectory, const WayPoint &p, const int &prevIndex, const int &turnIndex);


        static int GetClosestNextPointIndexDirectionFastV2(const std::vector<WayPoint> &trajectory, const WayPoint &p, const int &prevIndex = 0);

        static void FixPathDensityAndVelocity(std::vector<WayPoint> &path, const double &distanceDensity, const double &velocity);

        static void FixPathDensity(std::vector<WayPoint> &path, const double &distanceDensity);

        static void SmoothPath(std::vector<WayPoint> &path, double weight_data = 0.25, double weight_smooth = 0.25, double tolerance = 0.01);

        static void FixAngleOnly(std::vector<WayPoint> &path);

        static double CalcAngleAndCost(std::vector<WayPoint> &path, const double &lastCost = 0, const bool &bSmooth = true);

        static WayPoint *CheckLaneExits(const std::vector<WayPoint *> &nodes, const Lane *pL);

        static WayPoint *CheckNodeExits(const std::vector<WayPoint *> &nodes, const WayPoint *pL);

        static WayPoint *GetMinCostCell(const std::vector<WayPoint *> &cells);

        static void TraversePathTreeBackwards(
            WayPoint *pHead, WayPoint *pStartWP, std::vector<WayPoint> &localPath,
            std::vector<std::size_t> &lane_change_indexs,
            std::vector<std::vector<WayPoint>> &allLocalPaths);
        static void ExtractTrajectoryFromMap(
            DetectedObject &obstacle, RoadNetwork &map);
        static void WritePathToFile(const std::string &fileName, const std::vector<WayPoint> &path);
    };

} /* namespace PlannerHNS */

#endif /* PLANNINGHELPERS_H_ */
