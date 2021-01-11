
/// \file PlanningHelpers.cpp
/// \brief Helper functions for planning algorithms
/// \author Hatem Darweesh
/// \date Jun 16, 2016

#include "op_planner/PlanningHelpers.h"
#include "op_planner/MatrixOperations.h"
#include <string>
#include <float.h>

using namespace UtilityHNS;
using namespace std;

namespace PlannerHNS
{
        int PlanningHelpers::MatchIndex(const std::vector<PlannerHNS::WayPoint> &reference_line,
                       const double x, const double y)
        {

            auto func_distance_square = [](const PlannerHNS::WayPoint &point, const double x,
                                           const double y) {
                double dx = point.pos.x - x;
                double dy = point.pos.y - y;
                return dx * dx + dy * dy;
            };

            double distance_min = func_distance_square(reference_line.front(), x, y);
            std::size_t index_min = 0;

            for (std::size_t i = 1; i < reference_line.size(); ++i)
            {
                double distance_temp = func_distance_square(reference_line[i], x, y);
                if (distance_temp < distance_min)
                {
                    distance_min = distance_temp;
                    index_min = i;
                }
            }

            return index_min;
        }

    void PlanningHelpers::ConvertFrameclockwise(
        const double yaw, const double x0, const double y0, double &xWorld, double &yWrold)
    {
        double tempx = cos(yaw) * x0 + sin(yaw) * y0;
        double tempy = -sin(yaw) * x0 + cos(yaw) * y0;
        //输出
        xWorld = tempx;
        yWrold = tempy;
    }

    // default const bool bDirectionBased = true
    WayPoint *PlanningHelpers::GetClosestWaypointFromMap(const WayPoint &pos,
                                                         RoadNetwork &map,
                                                         const bool bDirectionBased)
    {
        double distance_to_nearest_lane = 1;
        Lane *pLane = 0;

        while (distance_to_nearest_lane < 100 && pLane == 0)
        {
            pLane =
                GetClosestLaneFromMap(pos, map, distance_to_nearest_lane, bDirectionBased);
            distance_to_nearest_lane += 1;
        }

        if (!pLane)
            return nullptr;

        int closest_index =
            PlanningHelpers::GetClosestNextPointIndexFast(pLane->points, pos);
        return &pLane->points.at(closest_index);
    }

    Lane *PlanningHelpers::GetClosestLaneFromMap(
        const WayPoint &pos, RoadNetwork &map, const double &distance, const bool bDirectionBased)
    {
        std::vector<std::pair<double, Lane *>> laneLinksList;
        double d = 0;
        double min_d = DBL_MAX;
        // 求出每一个路网中的lane与pos的最近距离
        for (unsigned int j = 0; j < map.roadSegments.size(); j++)
        {
            //lane中
            for (unsigned int k = 0; k < map.roadSegments.at(j).Lanes.size(); k++)
            {
                //Lane* pLane = &pEdge->lanes.at(k);
                d = 0;
                min_d = DBL_MAX;
                // point
                for (unsigned int pindex = 0; pindex < map.roadSegments.at(j).Lanes.at(k).points.size(); pindex++)
                {

                    d =
                        map.roadSegments.at(j).Lanes.at(k).points.at(pindex).pos.DistanceTo(pos.pos);
                    if (d < min_d)
                        min_d = d;
                }

                if (min_d < distance)
                    laneLinksList.push_back(make_pair(min_d, &map.roadSegments.at(j).Lanes.at(k)));
            }
        }

        if (laneLinksList.size() == 0)
            return nullptr;

        min_d = DBL_MAX;
        Lane *closest_lane = 0;
        for (unsigned int i = 0; i < laneLinksList.size(); i++)
        {
            RelativeInfo info;
            PlanningHelpers::GetRelativeInfo(laneLinksList.at(i).second->points, pos, info);

            if (info.perp_distance == 0 && laneLinksList.at(i).first != 0)
                continue;

            if (bDirectionBased && fabs(info.perp_distance) < min_d && fabs(info.angle_diff) < 45)
            {
                min_d = fabs(info.perp_distance);
                closest_lane = laneLinksList.at(i).second;
            }
            else if (!bDirectionBased && fabs(info.perp_distance) < min_d)
            {
                min_d = fabs(info.perp_distance);
                closest_lane = laneLinksList.at(i).second;
            }
        }

        return closest_lane;
    }

    double PlanningHelpers::slerp(const double a0, const double t0,
                                  const double a1, const double t1,
                                  const double t)
    {
        if (std::abs(t1 - t0) <= kMathEpsilon)
        {
            ADEBUG << "input time difference is too small";
            return UtilityHNS::UtilityH::FixNegativeAngle(a0);
        }
        const double a0_n = UtilityHNS::UtilityH::FixNegativeAngle(a0);
        const double a1_n = UtilityHNS::UtilityH::FixNegativeAngle(a1);
        double d = a1_n - a0_n;
        if (d > M_PI)
        {
            d = d - 2 * M_PI;
        }
        else if (d < -M_PI)
        {
            d = d + 2 * M_PI;
        }

        const double r = (t - t0) / (t1 - t0);
        const double a = a0_n + d * r;
        return UtilityHNS::UtilityH::FixNegativeAngle(a);
    }

    std::pair<double, double> PlanningHelpers::GetPathFrenetCoordinate(
        const std::vector<WayPoint> &reference_line, const GPSPoint contourPoint)
    {
        // 计算车辆当前位置与全局路径参考线的相对信息
        auto matched_path_point =
            MatchToPath(reference_line, contourPoint.x, contourPoint.y);
        double rtheta = matched_path_point.pos.a;
        double rx = matched_path_point.pos.x;
        double ry = matched_path_point.pos.y;
        double delta_x = contourPoint.x - rx;
        double delta_y = contourPoint.y - ry;
        double side = std::cos(rtheta) * delta_y - std::sin(rtheta) * delta_x;
        std::pair<double, double> relative_coordinate;
        // set s
        relative_coordinate.first = matched_path_point.s();
        //set l
        relative_coordinate.second =
            std::copysign(std::hypot(delta_x, delta_y), side);
        return relative_coordinate;
    }

    WayPoint PlanningHelpers::InterpolateUsingLinearApproximation(const WayPoint &p0,
                                                                  const WayPoint &p1,
                                                                  const double s)
    {
        double s0 = p0.s();
        double s1 = p1.s();
        WayPoint path_point;
        double weight = (s - s0) / (s1 - s0);
        double x = (1 - weight) * p0.pos.x + weight * p1.pos.x;
        double y = (1 - weight) * p0.pos.y + weight * p1.pos.y;
        double theta = slerp(p0.pos.a, p0.s(), p1.pos.a, p1.s(), s);
        double kappa = (1 - weight) * p0.kappa() + weight * p1.kappa();
        double dkappa = (1 - weight) * p0.dkappa() + weight * p1.dkappa();
        path_point = p1;
        path_point.pos.x = (x);
        path_point.pos.y = (y);
        path_point.pos.a = (theta);
        path_point.set_kappa(kappa);
        path_point.set_dkappa(dkappa);
        path_point.set_s(s);
        return path_point;
    }

    WayPoint PlanningHelpers::MatchToPath(const std::vector<WayPoint> &reference_line,
                                          const double x, const double y)
    {

        auto func_distance_square = [](const WayPoint &point, const double x,
                                       const double y) {
            double dx = point.pos.x - x;
            double dy = point.pos.y - y;
            return dx * dx + dy * dy;
        };

        double distance_min = func_distance_square(reference_line.front(), x, y);
        std::size_t index_min = 0;

        for (std::size_t i = 1; i < reference_line.size(); ++i)
        {
            double distance_temp = func_distance_square(reference_line[i], x, y);
            if (distance_temp < distance_min)
            {
                distance_min = distance_temp;
                index_min = i;
            }
        }

        std::size_t index_start = (index_min == 0) ? index_min : index_min - 1;
        std::size_t index_end =
            (index_min + 1 == reference_line.size()) ? index_min : index_min + 1;

        if (index_start == index_end)
        {
            return reference_line[index_start];
        }

        return FindProjectionPoint(reference_line[index_start],
                                   reference_line[index_end], x, y);
    }

    WayPoint PlanningHelpers::FindProjectionPoint(const WayPoint &p0,
                                                  const WayPoint &p1, const double x,
                                                  const double y)
    {
        double v0x = x - p0.pos.x;
        double v0y = y - p0.pos.y;

        double v1x = p1.pos.x - p0.pos.x;
        double v1y = p1.pos.y - p0.pos.y;

        double v1_norm = std::sqrt(v1x * v1x + v1y * v1y);
        double dot = v0x * v1x + v0y * v1y;

        double delta_s = dot / v1_norm;
        return InterpolateUsingLinearApproximation(p0, p1, p0.s() + delta_s);
    }

    WayPoint PlanningHelpers::MatchToPath(const std::vector<WayPoint> &reference_line,
                                          const double s)
    {
        auto comp = [](const WayPoint &point, const double s) {
            return point.s() < s;
        };

        auto it_lower =
            std::lower_bound(reference_line.begin(), reference_line.end(), s, comp);
        if (it_lower == reference_line.begin())
        {
            return reference_line.front();
        }
        else if (it_lower == reference_line.end())
        {
            return reference_line.back();
        }

        return InterpolateUsingLinearApproximation(*(it_lower - 1), *it_lower, s);
    }

    void PlanningHelpers::TraversePathTreeBackwards(
        WayPoint *pHead, WayPoint *pStartWP, std::vector<WayPoint> &localPath,
        std::vector<std::size_t> &lane_change_indexs,
        std::vector<std::vector<WayPoint>> &allLocalPaths)
    {

        if (pHead != nullptr && pHead->id != pStartWP->id)
        {
            if (pHead->pBacks.size() > 0)
            {
                allLocalPaths.emplace_back(localPath);
                TraversePathTreeBackwards(
                    GetMinCostCell(pHead->pBacks), pStartWP, localPath, lane_change_indexs, allLocalPaths);
                localPath.push_back(*pHead);
            }
            else if (pHead->pLeft && pHead->cost > 0)
            {
                cout << "Global Lane Change  Right " << endl;
                TraversePathTreeBackwards(
                    pHead->pLeft, pStartWP, localPath, lane_change_indexs, allLocalPaths);
                localPath.push_back(*pHead);
                lane_change_indexs.push_back(localPath.size() - 1);
            }
            else if (pHead->pRight && pHead->cost > 0)
            {
                cout << "Global Lane Change  Left " << endl;
                TraversePathTreeBackwards(
                    pHead->pRight, pStartWP, localPath, lane_change_indexs, allLocalPaths);
                localPath.push_back(*pHead);
                lane_change_indexs.push_back(localPath.size() - 1);
            }
        }
        else
        {
            assert(pHead);
        }
    }

    WayPoint *PlanningHelpers::GetMinCostCell(const vector<WayPoint *> &cells)
    {
        if (cells.size() == 1)
        {
            return cells.at(0);
        }

        WayPoint *pC = cells.at(0); //cost is distance
        for (unsigned int i = 1; i < cells.size(); i++)
        {
            if (cells.at(i)->cost < pC->cost)
                pC = cells.at(i);
        }

        return pC;
    }

    // 获取路点与轨迹间的相关信息
    bool PlanningHelpers::GetRelativeInfo(const std::vector<WayPoint> &trajectory,
                                          const WayPoint &p,
                                          RelativeInfo &info,
                                          const int &prevIndex)
    {
        if (trajectory.size() < 2)
            return false;

        WayPoint p0, p1;
        if (trajectory.size() == 2)
        {
            p0 = trajectory.at(0);
            p1 = WayPoint((trajectory.at(0).pos.x + trajectory.at(1).pos.x) / 2.0,
                          (trajectory.at(0).pos.y + trajectory.at(1).pos.y) / 2.0,
                          (trajectory.at(0).pos.z + trajectory.at(1).pos.z) / 2.0, trajectory.at(0).pos.a);
            info.iFront = 1;
            info.iBack = 0;
        }
        else
        {
            // 道路最近索引点
            // if (prevIndex < trajectory.size() - 1)
            // {
            //     info.iFront = GetClosestNextPointIndexFast(trajectory, p, prevIndex);
            // }
            // else
            // {
            //     std::cout << "prevIndex out of current range!!!!!!!" << std::endl;
            //     info.iFront = 0;
            // }
            // info.iFront = GetClosestNextPointIndexFast(trajectory, p, prevIndex);
            info.iFront = GetClosestNextPointIndexDirectionFast(trajectory, p, prevIndex);

            if (info.iFront > 0)
                info.iBack = info.iFront - 1;
            else
                info.iBack = 0;

            if (info.iFront == 0)
            {
                p0 = trajectory.at(info.iFront);
                p1 = trajectory.at(info.iFront + 1);
            }
            else if (info.iFront > 0 && info.iFront < trajectory.size() - 1)
            {
                p0 = trajectory.at(info.iFront - 1);
                p1 = trajectory.at(info.iFront);
            }
            else
            {
                p0 = trajectory.at(info.iFront - 1);
                p1 = WayPoint((p0.pos.x + trajectory.at(info.iFront).pos.x) / 2.0,
                              (p0.pos.y + trajectory.at(info.iFront).pos.y) / 2.0,
                              (p0.pos.z + trajectory.at(info.iFront).pos.z) / 2.0,
                              p0.pos.a);
            }
        }
        // 以车辆当前位置为原点建立车体坐标系，并将p1固定在x轴上
        WayPoint prevWP = p0;

        Mat3 rotationMat(-p1.pos.a);
        Mat3 translationMat(-p.pos.x, -p.pos.y);
        Mat3 invRotationMat(p1.pos.a);
        Mat3 invTranslationMat(p.pos.x, p.pos.y);

        p0.pos = translationMat * p0.pos;
        p0.pos = rotationMat * p0.pos;
        // P1在
        p1.pos = translationMat * p1.pos;
        p1.pos = rotationMat * p1.pos;

        double m = (p1.pos.y - p0.pos.y) / (p1.pos.x - p0.pos.x);
        info.perp_distance = p1.pos.y - m * p1.pos.x; // solve for x = 0

        if (std::isnan(info.perp_distance) || std::isinf(info.perp_distance))
            info.perp_distance = 0;

        info.to_front_distance = fabs(p1.pos.x); // distance on the x axes

        info.perp_point = p1;
        info.perp_point.pos.x = 0;                  // on the same y axis of the car
        info.perp_point.pos.y = info.perp_distance; //perp distance between the car and the trajectory

        info.perp_point.pos = invRotationMat * info.perp_point.pos;
        info.perp_point.pos = invTranslationMat * info.perp_point.pos;

        info.from_back_distance = hypot(info.perp_point.pos.y - prevWP.pos.y,
                                        info.perp_point.pos.x - prevWP.pos.x);

        info.angle_diff = UtilityH::AngleBetweenTwoAnglesPositive(p1.pos.a, p.pos.a) * RAD2DEG;

        return true;
    }

    bool PlanningHelpers::GetRelativeInfo_forwardback(const std::vector<WayPoint> &trajectory, const WayPoint &p, RelativeInfo &info, const int &prevIndex, const int &turnIndex)
	{
		if (trajectory.size() < 2)
			return false;

		if(prevIndex >= turnIndex)
		{
			std::cout << "error at finding clostest index by CD!!" << std::endl;
			return false;
		}

		WayPoint p0, p1;
		if (trajectory.size() == 2)
		{
			p0 = trajectory.at(0);
			p1 = WayPoint((trajectory.at(0).pos.x + trajectory.at(1).pos.x) / 2.0,
						  (trajectory.at(0).pos.y + trajectory.at(1).pos.y) / 2.0,
						  (trajectory.at(0).pos.z + trajectory.at(1).pos.z) / 2.0, trajectory.at(0).pos.a);
			info.iFront = 1;
			info.iBack = 0;
		}
		else
		{
			// 道路最近索引点
			info.iFront = GetClosestNextPointIndexDirectionFast_forwardback(trajectory, p, prevIndex, turnIndex);


			if (info.iFront > 0)
				info.iBack = info.iFront - 1;
			else
				info.iBack = 0;

			if (info.iFront == 0)
			{
				p0 = trajectory.at(info.iFront);
				p1 = trajectory.at(info.iFront + 1);
			}
			else if (info.iFront > 0 && info.iFront < trajectory.size() - 1)
			{
				p0 = trajectory.at(info.iFront - 1);
				p1 = trajectory.at(info.iFront);
			}
			else
			{
				p0 = trajectory.at(info.iFront - 1);
				p1 = WayPoint((p0.pos.x + trajectory.at(info.iFront).pos.x) / 2.0,
							  (p0.pos.y + trajectory.at(info.iFront).pos.y) / 2.0,
							  (p0.pos.z + trajectory.at(info.iFront).pos.z) / 2.0,
							  p0.pos.a);
			}
		}
		// 以车辆当前位置为原点建立车体坐标系，并将p1固定在x轴上
		WayPoint prevWP = p0;

		Mat3 rotationMat(-p1.pos.a);
		Mat3 translationMat(-p.pos.x, -p.pos.y);
		Mat3 invRotationMat(p1.pos.a);
		Mat3 invTranslationMat(p.pos.x, p.pos.y);

		p0.pos = translationMat * p0.pos;
		p0.pos = rotationMat * p0.pos;
		// P1在
		p1.pos = translationMat * p1.pos;
		p1.pos = rotationMat * p1.pos;

		double m = (p1.pos.y - p0.pos.y) / (p1.pos.x - p0.pos.x);
		info.perp_distance = p1.pos.y - m * p1.pos.x; // solve for x = 0

		if (std::isnan(info.perp_distance) || std::isinf(info.perp_distance))
			info.perp_distance = 0;

		info.to_front_distance = fabs(p1.pos.x); // distance on the x axes

		info.perp_point = p1;
		info.perp_point.pos.x = 0;					// on the same y axis of the car
		info.perp_point.pos.y = info.perp_distance; //perp distance between the car and the trajectory

		info.perp_point.pos = invRotationMat * info.perp_point.pos;
		info.perp_point.pos = invTranslationMat * info.perp_point.pos;

		info.from_back_distance = hypot(info.perp_point.pos.y - prevWP.pos.y,
										info.perp_point.pos.x - prevWP.pos.x);

		info.angle_diff = UtilityH::AngleBetweenTwoAnglesPositive(p1.pos.a, p.pos.a) * RAD2DEG;

		return true;
	}


    int PlanningHelpers::GetClosestNextPointIndexDirectionFast_forwardback(const std::vector<WayPoint> &trajectory, const WayPoint &p, const int &prevIndex, const int &turnIndex)
	{


		if(prevIndex < 0) return 0;

        // 从上次索引点开始---turnIndex存储的是转折点的后点，所以使用小于而非小于等于
		if(prevIndex >= turnIndex)
		{
			std::cout << "error at finding clostest index by CD!!" << std::endl;
			return prevIndex;
		}

		int size = (int)trajectory.size();

		if (size < 2 || turnIndex < 2)
			return 0;

		double d = 0, minD = DBL_MAX;
		int min_index = prevIndex;
		// 从上次索引点开始---turnIndex存储的是转折点的后点，所以使用小于而非小于等于
		for (unsigned int i = prevIndex; i < turnIndex; i++)
		{
			d = distance2pointsSqr(trajectory[i].pos, p.pos);
			double angle_diff = UtilityH::AngleBetweenTwoAnglesPositive(trajectory[i].pos.a, p.pos.a) * RAD2DEG;

			if (d < minD && angle_diff < 45)
			{
				min_index = i;
				minD = d;
			}
		}
        // 从上次索引点开始---turnIndex存储的是转折点的后点，所以使用小于而非小于等于
		if (min_index < turnIndex - 1)
		{
			GPSPoint curr, next;
			curr = trajectory.at(min_index).pos;
			next = trajectory.at(min_index + 1).pos;
			GPSPoint v_1(p.pos.x - curr.x, p.pos.y - curr.y, 0, 0);
			double norm1 = pointNorm(v_1);
			GPSPoint v_2(next.x - curr.x, next.y - curr.y, 0, 0);
			double norm2 = pointNorm(v_2);
			double dot_pro = v_1.x * v_2.x + v_1.y * v_2.y;
			double a = UtilityH::FixNegativeAngle(acos(dot_pro / (norm1 * norm2)));
			if (a <= M_PI_2)
				min_index = min_index + 1;
		}

		return min_index;
	}



    int PlanningHelpers::GetClosestNextPointIndexFast(const vector<WayPoint> &trajectory,
                                                      const WayPoint &p,
                                                      const int &prevIndex)
    {
        int size = (int)trajectory.size();

        if (size < 2 || prevIndex < 0)
            return 0;

        double d = 0, minD = DBL_MAX;
        int min_index = prevIndex;
        int iStart = prevIndex;
        int iEnd = size - 1;
        //  double hypot(double x, double y);
        double resolution = hypot(trajectory[1].pos.y - trajectory[0].pos.y,
                                  trajectory[1].pos.x - trajectory[0].pos.x);

        //divide every 5 meters
        int skip_factor = 5;
        if (resolution > skip_factor)
            resolution = skip_factor;
        // skip number
        int skip = 1;
        if (resolution > 0)
            skip = skip_factor / resolution;

        for (int i = 0; i < size; i += skip)
        {
            if ((int)(i + skip / 2) < size)
                d = (distance2pointsSqr(trajectory[i].pos, p.pos) +
                     distance2pointsSqr(trajectory[(int)(i + skip / 2)].pos, p.pos)) /
                    2.0;
            else
                d = distance2pointsSqr(trajectory[i].pos, p.pos);
            if (d < minD)
            {
                iStart = i - skip;
                iEnd = i + skip;
                minD = d;
                min_index = i;
            }
        }

        if ((size - skip / 2 - 1) > 0)
            d = (distance2pointsSqr(trajectory[size - 1].pos, p.pos) +
                 distance2pointsSqr(trajectory[size - (int)(skip / 2) - 1].pos, p.pos)) /
                2.0;
        else
            d = distance2pointsSqr(trajectory[size - 1].pos, p.pos);

        if (d < minD)
        {
            iStart = size - skip;
            iEnd = size + skip;
            minD = d;
            min_index = size - 1;
        }

        if (iStart < 0)
            iStart = 0;
        if (iEnd >= size)
            iEnd = size - 1;

        for (int i = iStart; i < iEnd; i++)
        {
            d = distance2pointsSqr(trajectory[i].pos, p.pos);
            if (d < minD)
            {
                min_index = i;
                minD = d;
            }
        }

        if (min_index < size - 1)
        {
            GPSPoint curr, next;
            curr = trajectory[min_index].pos;
            next = trajectory[min_index + 1].pos;
            GPSPoint v_1(p.pos.x - curr.x, p.pos.y - curr.y, 0, 0);
            double norm1 = pointNorm(v_1);
            GPSPoint v_2(next.x - curr.x, next.y - curr.y, 0, 0);
            double norm2 = pointNorm(v_2);
            double dot_pro = v_1.x * v_2.x + v_1.y * v_2.y;
            double a = UtilityH::FixNegativeAngle(acos(dot_pro / (norm1 * norm2)));
            if (a <= M_PI_2)
                min_index = min_index + 1;
        }

        return min_index;
    }

    int PlanningHelpers::GetClosestNextPointIndexDirectionFast(const vector<WayPoint> &trajectory,
                                                               const WayPoint &p,
                                                               const int &prevIndex)
    {
        int size = (int)trajectory.size();

        if (size < 2 || prevIndex < 0)
            return 0;

        double d = 0, minD = DBL_MAX;
        int min_index = prevIndex;
        // 从上次索引点开始
        for (unsigned int i = prevIndex; i < size; i++)
        {
            d = distance2pointsSqr(trajectory[i].pos, p.pos);
            double angle_diff = UtilityH::AngleBetweenTwoAnglesPositive(trajectory[i].pos.a, p.pos.a) * RAD2DEG;

            if (d < minD && angle_diff < 45)
            {
                min_index = i;
                minD = d;
            }
        }

        if (min_index < (int)trajectory.size() - 2)
        {
            GPSPoint curr, next;
            curr = trajectory.at(min_index).pos;
            next = trajectory.at(min_index + 1).pos;
            GPSPoint v_1(p.pos.x - curr.x, p.pos.y - curr.y, 0, 0);
            double norm1 = pointNorm(v_1);
            GPSPoint v_2(next.x - curr.x, next.y - curr.y, 0, 0);
            double norm2 = pointNorm(v_2);
            double dot_pro = v_1.x * v_2.x + v_1.y * v_2.y;
            double a = UtilityH::FixNegativeAngle(acos(dot_pro / (norm1 * norm2)));
            if (a <= M_PI_2)
                min_index = min_index + 1;
        }

        return min_index;
    }

    void PlanningHelpers::FixPathDensityAndVelocity(vector<WayPoint> &path,
                                                    const double &distanceDensity,
                                                    const double &velocity)
    {
        if (path.size() == 0 || distanceDensity == 0)
            return;

        double d = 0, a = 0;
        double margin = distanceDensity * 0.01;
        double remaining = 0;
        int nPoints = 0;
        vector<WayPoint> fixedPath;
        fixedPath.push_back(path.at(0));
        for (unsigned int si = 0, ei = 1; ei < path.size();)
        {
            d += hypot(path.at(ei).pos.x - path.at(ei - 1).pos.x, path.at(ei).pos.y - path.at(ei - 1).pos.y) + remaining;
            a = atan2(path.at(ei).pos.y - path.at(si).pos.y, path.at(ei).pos.x - path.at(si).pos.x);

            if (d < distanceDensity - margin) // skip
            {
                ei++;
                remaining = 0;
            }
            else if (d > (distanceDensity + margin)) // skip
            {
                WayPoint pm = path.at(si);
                nPoints = d / distanceDensity;
                for (int k = 0; k < nPoints; k++)
                {
                    pm.pos.x = pm.pos.x + distanceDensity * cos(a);
                    pm.pos.y = pm.pos.y + distanceDensity * sin(a);
                    fixedPath.push_back(pm);
                }
                remaining = d - nPoints * distanceDensity;
                si++;
                path.at(si).pos = pm.pos;
                path.at(si).v = velocity;
                d = 0;
                ei++;
            }
            else
            {
                d = 0;
                remaining = 0;
                path.at(ei).v = velocity;
                fixedPath.push_back(path.at(ei));
                ei++;
                si = ei - 1;
            }
        }

        path = fixedPath;
    }

    void PlanningHelpers::FixPathDensity(vector<WayPoint> &path,
                                         const double &distanceDensity)
    {
        if (path.size() == 0 || distanceDensity == 0)
            return;

        double d = 0, a = 0;
        double margin = distanceDensity * 0.01;
        double remaining = 0;
        int nPoints = 0;
        vector<WayPoint> fixedPath;
        fixedPath.push_back(path.at(0));
        for (unsigned int si = 0, ei = 1; ei < path.size();)
        {
            d += hypot(path.at(ei).pos.x - path.at(ei - 1).pos.x, path.at(ei).pos.y - path.at(ei - 1).pos.y) + remaining;
            a = atan2(path.at(ei).pos.y - path.at(si).pos.y, path.at(ei).pos.x - path.at(si).pos.x);

            if (d < distanceDensity - margin) // skip
            {
                ei++;
                remaining = 0;
            }
            else if (d > (distanceDensity + margin)) // skip
            {
                WayPoint pm = path.at(si);
                nPoints = d / distanceDensity;
                for (int k = 0; k < nPoints; k++)
                {
                    pm.pos.x = pm.pos.x + distanceDensity * cos(a);
                    pm.pos.y = pm.pos.y + distanceDensity * sin(a);
                    fixedPath.push_back(pm);
                }
                remaining = d - nPoints * distanceDensity;
                si++;
                path.at(si).pos = pm.pos;
                d = 0;
                ei++;
            }
            else
            {
                d = 0;
                remaining = 0;
                fixedPath.push_back(path.at(ei));
                ei++;
                si = ei - 1;
            }
        }

        path = fixedPath;
    }

    void PlanningHelpers::SmoothPath(vector<WayPoint> &path, double weight_data,
                                     double weight_smooth, double tolerance)
    {

        if (path.size() <= 2)
        {
            //cout << "Can't Smooth Path, Path_in Size=" << path.size() << endl;
            return;
        }

        const vector<WayPoint> &path_in = path;
        vector<WayPoint> smoothPath_out = path_in;

        double change = tolerance;
        double xtemp, ytemp;
        int nIterations = 0;

        int size = path_in.size();

        while (change >= tolerance)
        {
            change = 0.0;
            for (int i = 1; i < size - 1; i++)
            {
                //			if (smoothPath_out[i].pos.a != smoothPath_out[i - 1].pos.a)
                //				continue;

                xtemp = smoothPath_out[i].pos.x;
                ytemp = smoothPath_out[i].pos.y;

                smoothPath_out[i].pos.x += weight_data * (path_in[i].pos.x - smoothPath_out[i].pos.x);
                smoothPath_out[i].pos.y += weight_data * (path_in[i].pos.y - smoothPath_out[i].pos.y);

                smoothPath_out[i].pos.x += weight_smooth * (smoothPath_out[i - 1].pos.x + smoothPath_out[i + 1].pos.x - (2.0 * smoothPath_out[i].pos.x));
                smoothPath_out[i].pos.y += weight_smooth * (smoothPath_out[i - 1].pos.y + smoothPath_out[i + 1].pos.y - (2.0 * smoothPath_out[i].pos.y));

                change += fabs(xtemp - smoothPath_out[i].pos.x);
                change += fabs(ytemp - smoothPath_out[i].pos.y);
            }
            nIterations++;
        }

        path = smoothPath_out;
    }

    void PlanningHelpers::FixAngleOnly(std::vector<WayPoint> &path)
    {
        if (path.size() <= 2)
            return;

        path[0].pos.a = UtilityH::FixNegativeAngle(atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x));

        for (int j = 1; j < path.size() - 1; j++)
            path[j].pos.a = UtilityH::FixNegativeAngle(atan2(path[j + 1].pos.y - path[j].pos.y, path[j + 1].pos.x - path[j].pos.x));

        int j = (int)path.size() - 1;

        path[j].pos.a = path[j - 1].pos.a;

        for (int j = 0; j < path.size() - 1; j++)
        {
            if (path.at(j).pos.x == path.at(j + 1).pos.x && path.at(j).pos.y == path.at(j + 1).pos.y)
                path.at(j).pos.a = path.at(j + 1).pos.a;
        }
    }
    // 计算路径角度和代价（从起点开始累计的路径距离作为代价）
    double PlanningHelpers::CalcAngleAndCost(vector<WayPoint> &path,
                                             const double &lastCost,
                                             const bool &bSmooth)
    {
        if (path.size() < 2)
            return 0;
        if (path.size() == 2)
        {
            path[0].pos.a = UtilityH::FixNegativeAngle(atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x));
            path[0].cost = lastCost;
            path[0].set_s(lastCost);
            path[1].pos.a = path[0].pos.a;
            path[1].cost = path[0].cost + distance2points(path[0].pos, path[1].pos);
            path[0].set_s(path[1].cost);
            return path[1].cost;
        }

        path[0].pos.a = UtilityH::FixNegativeAngle(atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x));
        path[0].cost = lastCost;
        path[0].set_s(lastCost);
        for (int j = 1; j < path.size() - 1; j++)
        {
            path[j].pos.a = UtilityH::FixNegativeAngle(atan2(path[j + 1].pos.y - path[j].pos.y, path[j + 1].pos.x - path[j].pos.x));
            path[j].cost = path[j - 1].cost + distance2points(path[j - 1].pos, path[j].pos);
            path[j].set_s(path[j].cost);
        }

        int j = (int)path.size() - 1;

        path[j].pos.a = path[j - 1].pos.a;
        path[j].cost = path[j - 1].cost + distance2points(path[j - 1].pos, path[j].pos);
        path[j].set_s(path[j].cost);

        for (int j = 0; j < path.size() - 1; j++)
        {
            if (path.at(j).pos.x == path.at(j + 1).pos.x && path.at(j).pos.y == path.at(j + 1).pos.y)
                path.at(j).pos.a = path.at(j + 1).pos.a;
        }

        return path[j].cost;
    }

    void PlanningHelpers::DeleteWaypoints(std::vector<WayPoint *> &wps)
    {
        for (unsigned int i = 0; i < wps.size(); i++)
        {
            if (wps.at(i))
            {
                delete wps.at(i);
                wps.at(i) = 0;
            }
        }
        wps.clear();
    }

    WayPoint *PlanningHelpers::CheckLaneExits(const vector<WayPoint *> &nodes, const Lane *pL)
    {
        if (nodes.size() == 0)
            return nullptr;

        for (unsigned int i = 0; i < nodes.size(); i++)
        {
            if (nodes.at(i)->pLane == pL)
                return nodes.at(i);
        }

        return nullptr;
    }

    WayPoint *PlanningHelpers::CheckNodeExits(const vector<WayPoint *> &nodes, const WayPoint *pL)
    {
        if (nodes.size() == 0)
            return nullptr;

        for (unsigned int i = 0; i < nodes.size(); i++)
        {
            if (nodes.at(i)->laneId == pL->laneId && nodes.at(i)->id == pL->id)
                return nodes.at(i);
        }

        return nullptr;
    }

    void PlanningHelpers::WritePathToFile(const string &fileName, const vector<WayPoint> &path)
    {
        DataRW dataFile;
        ostringstream str_header;
        // str_header << "laneID"
        //            << ","
        //            << "wpID"
        //            << ","
        //            << "x"
        //            << ","
        //            << "y"
        //            << ","
        //            << "a"
        //            << ","
        //            << "global speed"
        //            << ","
        //            << "s"
        //            << ","
        //            << "kappa"
        //            << ","
        //            << "dkappa"
        //            << ",";
        vector<string> dataList;
        for (unsigned int i = 0; i < path.size(); i++)
        {

            // strwp << path.at(i).laneId << "," << path.at(i).id << "," << path.at(i).pos.x
            //       << "," << path.at(i).pos.y << "," << path.at(i).pos.a << "," << path.at(i).v
            //       << "," << path.at(i).s() << "," << path.at(i).kappa() << "," << path.at(i).dkappa()
            //       << ",";
            ostringstream strwp;
            strwp << path.at(i).pos.x
                  << "," << path.at(i).pos.y << "," << path.at(i).pos.z << "," << path.at(i).pos.a << "," << path.at(i).v;
            dataList.push_back(strwp.str());
            // if (path.at(i).pLeft == nullptr)
            // {
            //     ostringstream strwp;
            //     strwp << i << "," << path.at(i).id;
            //     dataList.push_back(strwp.str());
            // }
            // else
            // {
            //     ostringstream strwp;
            //     strwp << i ;
            //     dataList.push_back(strwp.str());
            // }
        }

        dataFile.WriteLogData("", fileName, str_header.str(), dataList);
    }

    // 为每个障碍物在路网中找到所有预测范围内的可能行驶路线
    // input: objects, current position,deceleeration, roadnetword
    void PlanningHelpers::ExtractTrajectoryFromMap(
        DetectedObject &obstacle, RoadNetwork &map)
    {
        // if objects` direction and velocity is determined;
        // only consider dynamic obstacle
        if (obstacle.bDirection && obstacle.bVelocity)
        {
            auto closest_lane_waypoint =
                PlanningHelpers::GetClosestWaypointFromMap(obstacle.center, map, false);

            auto obstacle_to_lane_center_distance =
                obstacle.center.pos.DistanceTo(closest_lane_waypoint->pos);
            // 只考虑离车道中心垂直距离小于半个车道的
            if (obstacle_to_lane_center_distance > 1.6)
            {
                ADEBUG << "ignore dynamic Obstacle which is not in map, id " << obstacle.Id()
                       << " distance: " << obstacle_to_lane_center_distance;
                return;
            }

            double m_PredictionDistance =
                obstacle.Speed() * FLAGS_trajectory_time_length;
            // 所有的路线都是沿地图中的车道的，适用于车辆预测，对于行人等障碍物的处理不是很合理
            if (closest_lane_waypoint->pFronts.empty())
                return;
            obstacle.predTrajectory.clear();
            obstacle.predTrajectory.emplace_back(obstacle.center);
            obstacle.predTrajectory.emplace_back(*closest_lane_waypoint);
            auto pH = closest_lane_waypoint->pFronts.at(0);
            double dist = 0;
            while (dist < m_PredictionDistance)
            {
                if (!pH->pFronts.empty())
                {
                    dist =
                        pH->pFronts.at(0)->pos.DistanceTo(closest_lane_waypoint->pos);
                    obstacle.predTrajectory.emplace_back(*(pH->pFronts.at(0)));
                    pH = pH->pFronts.at(0);
                }
                else
                {
                    break;
                }
            }

            ADEBUG << "Predict Dynamic obstacle trajectory, id " << obstacle.Id() << " size: "
                   << obstacle.predTrajectory.size()
                   << " m_PredictionDistance: " << m_PredictionDistance;

            double total_distance = 0;
            double accum_time = 0;
            obstacle.predTrajectory.at(0).timeCost = 0;
            // using uniform speed model to calculate trajectory time cost
            for (unsigned int i = 1; i < obstacle.predTrajectory.size(); i++)
            {
                total_distance += hypot(
                    obstacle.predTrajectory.at(i).pos.x - obstacle.predTrajectory.at(i - 1).pos.x,
                    obstacle.predTrajectory.at(i).pos.y - obstacle.predTrajectory.at(i - 1).pos.y);
                accum_time = total_distance / obstacle.center.v;
                obstacle.predTrajectory.at(i).timeCost = accum_time;
            }
        }
    }
} /* namespace PlannerHNS */
