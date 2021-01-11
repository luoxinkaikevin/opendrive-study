/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef OPENDRIVEMAPLOADER_H_
#define OPENDRIVEMAPLOADER_H_

// #include "op_planner/MappingHelpers.h"
#include "op_planner/PlanningHelpers.h"
#include "op_planner/RoadNetwork.h"
#include "op_planner/OpendriveMapRoad.h"
// dirent.h是用于目录操作的头文件，linux 默认在/usr/include目录下（会自动包含其他文件）
#include "dirent.h"
#include "op_utility/DataRW.h"
#include <fstream>
#include <cmath>
// #include <ros/ros.h>

namespace opendrive
{
    class OpenDriveLoader
    {

    public:
        OpenDriveLoader();
        ~OpenDriveLoader();
        // 获取目录中的文件名
        void getFileNameInFolder(const std::string &path, std::vector<std::string> &out_list);
        //加载country代码
        void loadCountryCods(const std::string &codes_csv_folder);
        //加载opendrive地图
        void loadOpenDRIVE(const std::string &xodr_file, const std::string &codes_folder, PlannerHNS::RoadNetwork &map, double resolution = 0.5, bool keep_right = true);
        // 获取map中的车道
        void getMapLanes(std::vector<PlannerHNS::Lane> &all_lanes, double resolution = 0.5);
        // 获取map中的交通信号
        void getTrafficLights(std::unordered_map<int, PlannerHNS::TrafficLight> &all_lights);
        // 获取map中的停止线
        void getStopLines(std::unordered_map<int, PlannerHNS::StopLine> &all_stop_lines);

        void getCrossings(std::unordered_map<int, PlannerHNS::Crossing> &all_crossings);
        void getParkSpot(
            std::unordered_map<int, PlannerHNS::ParkSpot> &all_parkings);
        // 连接道路关系
        void connectRoads();

    private:
        // 是否靠右行驶标志位
        bool keep_right_;
        // 道路向量
        std::vector<OpenDriveRoad> roads_list_;
        // 连接向量
        std::vector<Junction> junctions_list_;
        // country向量
        std::vector<std::pair<std::string, std::vector<CSV_Reader::LINE_DATA>>> country_signal_codes_;
        //根据道路id获取下一道路
        std::vector<OpenDriveRoad *> getRoadsBySuccId(int _id);
        //根据道路id获取前一道路
        std::vector<OpenDriveRoad *> getRoadsByPredId(int _id);
        // 根据道路id获取当前道路
        OpenDriveRoad *getRoadById(int _id);
        //根据连接id获取当前连接
        Junction *getJunctionById(int _id);
        //连接道路点
        void linkWayPoints(PlannerHNS::RoadNetwork &map);
        // 输出道路连接关系
        void coutRoadLinks(void);
        // 将解析出来的有关map的数据存储起来
        void saveMapData(PlannerHNS::RoadNetwork &map);
        // 寻找相邻的lane
        void FindAdjacentLanes(PlannerHNS::RoadNetwork &map);
        // 连接停止线
        void LinkStopLines(PlannerHNS::RoadNetwork &map);
        // 连接红绿灯
        void LinkTrafficLights(PlannerHNS::RoadNetwork &map);
        // 连接人行道
        void LinkCrossWalks(PlannerHNS::RoadNetwork &map);
        // 暂时无用
        void findAsterArea(PlannerHNS::RoadNetwork &map);
        // 连接停车点
        void LinkParkSpot(PlannerHNS::RoadNetwork &map);
        // ？
        void LinkMissingBranchingWayPointsV2(PlannerHNS::RoadNetwork &map);
    
    public:
        void CopyLaneSign(int &i, bool &j)
        {
            if (i > 0)
            {
                j = true;
            }
            else
            {
                j = false;
            }
        }

        double MatchDistance(const std::vector<PlannerHNS::WayPoint> &reference_line,
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

            return std::pow(distance_min,0.5);
        }


        int MatchIndex(const std::vector<PlannerHNS::WayPoint> &reference_line,
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

    };

} // namespace opendrive

#endif // OPENDRIVE2AUTOWARE_CONVERTER
