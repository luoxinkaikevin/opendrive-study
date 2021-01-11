#pragma once

#include <math.h>
#include <fstream>
#include <functional>
#include "op_planner/OpendriveMapObjects.h"
#include "op_planner/RoadNetwork.h"
#include "op_planner/MatrixOperations.h"
#include "op_planner/box2d.h"
#include "op_planner/RoadElement.h"
#include "op_planner/PlanningHelpers.h"
namespace opendrive
{

    // 前驱道路连接关系
    class FromRoadLink
    {
    public:
        LINK_TYPE link_type_;
        int from_road_id_;
        CONTACT_POINT contact_point_;

        FromRoadLink(TiXmlElement *main_element)
        {
            if (XmlHelpers::getStringAttribute(main_element, "elementType", "").compare("road") == 0)
                link_type_ = ROAD_LINK;
            else
                link_type_ = JUNCTION_LINK;

            from_road_id_ = XmlHelpers::getIntAttribute(main_element, "elementId", 0);

            if (XmlHelpers::getStringAttribute(main_element, "contactPoint", "").compare("start") == 0)
                contact_point_ = START_POINT;
            else if (XmlHelpers::getStringAttribute(main_element, "contactPoint", "").compare("end") == 0)
                contact_point_ = END_POINT;
            else
                contact_point_ = EMPTY_POINT;
        }
    };
    // 后继道路连接关系
    class ToRoadLink
    {
    public:
        LINK_TYPE link_type_;
        int to_road_id_;
        CONTACT_POINT contact_point_;

        ToRoadLink(TiXmlElement *main_element)
        {
            if (XmlHelpers::getStringAttribute(main_element, "elementType", "").compare("road") == 0)
                link_type_ = ROAD_LINK;
            else
                link_type_ = JUNCTION_LINK;

            to_road_id_ = XmlHelpers::getIntAttribute(main_element, "elementId", 0);

            if (XmlHelpers::getStringAttribute(main_element, "contactPoint", "").compare("start") == 0)
                contact_point_ = START_POINT;
            else if (XmlHelpers::getStringAttribute(main_element, "contactPoint", "").compare("end") == 0)
                contact_point_ = END_POINT;
            // junction type
            else
                contact_point_ = EMPTY_POINT;
        }
    };

    class OpenDriveRoad
    {
    public:
        std::string name_;
        // road id
        int id_;
        //连接关系id
        int junction_id_;
        //长度
        double length_;
        // 速度
        double max_speed;
        //是否右行优先
        bool keep_right_;

        std::string road_type_;

        std::vector<FromRoadLink> predecessor_road_;
        std::vector<ToRoadLink> successor_road_;
        std::vector<Geometry> geometries_;
        std::vector<Elevation> elevations_;
        std::vector<RoadSection> sections_;
        std::vector<LaneOffset> laneOffsets_;
        std::vector<Signal> road_signals_;
        std::vector<SignalRef> road_signals_references_;
        std::vector<RoadObject> road_objects_;
        std::vector<RoadObjectRef> road_objects_references_;
        std::vector<Connection> to_roads_;
        std::vector<Connection> from_roads_;

        std::vector<std::pair<std::string, std::vector<CSV_Reader::LINE_DATA>>> *p_country_signal_codes_;
        std::vector<Connection> getFirstSectionConnections(OpenDriveRoad *_p_predecessor_road);
        std::vector<Connection> getLastSectionConnections(OpenDriveRoad *_p_predecessor_road);

        void getRoadLanes(std::vector<PlannerHNS::Lane> &lanes_list, double resolution = 0.5);
        void getTrafficLights(
            std::unordered_map<int, PlannerHNS::TrafficLight> &all_lights);
        void getTrafficSigns(std::vector<PlannerHNS::TrafficSign> &all_signs);
        void getStopLines(std::unordered_map<int, PlannerHNS::StopLine> &all_stop_lines);
        void getCrossWalk(std::unordered_map<int, PlannerHNS::Crossing> &all_crosswalk);
        void getParkSpot(std::unordered_map<int, PlannerHNS::ParkSpot> &all_parkspot);
        void insertUniqueToConnection(const Connection &_connection);
        void insertUniqueFromConnection(const Connection &_connection);

        OpenDriveRoad()
        {
            p_country_signal_codes_ = nullptr;
            id_ = 0;
            junction_id_ = 0;
            length_ = 0;
            keep_right_ = true;
            max_speed = 0;
        }
        //将xdor文档中的road信息全部读取出来
        OpenDriveRoad(TiXmlElement *main_element,
                      std::vector<std::pair<std::string,
                                            std::vector<CSV_Reader::LINE_DATA>>> *country_signal_codes = nullptr,
                      bool keep_right = true);
        // 获取第一个lanesection
        RoadSection *getFirstSection()
        {
            if (sections_.size() == 0)
                return nullptr;

            return &sections_.at(0);
        }
        // 获取最后一个lanesection
        RoadSection *getLastSection()
        {
            if (sections_.size() == 0)
                return nullptr;

            return &sections_.at(sections_.size() - 1);
        }

    private:
        // 根据sOffset获取当前lane的道路几何形状
        Geometry *getMatchingGeometry(const double &sOffset)
        {
            for (unsigned int i = 0; i < geometries_.size(); i++)
            {
                if (sOffset >= geometries_.at(i).s && sOffset < (geometries_.at(i).s + geometries_.at(i).length))
                {
                    return &geometries_.at(i);
                }
            }
            return nullptr;
        }
        // 根据sOffset获取海拔高度
        Elevation *getMatchingElevations(const double &sOffset)
        {
            if (elevations_.size() == 0)
                return nullptr;

            if (elevations_.size() == 1)
                return &elevations_.at(0);

            for (int i = 1; i < elevations_.size(); i++)
            {
                if (sOffset >= elevations_.at(i - 1).s && sOffset < elevations_.at(i).s)
                {
                    return &elevations_.at(i - 1);
                }
            }

            return &elevations_.at(elevations_.size() - 1);
        }
        // 根据sOffset获取laneoffset
        LaneOffset *getMatchingLaneOffset(const double &sOffset)
        {
            if (laneOffsets_.size() == 0)
                return nullptr;

            if (laneOffsets_.size() == 1)
                return &laneOffsets_.at(0);

            for (int i = 1; i < laneOffsets_.size(); i++)
            {
                if (sOffset >= laneOffsets_.at(i - 1).s && sOffset < laneOffsets_.at(i).s)
                {
                    return &laneOffsets_.at(i - 1);
                }
            }

            return &laneOffsets_.at(laneOffsets_.size() - 1);
        }
        // get road laneSection index by arc
        RoadSection *getMatchingSection(const double &sOffset)
        {
            if (sections_.size() == 0)
                return nullptr;

            if (sections_.size() == 1)
                return &sections_.at(0);

            for (unsigned int i = 1; i < sections_.size(); i++)
            {
                if (sOffset >= sections_.at(i - 1).s_ && sOffset < sections_.at(i).s_)
                {
                    return &sections_.at(i - 1);
                }
            }
            return &sections_.at(sections_.size() - 1);
        }

        void insertUniqueFromSectionIds(int from_section_id, const OpenDriveLane *curr_lane, PlannerHNS::Lane &_l);
        void insertUniqueToSectionIds(int to_section_id, const OpenDriveLane *curr_lane, PlannerHNS::Lane &_l);
        void insertUniqueFromRoadIds(int curr_section_id, int curr_lane_id, PlannerHNS::Lane &_l);
        void insertUniqueToRoadIds(int curr_section_id, int curr_lane_id, PlannerHNS::Lane &_l);

        void createAdjecntLanes(std::vector<PlannerHNS::Lane> &lanes_list);
        bool createSingleCenterPoint(double _ds, PlannerHNS::WayPoint &_p);
        void createRoadLanes(std::vector<PlannerHNS::Lane> &lanes_list);
        void createRoadCenterInfo(std::vector<RoadCenterInfo> &points_list, double resolution = 0.5);
        bool createRoadCenterPoint(RoadCenterInfo &inf_point, double _s);
        void insertRoadCenterInfo(std::vector<RoadCenterInfo> &points_list, RoadCenterInfo &inf_point);
        void fixRedundantPointsLanes(PlannerHNS::Lane &_lane);
        void createSectionPoints(const RoadCenterInfo &ref_info, std::vector<PlannerHNS::Lane> &lanes_list,
                                 RoadSection *p_sec, int &wp_id_seq, std::vector<int> &left_lane_ids, std::vector<int> &right_lane_ids);

        // get single lane by id in lane list and refresh it
        PlannerHNS::Lane *getLaneById(const int &_l_id,
                                      std::vector<PlannerHNS::Lane> &_lanes_list)
        {
            for (unsigned int i = 0; i < _lanes_list.size(); i++)
            {
                if (_lanes_list.at(i).id == _l_id)
                    return &_lanes_list.at(i);
            }

            return nullptr;
        }
        // 判断向量_list中是否含有val值
        bool exists(const std::vector<int> &_list, int _val)
        {
            for (unsigned int j = 0; j < _list.size(); j++)
            {
                if (_list.at(j) == _val)
                {
                    return true;
                }
            }

            return false;
        }

        bool getLaneChangeFlag(LINE_MARKING_TYPE mark_type)
        {
            if (mark_type == BROKEN_LINE_MARK)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        OBJECT_TYPE getAutowareMainTypeFromCode(const std::string &country_code, const std::string &type, const std::string &sub_type);
        TRAFFIC_LIGHT_TYPE getAutowareLightTypeFromCode(const std::string &country_code, const std::string &type, const std::string &sub_type);
        ROAD_SIGN_TYPE getAutowareRoadSignTypeFromCode(const std::string &country_code, const std::string &type, const std::string &sub_type);
        ROAD_MARK_TYPE getAutowareRoadMarksTypeFromCode(const std::string &country_code, const std::string &type, const std::string &sub_type);

        OBJECT_TYPE getObjTypeFromText(const std::string &autoware_type);
        TRAFFIC_LIGHT_TYPE getLightTypeFromText(const std::string &autoware_type);
        ROAD_SIGN_TYPE getSignTypeFromText(const std::string &autoware_type);
        ROAD_MARK_TYPE getMarkTypeFromText(const std::string &autoware_type);
    };

} // namespace opendrive
