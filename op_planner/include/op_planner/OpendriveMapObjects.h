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

#ifndef OPENDRIVEMAPOBJECTS_H_
#define OPENDRIVEMAPOBJECTS_H_

#include "op_planner/RoadNetwork.h"
#include "op_planner/Spiral.h"
#include "op_planner/xml_helpers.h"

namespace opendrive
{
    //enum LINK_TYPE {PREDECESSOR_LINK, SUCCESSOR_LINK, EMPTY_LINK };
    // 道路连接关系
    enum LINK_TYPE
    {
        ROAD_LINK,
        JUNCTION_LINK
    };
    // 道路连接点
    enum CONTACT_POINT
    {
        START_POINT,
        END_POINT,
        EMPTY_POINT
    };
    // 道路连接几何类型
    enum GEOMETRY_TYPE
    {
        LINE_GEOMETRY,
        SPIRAL_GEOMETRY,
        ARC_GEOMETRY,
        POLY3_GEOMETRY,
        PARAM_POLY3_GEOMETRY,
        UNKNOWN_GEOMETRY
    };
    //道路海拔
    enum ELEVATION_TYPE
    {
        ELEVATION_PROFILE,
        LATERAL_PROFILE
    };
    // 车道方向
    enum LANE_DIRECTION
    {
        LEFT_LANE,
        RIGHT_LANE,
        CENTER_LANE
    };
    // 道路标记:
    enum LINE_MARKING_TYPE
    {
        BROKEN_LINE_MARK,
        SOLID_LINE_MARK,
        NONE_LINE_MARK,
        UNKNOWN_LINE_MARK
    };
    // 对象:交通灯\路灯\路标
    enum OBJECT_TYPE
    {
        TRAFFIC_LIGHT,
        ROAD_SIGN,
        ROAD_MARK,
        UNKNOWN_OBJECT
    };
    // 交通灯类型:垂直\水平\行人\未知
    enum TRAFFIC_LIGHT_TYPE
    {
        VERTICAL_DEFAULT_LIGHT,
        HORIZONTAL_DEFAULTLIGHT,
        PEDESTRIAN_DEFAULT_LIGHT,
        UNKNOWN_LIGHT
    };
    // 路牌:限速\停止\不能停车\未知
    enum ROAD_SIGN_TYPE
    {
        SPEED_LIMIT_SIGN,
        STOP_SIGN,
        NO_PARKING_SIGN,
        UNKNOWN_SIGN
    };
    // 道路标记:停止线|等待线\直行\左转\右转\直行加左转\直行加右转\全向\掉头(u-turn)\禁止掉头\未知
    enum ROAD_MARK_TYPE
    {
        STOP_LINE_MARK,
        WAITING_LINE_MARK,
        FORWARD_DIRECTION_MARK,
        LEFT_DIRECTION_MARK,
        RIGHT_DIRECTION_MARK,
        FORWARD_LEFT_DIRECTION_MARK,
        FORWARD_RIGHT_DIRECTION_MARK,
        ALL_DIRECTION_MARK,
        U_TURN_DIRECTION_MARK,
        NO_U_TURN_DIRECTION_MARK,
        UNKNOWN_ROAD_MARK
    };
    // 道路方向:同向\反向\双向
    enum ORIENTATION_TYPE
    {
        SAME_DIRECTION,
        OPPOSIT_DIRECTION,
        BOTH_DIRECTIONS
    };
    // 车道类型:无车道\驾驶车道\停车\路肩\自行车道\路边
    enum LANE_TYPE
    {
        UNKNOW,
        NONE_LANE,
        DRIVING_LANE,
        STOP_LANE,
        SHOULDER_LANE,
        BIKING_LANE,
        SIDEWALK_LANE,
        BORDER_LANE,
        RESTRICTED_LANE,
        PARKING_LANE,
        BIDIRECTIONAL_LANE,
        MEDIAN_LANE,
        SPECIAL1_LANE,
        SPECIAL2_LANE,
        SPECIAL3_LANE,
        ROADWORKS_LANE,
        TRAM_LANE,
        RAIL_LANE,
        ENTRY_LANE,
        EXIT_LANE,
        OFFRAMP_LANE,
        ONRAMP_LANE,
        PLANE_LANE
    };

    // opendrive<header>解析
    class OpenDriveHeader
    {

    public:
        OpenDriveHeader()
        {
            rev_major_ = 0;
            rev_minor_ = 0;
            north_ = 0;
            south_ = 0;
            east_ = 0;
            west_ = 0;
            max_road_ = 0;
            max_junc_ = 0;
            max_prg_ = 0;
            date_ = time(0);
        }

        OpenDriveHeader(TiXmlElement *main_element)
        {
            if (main_element != nullptr)
            {
                if (main_element->Attribute("revMajor") != nullptr)
                    rev_major_ = strtol(main_element->Attribute("revMajor"), NULL, 10);
                else
                    rev_major_ = 0;

                if (main_element->Attribute("revMinor") != nullptr)
                    rev_minor_ = strtol(main_element->Attribute("revMinor"), NULL, 10);
                else
                    rev_minor_ = 0;
                if (main_element->Attribute("name") != nullptr)
                    name_ = std::string(main_element->Attribute("name"));

                if (main_element->Attribute("version") != nullptr)
                    version_ = std::string(main_element->Attribute("version"));

                if (main_element->Attribute("date") != nullptr)
                {
                    struct tm _tm;
                    strptime(main_element->Attribute("date"), "%Day %Mon %dd %hh:%mm:%ss %yyyy", &_tm);
                    date_ = mktime(&_tm);
                }

                if (main_element->Attribute("north") != nullptr)
                    north_ = strtod(main_element->Attribute("north"), NULL);
                else
                    north_ = 0;

                if (main_element->Attribute("south") != nullptr)
                    south_ = strtod(main_element->Attribute("south"), NULL);
                else
                    south_ = 0;

                if (main_element->Attribute("east") != nullptr)
                    east_ = strtod(main_element->Attribute("east"), NULL);
                else
                    east_ = 0;

                if (main_element->Attribute("west") != nullptr)
                    west_ = strtod(main_element->Attribute("west"), NULL);
                else
                    west_ = 0;

                if (main_element->Attribute("maxRoad") != nullptr)
                    max_road_ = strtol(main_element->Attribute("maxRoad"), NULL, 10);
                else
                    max_road_ = 0;

                if (main_element->Attribute("maxJunc") != nullptr)
                    max_junc_ = strtol(main_element->Attribute("maxJunc"), NULL, 10);
                else
                    max_junc_ = 0;

                if (main_element->Attribute("maxPrg") != nullptr)
                    max_prg_ = strtol(main_element->Attribute("maxPrg"), NULL, 10);
                else
                    max_prg_ = 0;

                if (main_element->Attribute("vendor") != nullptr)
                    vendor_ = std::string(main_element->Attribute("vendor"));
            }
        }

        int rev_major_;
        int rev_minor_;
        std::string name_;
        std::string version_;
        time_t date_;
        double north_;
        double south_;
        double east_;
        double west_;
        int max_road_;
        int max_junc_;
        int max_prg_;
        std::string vendor_;
    };

    //from <width> from <lane> from <left,right,center> from <laneSection> from <road>
    // from 逐级升高
    class LaneWidth
    {
    public:
        double sOffset, a, b, c, d;
        LaneWidth(TiXmlElement *main_element)
        {
            sOffset = XmlHelpers::getDoubleAttribute(main_element, "sOffset", 0);
            a = XmlHelpers::getDoubleAttribute(main_element, "a", 0);
            b = XmlHelpers::getDoubleAttribute(main_element, "b", 0);
            c = XmlHelpers::getDoubleAttribute(main_element, "c", 0);
            d = XmlHelpers::getDoubleAttribute(main_element, "d", 0);
        }
    };

    //from <roadMark> from <lane> from <left,right,center> from <laneSection> from <road>
    // 左右边界定义
    class RoadMark
    {
    public:
        double sOffset, width;
        LINE_MARKING_TYPE type;
        std::string weight;
        std::string color;

        RoadMark(TiXmlElement *main_element)
        {
            sOffset = XmlHelpers::getDoubleAttribute(main_element, "sOffset", 0);
            width = XmlHelpers::getDoubleAttribute(main_element, "width", 0);
            weight = XmlHelpers::getStringAttribute(main_element, "weight", "");
            color = XmlHelpers::getStringAttribute(main_element, "color", "");
            std::string str_type = XmlHelpers::getStringAttribute(main_element, "type", "");

            if (str_type.compare("broken") == 0)
            {
                type = BROKEN_LINE_MARK;
            }
            else if (str_type.compare("solid") == 0)
            {
                type = SOLID_LINE_MARK;
            }
            else if (str_type.compare("none") == 0)
            {
                type = NONE_LINE_MARK;
            }
            else
            {
                type = UNKNOWN_LINE_MARK;
            }
        }
    };

    class Speed
    {
    public:
        double sOffset, speed;
        std::string unit;

        Speed(TiXmlElement *main_element)
        {
            sOffset = XmlHelpers::getDoubleAttribute(main_element, "sOffset", 0);
            speed = XmlHelpers::getDoubleAttribute(main_element, "max", 0);
            unit = XmlHelpers::getStringAttribute(main_element, "unit", "");

            if (unit.compare("km/h") == 0)
            {
                speed = speed / 3.6;
            }
            if (speed == 0)
            {
                speed = 3;
            }
        }
    };

    //from <laneOffset> from <lanes> from <road>
    // 每次多项式函数改变时，都需要一个新的车道偏移
    class LaneOffset
    {
    public:
        double s, a, b, c, d;
        LaneOffset(TiXmlElement *main_element)
        {
            s = XmlHelpers::getDoubleAttribute(main_element, "s", 0);
            a = XmlHelpers::getDoubleAttribute(main_element, "a", 0);
            b = XmlHelpers::getDoubleAttribute(main_element, "b", 0);
            c = XmlHelpers::getDoubleAttribute(main_element, "c", 0);
            d = XmlHelpers::getDoubleAttribute(main_element, "d", 0);
        }

        double getOffset(const double _s)
        {
            double internal_s = _s - s;
            double h = a + (b * internal_s) + (c * pow(internal_s, 2)) + (d * pow(internal_s, 3));
            return h;
        }
    };

    //from <elevation> from <elevationProfile> from <road>
    //from <superelevation> from <lateralProfile> from <road>
    class Elevation
    {
    public:
        double s, a, b, c, d;
        ELEVATION_TYPE type;
        Elevation(TiXmlElement *main_element)
        {
            std::string val = XmlHelpers::getStringValue(main_element, "");
            if (val.compare("elevation") == 0)
            {
                type = ELEVATION_PROFILE;
            }
            else if (val.compare("superelevation") == 0)
            {
                type = LATERAL_PROFILE;
            }

            s = XmlHelpers::getDoubleAttribute(main_element, "s", 0);
            a = XmlHelpers::getDoubleAttribute(main_element, "a", 0);
            b = XmlHelpers::getDoubleAttribute(main_element, "b", 0);
            c = XmlHelpers::getDoubleAttribute(main_element, "c", 0);
            d = XmlHelpers::getDoubleAttribute(main_element, "d", 0);
        }

        double getHeigh(const double _s)
        {
            double internal_s = _s - s;
            double h = a + (b * internal_s) + (c * pow(internal_s, 2)) + (d * pow(internal_s, 3));
            return h;
        }
    };

    //from <Junction> from <OpenDRIVE>
    class Connection
    {
    public:
        int id_;
        int incoming_section_; //section id in road
        int outgoing_section_;

        int incoming_road_; // road id
        int outgoing_road_;

        bool flag_flip;
        std::string contact_point_;
        std::vector<std::pair<int, int>> lane_links;

        Connection()
        {
            id_ = -1;
            incoming_section_ = -1;
            outgoing_section_ = -1;
            incoming_road_ = -1;
            outgoing_road_ = -1;
            flag_flip = false;
        }

        Connection(TiXmlElement *main_element)
        {
            incoming_section_ = -1;
            id_ = XmlHelpers::getIntAttribute(main_element, "id", 0);
            incoming_road_ = XmlHelpers::getIntAttribute(main_element, "incomingRoad", 0);
            outgoing_road_ = XmlHelpers::getIntAttribute(main_element, "connectingRoad", 0);
            contact_point_ = XmlHelpers::getStringAttribute(main_element, "contactPoint", "");

            std::vector<TiXmlElement *> elements;
            if (main_element != nullptr)
            {
                XmlHelpers::findElements("laneLink", main_element->FirstChildElement(), elements);
                for (unsigned int i = 0; i < elements.size(); i++)
                {
                    int from_id = XmlHelpers::getIntAttribute(elements.at(i), "from", 0);
                    int to_id = XmlHelpers::getIntAttribute(elements.at(i), "to", 0);
                    lane_links.push_back(std::make_pair(from_id, to_id));
                }
            }
        }

        int getToLane(const int &_from)
        {
            for (unsigned int i = 0; i < lane_links.size(); i++)
            {
                if (lane_links.at(i).first == _from)
                    return lane_links.at(i).second;
            }

            return 0;
        }

        int getFromLane(const int &_to)
        {
            for (unsigned int i = 0; i < lane_links.size(); i++)
            {
                if (lane_links.at(i).second == _to)
                    return lane_links.at(i).first;
            }

            return 0;
        }

        void flip()
        {
            int tmp = incoming_road_;
            incoming_road_ = outgoing_road_;
            outgoing_road_ = tmp;

            tmp = incoming_section_;
            incoming_section_ = outgoing_section_;
            outgoing_section_ = tmp;

            for (auto &lane_link : lane_links)
            {
                tmp = lane_link.first;
                lane_link.first = lane_link.second;
                lane_link.second = tmp;
            }
        }

        void flipRoad()
        {
            int tmp = incoming_road_;
            incoming_road_ = outgoing_road_;
            outgoing_road_ = tmp;

            tmp = incoming_section_;
            incoming_section_ = outgoing_section_;
            outgoing_section_ = tmp;
            // flag_flip = true;
        }
    };

    //from <OpenDRIVE>
    class Junction
    {
    public:
        std::string name_;
        int id_;
        std::vector<Connection> connections_;

        Junction(TiXmlElement *main_element)
        {
            name_ = XmlHelpers::getStringAttribute(main_element, "name", "");
            id_ = XmlHelpers::getIntAttribute(main_element, "id", 0);
            std::vector<TiXmlElement *> elements;
            if (main_element != nullptr)
            {
                XmlHelpers::findElements("connection", main_element->FirstChildElement(), elements);
                for (unsigned int i = 0; i < elements.size(); i++)
                {
                    connections_.push_back(Connection(elements.at(i)));
                }
            }
        }
        std::vector<Connection> getConnectionsByRoadId(int road_id)
        {
            std::vector<Connection> ret_connections;
            for (const auto connection : connections_)
            {
                if (connection.incoming_road_ == road_id)
                {
                    ret_connections.push_back(connection);
                }
            }
            return ret_connections;
        }
    };

    class MaxSpeed
    {
    public:
        double speed_;

        MaxSpeed(TiXmlElement *main_element)
        {
            speed_ = 0;
            speed_ = XmlHelpers::getDoubleAttribute(main_element, "max", 0);
            std::string speed_unit = XmlHelpers::getStringAttribute(main_element, "unit", "");
            // convert to m/s
            if (main_element != nullptr)
            {
                if (speed_unit.compare("mph") == 0)
                {
                    speed_ = speed_ * 1.61 / 3.6;
                }
                else if (speed_unit.compare("kmh") == 0)
                {
                    speed_ = speed_ / 3.6;
                }
            }
        }
    };

    //from  <planView> from <road>
    // 定义道路参考线形状
    class Geometry
    {
    public:
        double s, x, y, heading, length; //general use 'line'
        double curveStart, curveEnd;     // for spiral
        double curvature;                //for arc
        double au, bu, cu, du, av, bv, cv, dv, prange;

        GEOMETRY_TYPE type;

        Geometry(TiXmlElement *main_element)
        {
            type = UNKNOWN_GEOMETRY;
            curvature = 0;
            curveStart = 0;
            curveEnd = 0;
            au = bu = cu = du = av = bv = cv = dv = prange = 0;
            s = XmlHelpers::getDoubleAttribute(main_element, "s", 0);
            x = XmlHelpers::getDoubleAttribute(main_element, "x", 0);
            y = XmlHelpers::getDoubleAttribute(main_element, "y", 0);
            heading = XmlHelpers::getDoubleAttribute(main_element, "hdg", 0);
            length = XmlHelpers::getDoubleAttribute(main_element, "length", 0);

            if (main_element != nullptr)
            {
                TiXmlElement *type_element = main_element->FirstChildElement();
                std::string val = XmlHelpers::getStringValue(type_element, "");
                if (val.compare("line") == 0)
                {
                    type = LINE_GEOMETRY;
                }
                else if (val.compare("arc") == 0)
                {
                    type = ARC_GEOMETRY;
                    curvature = XmlHelpers::getDoubleAttribute(type_element, "curvature", 0);
                }
                else if (val.compare("spiral") == 0)
                {
                    type = SPIRAL_GEOMETRY;
                    curveStart = XmlHelpers::getDoubleAttribute(type_element, "curvStart", 0);
                    curveEnd = XmlHelpers::getDoubleAttribute(type_element, "curvEnd", 0);
                }
                else if (val.compare("paramPoly3") == 0)
                {
                    type = PARAM_POLY3_GEOMETRY;
                    au = XmlHelpers::getDoubleAttribute(type_element, "aU", 0);
                    bu = XmlHelpers::getDoubleAttribute(type_element, "bU", 0);
                    cu = XmlHelpers::getDoubleAttribute(type_element, "cU", 0);
                    du = XmlHelpers::getDoubleAttribute(type_element, "dU", 0);
                    av = XmlHelpers::getDoubleAttribute(type_element, "aV", 0);
                    bv = XmlHelpers::getDoubleAttribute(type_element, "bV", 0);
                    cv = XmlHelpers::getDoubleAttribute(type_element, "cV", 0);
                    dv = XmlHelpers::getDoubleAttribute(type_element, "dV", 0);
                    prange = XmlHelpers::getDoubleAttribute(type_element, "pRange", 1);
                }
            }
        }
        // 在road中当前laneSection段获取当前弧长为_s的路点坐标,_s在当前laneSection上返回成功,否则返回失败
        bool getPoint(const double _s, PlannerHNS::WayPoint &wp)
        {
            if (_s < s || _s >= (s + length)) //only calculate points inside the geometry
                return false;
            double internal_s = _s - s;
            // if(internal_s<0.01)
            // 	return false;

            if (type == LINE_GEOMETRY)
            {
                wp.pos.x = x + internal_s * cos(heading);
                wp.pos.y = y + internal_s * sin(heading);
                wp.pos.a = heading;
            }
            else if (type == ARC_GEOMETRY)
            {

                double c = curvature;
                double hdg = heading - M_PI_2;

                double a = 2.0 / c * sin(internal_s * c / 2.0);
                double alpha = ((M_PI - (internal_s * c)) / 2.0) - hdg;

                wp.pos.x = x + (-1.0 * a * cos(alpha));
                wp.pos.y = y + (a * sin(alpha));
                wp.pos.a = heading + internal_s * c;

                //	std::cout << " ARC Calc" << std::endl;
            }
            else if (type == SPIRAL_GEOMETRY)
            {

                double curvDot = (curveEnd - curveStart) / length;

                double _x = 0;
                double _y = 0;
                double _a = 0;

                double rotation_angle = heading;

                PlannerHNS::GPSPoint t_p(_x, _y, 0, _a);

                eulerSpiral(x, y, curveStart, heading, curvDot, internal_s, t_p.x, t_p.y, t_p.a);

                wp.pos.x = t_p.x;
                wp.pos.y = t_p.y;
                wp.pos.z = t_p.z;
                wp.pos.a = t_p.a;
            }
            else if (type == PARAM_POLY3_GEOMETRY)
            {
                double p = (internal_s / length) * prange;
                double u = au + bu * p + cu * pow(p, 2) + du * pow(p, 3);
                double v = av + bv * p + cv * pow(p, 2) + dv * pow(p, 3);

                wp.pos.x = u * cos(heading) - v * sin(heading) + x;
                wp.pos.y = u * sin(heading) + v * cos(heading) + y;
                wp.pos.z = 0.0;
                double derivation_u = bu + cu * p + du * pow(p, 2);
                double derivation_v = bv + cv * p + dv * pow(p, 2);
                double tmp_a = atan2(derivation_v, derivation_u);
                wp.pos.a = heading + tmp_a;
            }

            return true;
        }
    };
    // lane的前驱link
    class FromLaneLink
    {
    public:
        int from_lane_id;
        FromLaneLink(TiXmlElement *main_element)
        {
            from_lane_id = XmlHelpers::getIntAttribute(main_element, "id", 0);
        }
    };
    // lane的后驱link
    class ToLaneLink
    {
    public:
        int to_lane_id;
        ToLaneLink(TiXmlElement *main_element)
        {
            to_lane_id = XmlHelpers::getIntAttribute(main_element, "id", 0);
        }
    };

    //from <left/center/right> from <laneSection> <lanes> from <road>
    // 单条车道的所有元素

    class OpenDriveLane
    {
    public:
        int id_;
        int level_;
        double length_;

        LANE_DIRECTION dir_;
        LANE_TYPE type_;
        std::vector<int> fromIds_;
        std::vector<int> toIds_;
        std::vector<LaneWidth> width_list_;
        std::vector<RoadMark> mark_list_;
        double speed_;
        // std::vector<Speed> speed_list_;
        std::vector<FromLaneLink> from_lane_;
        std::vector<ToLaneLink> to_lane_;

        // LANE_DIRECTION:left/center/right
        OpenDriveLane(TiXmlElement *main_element,
                      const LANE_DIRECTION &_dir,
                      const double &s_offset,
                      const double &lane_length,
                      const int &section_index)
        {
            dir_ = _dir;
            type_ = DRIVING_LANE;
            id_ = XmlHelpers::getIntAttribute(main_element, "id", 0);
            level_ = XmlHelpers::getIntAttribute(main_element, "level", 0);
            std::string str_type = XmlHelpers::getStringAttribute(main_element, "type", "");
            length_ = lane_length;

            if (str_type.compare("driving") == 0)
                type_ = DRIVING_LANE;
            else if (str_type.compare("shoulder") == 0)
                type_ = SHOULDER_LANE;
            else if (str_type.compare("sidewalk") == 0)
                type_ = SIDEWALK_LANE;
            else if (str_type.compare("none") == 0)
                type_ = NONE_LANE;
            else if (str_type.compare("parking") == 0)
                type_ = PARKING_LANE;
            else
                type_ = UNKNOW;

            std::vector<TiXmlElement *> elements;
            XmlHelpers::findFirstElement("link", main_element, elements);

            if (elements.size() > 0)
            {
                std::vector<TiXmlElement *> pred_elements, succ_elements;
                XmlHelpers::findElements("predecessor", elements.at(0)->FirstChildElement(), pred_elements);
                for (unsigned int j = 0; j < pred_elements.size(); j++)
                {
                    // from_lane_.push_back(FromLaneLink(pred_elements.at(j)));
                    if (id_ > 0) //left lanes
                    {
                        to_lane_.push_back(ToLaneLink(pred_elements.at(j)));
                    }
                    else if (id_ < 0) //right lanes
                    {
                        from_lane_.push_back(FromLaneLink(pred_elements.at(j)));
                    }
                }

                XmlHelpers::findElements("successor", elements.at(0)->FirstChildElement(), succ_elements);
                for (unsigned int j = 0; j < succ_elements.size(); j++)
                {
                    // to_lane_.push_back(ToLaneLink(succ_elements.at(j)));
                    if (id_ > 0) //left lanes
                    {
                        from_lane_.push_back(FromLaneLink(succ_elements.at(j)));
                    }
                    else if (id_ < 0)
                    {
                        to_lane_.push_back(ToLaneLink(succ_elements.at(j)));
                    }
                }
            }

            std::vector<TiXmlElement *> width_elements;
            if (main_element != nullptr)
            {
                XmlHelpers::findElements("width", main_element->FirstChildElement(), width_elements);
                for (unsigned int i = 0; i < width_elements.size(); i++)
                {
                    width_list_.push_back(LaneWidth(width_elements.at(i)));
                }
            }
            std::vector<TiXmlElement *> mark_elements;
            if (main_element != nullptr)
            {
                XmlHelpers::findElements("roadMark", main_element->FirstChildElement(), mark_elements);
                for (unsigned int i = 0; i < mark_elements.size(); i++)
                {
                    mark_list_.push_back(RoadMark(mark_elements.at(i)));
                }
            }

            std::vector<TiXmlElement *> speed_elements;
            if (main_element != nullptr)
            {
                XmlHelpers::findElements("speed", main_element->FirstChildElement(), speed_elements);
                for (unsigned int i = 0; i < speed_elements.size(); i++)
                {
                    speed_ = Speed(speed_elements.at(i)).speed;
                    // speed_list_.push_back(Speed(speed_elements.at(i)));
                }
            }
        }
        // 在width_list_找自身lane对应的width参数
        LaneWidth *getMatchingWidth(const double &sOffset)
        {
            if (width_list_.size() == 0)
                return nullptr;

            if (width_list_.size() == 1)
                return &width_list_.at(0);

            for (int i = 1; i < width_list_.size(); i++)
            {
                if (sOffset >= width_list_.at(i - 1).sOffset && sOffset < width_list_.at(i).sOffset)
                {
                    return &width_list_.at(i - 1);
                }
            }

            return &width_list_.at(width_list_.size() - 1);
        }

        // 在width_list_找自身lane对应的width参数
        RoadMark *getMatchingRoadMark(const double &sOffset)
        {
            if (mark_list_.size() == 0)
                return nullptr;

            if (mark_list_.size() == 1)
                return &mark_list_.at(0);

            for (int i = 1; i < mark_list_.size(); i++)
            {
                if (sOffset >= mark_list_.at(i - 1).sOffset && sOffset < mark_list_.at(i).sOffset)
                {
                    return &mark_list_.at(i - 1);
                }
            }

            return &mark_list_.at(mark_list_.size() - 1);
        }

        LINE_MARKING_TYPE getLaneMark(const double &sOffset)
        {
            RoadMark *p_mark = getMatchingRoadMark(sOffset);

            if (p_mark != nullptr)
            {
                return p_mark->type;
            }

            return LINE_MARKING_TYPE::SOLID_LINE_MARK;
        }

        double getLaneWidth(const double &sOffset)
        {
            LaneWidth *p_width = getMatchingWidth(sOffset);

            if (p_width != nullptr)
            {
                double s_local = sOffset - p_width->sOffset;
                double width = p_width->a + (p_width->b * s_local) + (p_width->c * pow(s_local, 2)) + (p_width->d * pow(s_local, 3));
                return width;
            }
            return 0;
        }
    };
    // 道路参考中心线信息
    class RoadCenterInfo
    {
    public:
        double ds_;
        PlannerHNS::GPSPoint center_p_;
        // laneoffset 实际道路左右方向边界到道路几何中心线的距离
        double offset_width_;

        RoadCenterInfo()
        {
            ds_ = 0;
            offset_width_ = 0;
        }
    };

    //laneSection中所有元素
    class RoadSection
    {
    public:
        //start arc
        double s_;
        // section id in road <0>
        int id_;
        double length_;
        std::vector<OpenDriveLane> left_lanes_;
        std::vector<OpenDriveLane> right_lanes_;
        std::vector<OpenDriveLane> center_lane_;
        RoadSection *p_next_section_;
        RoadSection *p_prev_section_;
        // 构造函数内赋值
        RoadSection(TiXmlElement *main_element,
                    const double &s_offset,
                    const double &section_length,
                    const int &section_index)
        {
            length_ = section_length;
            id_ = section_index;
            s_ = s_offset;
            p_next_section_ = nullptr;
            p_prev_section_ = nullptr;

            std::vector<TiXmlElement *> sub_elements;
            std::vector<TiXmlElement *> lane_elements;

            sub_elements.clear();
            XmlHelpers::findFirstElement("left", main_element, sub_elements);
            if (sub_elements.size() > 0)
            {
                lane_elements.clear();
                XmlHelpers::findElements("lane", sub_elements.at(0)->FirstChildElement(), lane_elements);
                for (unsigned int k = 0; k < lane_elements.size(); k++)
                {
                    OpenDriveLane parsed_lane = OpenDriveLane(lane_elements.at(k), LEFT_LANE, s_offset, section_length, section_index);
                    // if (parsed_lane.type_ == DRIVING_LANE)
                    left_lanes_.push_back(parsed_lane);
                }
            }

            sub_elements.clear();
            XmlHelpers::findFirstElement("center", main_element, sub_elements);
            if (sub_elements.size() > 0)
            {
                lane_elements.clear();
                XmlHelpers::findElements("lane", sub_elements.at(0)->FirstChildElement(), lane_elements);
                for (unsigned int k = 0; k < lane_elements.size(); k++)
                {
                    center_lane_.push_back(OpenDriveLane(lane_elements.at(k), CENTER_LANE, s_offset, section_length, section_index));
                }
            }

            sub_elements.clear();
            XmlHelpers::findFirstElement("right", main_element, sub_elements);
            if (sub_elements.size() > 0)
            {
                lane_elements.clear();
                XmlHelpers::findElements("lane", sub_elements.at(0)->FirstChildElement(), lane_elements);
                for (unsigned int k = 0; k < lane_elements.size(); k++)
                {
                    OpenDriveLane parsed_lane = OpenDriveLane(lane_elements.at(k), RIGHT_LANE, s_offset, section_length, section_index);
                    // if (parsed_lane.type_ == DRIVING_LANE)
                    right_lanes_.push_back(parsed_lane);
                }
            }

            std::sort(left_lanes_.begin(), left_lanes_.end(), sort_lanes_asc);
            std::sort(right_lanes_.begin(), right_lanes_.end(), sort_lanes_desc);
        }

        // 按升序排列
        static bool sort_lanes_asc(const OpenDriveLane &l1, const OpenDriveLane &l2)
        {
            if (l1.id_ < l2.id_)
                return true;
            else
                return false;
        }
        // 按降序排列
        static bool sort_lanes_desc(const OpenDriveLane &l1, const OpenDriveLane &l2)
        {
            if (l1.id_ > l2.id_)
                return true;
            else
                return false;
        }
    };

    //from <signals> from <road>
    class Signal
    {
    public:
        int id_;
        std::string name_;
        std::string text_;
        std::string unit_;
        std::string country_code_;

        double s_;
        double t_;
        double value_;
        double h_;
        double w_;
        double yaw_; //hOffset
        double pitch_;
        double roll_;
        double zOffset_;
        bool dynamic_;
        ORIENTATION_TYPE orientation_;
        std::string type_;
        std::string sub_type_;
        // if empty, use orientation to juage vaild lanes
        std::vector<int> valid_lanes_ids_;
        std::string signal_location;

        Signal(TiXmlElement *main_element)
        {
            name_ = XmlHelpers::getStringAttribute(main_element, "name", "");
            s_ = XmlHelpers::getDoubleAttribute(main_element, "s", 0);
            t_ = XmlHelpers::getDoubleAttribute(main_element, "t", 0);
            id_ = XmlHelpers::getIntAttribute(main_element, "id", 0);

            text_ = XmlHelpers::getStringAttribute(main_element, "text", "");
            unit_ = XmlHelpers::getStringAttribute(main_element, "unit", "");
            country_code_ = XmlHelpers::getStringAttribute(main_element, "country", "");

            zOffset_ = XmlHelpers::getDoubleAttribute(main_element, "zOffset", 0);
            value_ = XmlHelpers::getDoubleAttribute(main_element, "value", 0);
            h_ = XmlHelpers::getDoubleAttribute(main_element, "height", 0);
            w_ = XmlHelpers::getDoubleAttribute(main_element, "width", 0);
            yaw_ = XmlHelpers::getDoubleAttribute(main_element, "hOffset", 0);
            roll_ = XmlHelpers::getDoubleAttribute(main_element, "roll", 0);
            pitch_ = XmlHelpers::getDoubleAttribute(main_element, "pitch", 0);

            std::string dynamic_str = XmlHelpers::getStringAttribute(main_element, "dynamic", "");
            if (dynamic_str.compare("yes") == 0)
                dynamic_ = true;
            else
                dynamic_ = false;

            std::string orientation_str = XmlHelpers::getStringAttribute(main_element, "orientation", "");
            if (orientation_str.compare("+") == 0)
                orientation_ = SAME_DIRECTION;
            else if (orientation_str.compare("-") == 0)
                orientation_ = OPPOSIT_DIRECTION;
            else
                orientation_ = BOTH_DIRECTIONS;

            type_ = XmlHelpers::getStringAttribute(main_element, "type", "");
            sub_type_ = XmlHelpers::getStringAttribute(main_element, "subtype", "");

            if (main_element != nullptr)
            {
                std::vector<TiXmlElement *> elements;
                elements.clear();
                XmlHelpers::findElements("validity", main_element->FirstChildElement(), elements);
                for (unsigned int i = 0; i < elements.size(); i++)
                {
                    int _from_lane = XmlHelpers::getIntAttribute(elements.at(i), "fromLane", 0);
                    int _to_lane = XmlHelpers::getIntAttribute(elements.at(i), "toLane", 0);

                    for (int k = _from_lane; k < _to_lane; k++)
                    {
                        valid_lanes_ids_.push_back(k);
                    }
                }
            }

            if (main_element != nullptr)
            {
                std::vector<TiXmlElement *> elements;
                elements.clear();
                XmlHelpers::findElements("userData", main_element->FirstChildElement(), elements);
                for (unsigned int i = 0; i < elements.size(); i++)
                {
                    std::vector<TiXmlElement *> user_elements;
                    user_elements.clear();
                    XmlHelpers::findElements("vectorSignal", elements.at(i)->FirstChildElement(), user_elements);
                    {
                        for (unsigned int i = 0; i < user_elements.size(); i++)
                        {
                            signal_location = XmlHelpers::getStringAttribute(user_elements.at(i), "signalId", "");
                        }
                    }
                }
            }
        }
    };

    //from <signals> from <road>
    // 为了在多条道路上读一个红绿灯设立的
    class SignalRef
    {
    public:
        double s_;
        double t_;
        int id_;
        ORIENTATION_TYPE orientation_;
        std::vector<int> valid_lanes_ids_;

        SignalRef(TiXmlElement *main_element)
        {
            s_ = XmlHelpers::getDoubleAttribute(main_element, "s", 0);
            t_ = XmlHelpers::getDoubleAttribute(main_element, "t", 0);
            id_ = XmlHelpers::getIntAttribute(main_element, "id", 0);
            std::string orientation_str =
                XmlHelpers::getStringAttribute(main_element, "orientation", "");
            if (orientation_str.compare("+") == 0)
                orientation_ = SAME_DIRECTION;
            else if (orientation_str.compare("-") == 0)
                orientation_ = OPPOSIT_DIRECTION;

            if (main_element != nullptr)
            {
                std::vector<TiXmlElement *> elements;
                elements.clear();
                XmlHelpers::findElements("validity", main_element->FirstChildElement(), elements);
                for (unsigned int i = 0; i < elements.size(); i++)
                {
                    int _from_lane = XmlHelpers::getIntAttribute(elements.at(i), "fromLane", 0);
                    int _to_lane = XmlHelpers::getIntAttribute(elements.at(i), "toLane", 0);

                    for (int k = _from_lane; k < _to_lane; k++)
                    {
                        valid_lanes_ids_.push_back(k);
                    }
                }
            }
        }
    };

    class cornerLocal
    {
    public:
        double u_;
        double v_;

        cornerLocal(TiXmlElement *main_element)
        {
            u_ = XmlHelpers::getDoubleAttribute(main_element, "u", 0);
            v_ = XmlHelpers::getDoubleAttribute(main_element, "v", 0);
        }
    };

    //from <objects> from <road>
    class RoadObject
    {
    public:
        std::string name_;
        int id_;
        double s_;
        double t_;
        double zOffset_;
        ORIENTATION_TYPE orientation_;
        double l_;
        double w_;
        double h_;
        double yaw_; //hOffset, hdg
        double pitch_;
        double roll_;
        std::vector<cornerLocal> corner_local;

        RoadObject(TiXmlElement *main_element)
        {
            s_ = XmlHelpers::getDoubleAttribute(main_element, "s", 0);
            t_ = XmlHelpers::getDoubleAttribute(main_element, "t", 0);
            id_ = XmlHelpers::getIntAttribute(main_element, "id", 0);
            name_ = XmlHelpers::getStringAttribute(main_element, "name", "");
            zOffset_ = XmlHelpers::getDoubleAttribute(main_element, "zOffset", 0);
            h_ = XmlHelpers::getDoubleAttribute(main_element, "height", 0);
            w_ = XmlHelpers::getDoubleAttribute(main_element, "width", 0);
            l_ = XmlHelpers::getDoubleAttribute(main_element, "length", 0);
            yaw_ = XmlHelpers::getDoubleAttribute(main_element, "hdg", 0);
            roll_ = XmlHelpers::getDoubleAttribute(main_element, "roll", 0);
            pitch_ = XmlHelpers::getDoubleAttribute(main_element, "pitch", 0);

            std::string orientation_str = XmlHelpers::getStringAttribute(main_element, "orientation", "");
            if (orientation_str.compare("+") == 0)
                orientation_ = SAME_DIRECTION;
            else if (orientation_str.compare("-") == 0)
                orientation_ = OPPOSIT_DIRECTION;
            else
                orientation_ = BOTH_DIRECTIONS;

            std::vector<TiXmlElement *> sub_elements;
            std::vector<TiXmlElement *> corner_elements;
            sub_elements.clear();
            XmlHelpers::findFirstElementInOneRoad("outline", main_element, sub_elements);
            // std::cout<<"sub_elements size "<<sub_elements.size()<<std::endl;
            if (sub_elements.size() > 0)
            {
                corner_elements.clear();
                XmlHelpers::findElements(
                    "cornerLocal", sub_elements.at(0)->FirstChildElement(), corner_elements);
                // std::cout<<"corner_elements size "<<corner_elements.size()<<std::endl;
                for (unsigned int k = 0; k < corner_elements.size(); k++)
                {
                    cornerLocal parsed_corner = cornerLocal(corner_elements.at(k));
                    corner_local.push_back(parsed_corner);
                }
            }
        }
    };
    // 道路对象,目前只有停车场
    class RoadObjectRef
    {
    public:
        RoadObjectRef(TiXmlElement *main_element)
        {
        }
    };

    class RoadObjectTunnel
    {
    public:
        RoadObjectTunnel(TiXmlElement *main_element)
        {
        }
    };

    class RoadObjectBridge
    {
    public:
        RoadObjectBridge(TiXmlElement *main_element)
        {
        }
    };
} // namespace opendrive

#endif // OPENDRIVE_OBJECTS
