#include "op_planner/OpendriveMapRoad.h"

namespace opendrive
{
    std::fstream lane_file;

    double g_epsilon = 0.00001;
    bool append_0 = false;
    bool append_1 = false;
    bool append_2 = false;

    OpenDriveRoad::OpenDriveRoad(
        TiXmlElement *main_element,
        std::vector<std::pair<std::string, std::vector<CSV_Reader::LINE_DATA>>> *country_signal_codes,
        bool keep_right)
    {
        name_ = XmlHelpers::getStringAttribute(main_element, "name", "");
        id_ = XmlHelpers::getIntAttribute(main_element, "id", 0);
        junction_id_ = XmlHelpers::getIntAttribute(main_element, "junction", -1);
        length_ = XmlHelpers::getDoubleAttribute(main_element, "length", 0.0);
        p_country_signal_codes_ = country_signal_codes;
        keep_right_ = keep_right;

        //Read Links
        std::vector<TiXmlElement *> sub_elements;
        std::vector<TiXmlElement *> lane_elements;
        std::vector<TiXmlElement *> elements;

        XmlHelpers::findFirstElement("link", main_element, elements);
        if (elements.size() > 0)
        {
            std::vector<TiXmlElement *> pred_elements, succ_elements;

            XmlHelpers::findElements("predecessor", elements.at(0)->FirstChildElement(), pred_elements);

            for (unsigned int j = 0; j < pred_elements.size(); j++)
            {
                predecessor_road_.push_back(FromRoadLink(pred_elements.at(j)));
            }

            XmlHelpers::findElements("successor", elements.at(0)->FirstChildElement(), succ_elements);
            for (unsigned int j = 0; j < succ_elements.size(); j++)
            {
                successor_road_.push_back(ToRoadLink(succ_elements.at(j)));
            }
        }

        elements.clear();
        XmlHelpers::findFirstElement("type", main_element, elements);
        if (elements.size() > 0)
        {
            road_type_ = XmlHelpers::getStringAttribute(elements.at(0), "type", "town");

            //Get Geometries
            std::vector<TiXmlElement *> speed_elements;
            XmlHelpers::findElements("speed", elements.at(0), speed_elements);
            for (unsigned int j = 0; j < speed_elements.size(); j++)
            {
                max_speed = MaxSpeed(speed_elements.at(j)).speed_;
            }
        }

        elements.clear();
        XmlHelpers::findFirstElement("planView", main_element, elements);
        if (elements.size() > 0)
        {
            //Get Geometries
            std::vector<TiXmlElement *> geom_elements;
            XmlHelpers::findElements("geometry", elements.at(0), geom_elements);
            for (unsigned int j = 0; j < geom_elements.size(); j++)
            {
                double length = XmlHelpers::getDoubleAttribute(geom_elements.at(j), "length", 0.0);
                geometries_.push_back(Geometry(geom_elements.at(j)));
            }
        }

        elements.clear();
        XmlHelpers::findFirstElement("elevationProfile", main_element, elements);
        if (elements.size() > 0)
        {
            std::vector<TiXmlElement *> elev_elements;
            XmlHelpers::findElements("elevation", elements.at(0), elev_elements);
            for (unsigned int j = 0; j < elev_elements.size(); j++)
            {
                elevations_.push_back(Elevation(elev_elements.at(j)));
            }
        }

        elements.clear();
        XmlHelpers::findFirstElement("lanes", main_element, elements);
        if (elements.size() > 0)
        {
            //laneOffsets
            std::vector<TiXmlElement *> offsets;
            XmlHelpers::findElements("laneOffset", elements.at(0), offsets);
            for (unsigned int j = 0; j < offsets.size(); j++)
            {
                laneOffsets_.push_back(LaneOffset(offsets.at(j)));
            }

            //laneSections and lanes
            std::vector<TiXmlElement *> sections;
            XmlHelpers::findElements("laneSection", elements.at(0), sections);

            for (unsigned int j = 0; j < sections.size(); j++)
            {
                double curr_section_s = XmlHelpers::getDoubleAttribute(sections.at(j), "s", 0.0);
                double next_section_s = 0.0;
                double section_length = 0.0;
                // not last lane section
                if (j < sections.size() - 1)
                {
                    next_section_s = XmlHelpers::getDoubleAttribute(sections.at(j + 1), "s", 0.0);
                    // set 0.001 to keep double accuracy
                    section_length = next_section_s - curr_section_s;
                    // section_length = next_section_s - curr_section_s;
                }
                // last lane section
                else
                {
                    section_length = length_ - curr_section_s;
                }
                // 如果lanesection的长度有问题
                if (section_length <= 0)
                {
                    std::cout << "Too small Section length!!" << std::endl;
                    continue;
                }
                sections_.push_back(RoadSection(sections.at(j), curr_section_s, section_length, j));
            }
        }

        elements.clear();
        XmlHelpers::findFirstElementInOneRoad("signals", main_element, elements);
        // std::cout<<"elements.size()"<<elements.size()<<std::endl;
        if (elements.size() > 0)
        {
            std::vector<TiXmlElement *> signals_;
            XmlHelpers::findElements("signal", elements.at(0), signals_);

            for (unsigned int j = 0; j < signals_.size(); j++)
            {
                road_signals_.push_back(Signal(signals_.at(j)));
            }
        }

        elements.clear();
        XmlHelpers::findFirstElementInOneRoad("objects", main_element, elements);
        if (elements.size() > 0)
        {
            std::vector<TiXmlElement *> object_;
            XmlHelpers::findElements("object", elements.at(0), object_);

            for (unsigned int j = 0; j < object_.size(); j++)
            {
                road_objects_.push_back(RoadObject(object_.at(j)));
            }
        }
    }
    
    // 获取弧长所对应的道路中心上的路点
    bool OpenDriveRoad::createSingleCenterPoint(double _ds, PlannerHNS::WayPoint &_p)
    {
        for (unsigned int i = 0; i < geometries_.size(); i++)
        {
            if (_ds >= geometries_.at(i).s && _ds <= (geometries_.at(i).s + geometries_.at(i).length))
            {
                if (geometries_.at(i).getPoint(_ds, _p))
                {
                    Elevation *p_elv = getMatchingElevations(_ds);
                    if (p_elv != nullptr)
                    {
                        _p.pos.z = p_elv->getHeigh(_ds);
                    }
                    return true;
                }
            }
        }

        return false;
    }
    // get road center point by arc
    bool OpenDriveRoad::createRoadCenterPoint(RoadCenterInfo &inf_point, double _s)
    {
        for (unsigned int i = 0; i < geometries_.size(); i++)
        {
            double end_distance = geometries_.at(i).s + geometries_.at(i).length;
            //if arc in this geometry
            if (_s >= geometries_.at(i).s && _s <= end_distance)
            {
                RoadCenterInfo inf;
                PlannerHNS::WayPoint p;
                if (geometries_.at(i).getPoint(_s, p))
                {
                    Elevation *p_elv = getMatchingElevations(_s);
                    if (p_elv != nullptr)
                    {
                        p.pos.z = p_elv->getHeigh(_s);
                    }

                    LaneOffset *p_lane_off = getMatchingLaneOffset(_s);
                    if (p_lane_off != nullptr)
                    {
                        inf.offset_width_ = p_lane_off->getOffset(_s);
                        double a = p.pos.a + M_PI_2;
                        p.pos.x += inf.offset_width_ * cos(a);
                        p.pos.y += inf.offset_width_ * sin(a);
                        inf.center_p_ = p.pos;
                    }
                    else
                    {
                        inf.center_p_ = p.pos;
                    }

                    inf.ds_ = _s;
                    inf.center_p_ = p.pos;
                    inf_point = inf;
                    return true;
                }
            }
        }

        return false;
    }
    // check points exist and add end point
    void OpenDriveRoad::insertRoadCenterInfo(std::vector<RoadCenterInfo> &points_list,
                                             RoadCenterInfo &inf_point)
    {
        for (unsigned int i = 0; i < points_list.size(); i++)
        {
            if (inf_point.ds_ == points_list.at(i).ds_) // exist
            {
                return;
            }
            else if (inf_point.ds_ < points_list.at(i).ds_)
            {
                points_list.insert(points_list.begin() + i, inf_point);
                return;
            }
        }

        points_list.push_back(inf_point);
    }
    // creat reference center line points only based on geometry
    std::fstream reference_line;
    void OpenDriveRoad::createRoadCenterInfo(std::vector<RoadCenterInfo> &points_list,
                                             double resolution)
    {
        PlannerHNS::WayPoint p;
        RoadCenterInfo inf;
        points_list.clear();
        if (append_1)
        {
            reference_line.open("/home/chen/log/reference_line.txt", std::ios::app);
        }
        else
        {
            append_1 = true;
            reference_line.open("/home/chen/log/reference_line.txt", std::ios::out);
        }

        for (unsigned int i = 0; i < geometries_.size(); i++)
        {
            int n_waypoints = floor(geometries_.at(i).length / resolution) + 1;
            double s_inc = geometries_.at(i).s;
            double remaining_distance = 0.0;
            double end_distance = geometries_.at(i).s + geometries_.at(i).length;

            for (int j = 0; j < n_waypoints; j++)
            {
                if (geometries_.at(i).getPoint(s_inc, p))
                {
                    Elevation *p_elv = getMatchingElevations(s_inc);
                    if (p_elv != nullptr)
                    {
                        p.pos.z = p_elv->getHeigh(s_inc);
                    }

                    LaneOffset *p_lane_off = getMatchingLaneOffset(s_inc);
                    if (p_lane_off != nullptr)
                    {
                        inf.offset_width_ = p_lane_off->getOffset(s_inc);
                        double a = p.pos.a + M_PI_2;
                        p.pos.x += inf.offset_width_ * cos(a);
                        p.pos.y += inf.offset_width_ * sin(a);
                        inf.center_p_ = p.pos;
                    }
                    else
                    {
                        inf.center_p_ = p.pos;
                    }

                    inf.ds_ = s_inc;
                    points_list.push_back(inf);
                }
                // refresh arc
                remaining_distance = end_distance - s_inc;
                if (remaining_distance < resolution)
                {
                    if (remaining_distance > g_epsilon)
                    {
                        s_inc += remaining_distance;
                        if (geometries_.at(i).getPoint(s_inc, p))
                        {
                            Elevation *p_elv = getMatchingElevations(s_inc);
                            if (p_elv != nullptr)
                            {
                                p.pos.z = p_elv->getHeigh(s_inc);
                            }

                            LaneOffset *p_lane_off = getMatchingLaneOffset(s_inc);
                            if (p_lane_off != nullptr)
                            {
                                inf.offset_width_ = p_lane_off->getOffset(s_inc);
                                double a = p.pos.a + M_PI_2;
                                p.pos.x += inf.offset_width_ * cos(a);
                                p.pos.y += inf.offset_width_ * sin(a);
                                inf.center_p_ = p.pos;
                            }
                            else
                            {
                                inf.center_p_ = p.pos;
                            }

                            inf.ds_ = s_inc;
                            points_list.push_back(inf);
                            reference_line << p.pos.x << "," << p.pos.y << ", " << p.pos.z << std::endl;
                        }
                    }
                }
                else
                    s_inc += resolution;

                reference_line << p.pos.x << "," << p.pos.y << ", " << p.pos.z << std::endl;
            }
        }

        // //insert start and end point
        // for (unsigned int i = 0; i < sections_.size(); i++)
        // {
        //     double s_inc = sections_.at(i).s_;
        //     if (createRoadCenterPoint(inf, s_inc))
        //     {
        //         insertRoadCenterInfo(points_list, inf);
        //     }
        //     s_inc = sections_.at(i).s_ + sections_.at(i).length_;

        //     if (createRoadCenterPoint(inf, s_inc))
        //     {
        //         insertRoadCenterInfo(points_list, inf);
        //     }
        // }

        reference_line.close();
    }

    void OpenDriveRoad::insertUniqueFromSectionIds(int from_section_id,
                                                   const OpenDriveLane *curr_lane,
                                                   PlannerHNS::Lane &_l)
    {
        for (unsigned int i = 0; i < curr_lane->from_lane_.size(); i++)
        {
            int from_lane_id = curr_lane->from_lane_.at(i).from_lane_id;
            int from_gen_id = (id_ * 100) + from_section_id * 10 + from_lane_id + 10;

            if (exists(_l.fromIds, from_gen_id))
            {
                std::cout << "Redundant Connection, from road: " << id_ << ", to road: " << id_
                          << ", to section: " << from_section_id + 1 << ", GenLaneID: " << from_gen_id << std::endl;
            }
            else
            {
                _l.fromIds.push_back(from_gen_id);
            }
        }
    }

    void OpenDriveRoad::insertUniqueToSectionIds(int to_section_id,
                                                 const OpenDriveLane *curr_lane,
                                                 PlannerHNS::Lane &_l)
    {
        for (unsigned int i = 0; i < curr_lane->to_lane_.size(); i++)
        {
            int to_lane_id = curr_lane->to_lane_.at(i).to_lane_id;
            int to_gen_id = (id_ * 100) + to_section_id * 10 + to_lane_id + 10;

            if (exists(_l.toIds, to_gen_id))
            {
                std::cout << "Redundant Connection, from road: " << id_ << ", to road: " << id_
                          << ", from section: " << to_section_id - 1 << ", GenLaneID: " << to_gen_id << std::endl;
            }
            else
            {
                _l.toIds.push_back(to_gen_id);
            }
        }
    }

    void OpenDriveRoad::insertUniqueFromRoadIds(int curr_section_id,
                                                int curr_lane_id,
                                                PlannerHNS::Lane &_l)
    {

        for (unsigned int i = 0; i < from_roads_.size(); i++)
        {
            int from_lane_id = 0;

            from_lane_id = from_roads_.at(i).getFromLane(curr_lane_id);

            if (from_lane_id != 0)
            {
                if (from_roads_.at(i).outgoing_road_ != id_)
                    std::cout << "Something Very Bad Happened in InsertUniqueFromRoadIds, outgoing_road doesn't match current_road, "
                              << from_roads_.at(i).outgoing_road_ << ", " << id_ << std::endl;

                int from_gen_id =
                    (from_roads_.at(i).incoming_road_ * 100) +
                    from_roads_.at(i).incoming_section_ * 10 + from_lane_id + 10;
                if (exists(_l.fromIds, from_gen_id))
                {
                    std::cout << "Redundant Connection, from road: " << from_roads_.at(i).incoming_road_ << ", to road: " << id_
                              << ", to section: " << curr_section_id << ", GenLaneID: " << from_gen_id << std::endl;
                }
                else
                {
                    _l.fromIds.push_back(from_gen_id);
                }
            }
        }
    }

    void OpenDriveRoad::insertUniqueToRoadIds(int curr_section_id,
                                              int curr_lane_id,
                                              PlannerHNS::Lane &_l)
    {

        for (unsigned int i = 0; i < to_roads_.size(); i++)
        {

            int to_lane_id = 0;
            to_lane_id = to_roads_.at(i).getToLane(curr_lane_id);
            if (to_lane_id != 0)
            {
                if (to_roads_.at(i).incoming_road_ != id_)
                    std::cout << "incoming_road doesn't match current_road, "
                              << to_roads_.at(i).incoming_road_ << ", " << id_ << std::endl;

                int to_gen_id = (to_roads_.at(i).outgoing_road_ * 100) +
                                to_roads_.at(i).outgoing_section_ * 10 + to_lane_id + 10;
                if (exists(_l.toIds, to_gen_id))
                {
                    std::cout << "Redundant Connection, to road: " << to_roads_.at(i).outgoing_road_ << ", from road: " << id_
                              << ", from section: " << curr_section_id << ", GenLaneID: " << to_gen_id << std::endl;
                }
                else
                {
                    _l.toIds.push_back(to_gen_id);
                }
            }
        }
    }

    void OpenDriveRoad::insertUniqueToConnection(const Connection &_connection)
    {

        bool bFound = false;
        // for every successor road
        for (unsigned int j = 0; j < to_roads_.size(); j++)
        {
            if (to_roads_.at(j).incoming_road_ == _connection.incoming_road_ &&
                to_roads_.at(j).outgoing_road_ == _connection.outgoing_road_)
            {
                for (unsigned int k = 0; k < to_roads_.at(j).lane_links.size(); k++)
                {
                    for (unsigned int lk = 0; lk < _connection.lane_links.size(); lk++)
                    {
                        if (to_roads_.at(j).lane_links.at(k).first ==
                                _connection.lane_links.at(lk).first &&
                            to_roads_.at(j).lane_links.at(k).second ==
                                _connection.lane_links.at(lk).second)
                        {
                            bFound = true;
                            break;
                        }
                    }

                    if (bFound == true)
                        break;
                }

                if (bFound == true)
                    break;
            }
        }

        if (!bFound)
        {
            to_roads_.push_back(_connection);
        }
        else
        {
            std::cout << "To Connection already exists, From : " << _connection.incoming_road_
                      << ", To: " << _connection.outgoing_road_ << std::endl;
        }
    }

    void OpenDriveRoad::insertUniqueFromConnection(const Connection &_connection)
    {
        bool bFound = false;
        // check connection whether exist or not
        for (unsigned int j = 0; j < from_roads_.size(); j++)
        {
            if (from_roads_.at(j).incoming_road_ == _connection.incoming_road_ &&
                from_roads_.at(j).outgoing_road_ == _connection.outgoing_road_)
            {
                for (unsigned int k = 0; k < from_roads_.at(j).lane_links.size(); k++)
                {
                    for (unsigned int lk = 0; lk < _connection.lane_links.size(); lk++)
                    {
                        if ((from_roads_.at(j).lane_links.at(k).first ==
                             _connection.lane_links.at(lk).first) &&
                            (from_roads_.at(j).lane_links.at(k).second ==
                             _connection.lane_links.at(lk).second))
                        {
                            bFound = true;
                            break;
                        }
                    }

                    if (bFound == true)
                        break;
                }

                if (bFound == true)
                    break;
            }
        }

        if (!bFound)
        {
            from_roads_.push_back(_connection);
        }
        else
        {
            std::cout << "From Connection already exists, From : " << _connection.incoming_road_
                      << ", To: " << _connection.outgoing_road_ << std::endl;
        }
    }
    
    // create all lanes in one road
    // assign id\num\roadId\length\fromIds\toIds\speed
    void OpenDriveRoad::createRoadLanes(std::vector<PlannerHNS::Lane> &lanes_list)
    {

        using namespace std::placeholders;
        std::function<void(int, int, PlannerHNS::Lane &)> insertToRoad;
        std::function<void(int, int, PlannerHNS::Lane &)> insertFromRoad;
        std::function<void(int, const OpenDriveLane *, PlannerHNS::Lane &)> insertToSection;
        std::function<void(int, const OpenDriveLane *, PlannerHNS::Lane &)> insertFromSection;

        //flip to and from depending on keep right or keep left rule
        if (keep_right_)
        {
            insertToRoad = std::bind(&OpenDriveRoad::insertUniqueToRoadIds, this, _1, _2, _3);
            insertFromRoad = std::bind(&OpenDriveRoad::insertUniqueFromRoadIds, this, _1, _2, _3);
            insertToSection = std::bind(&OpenDriveRoad::insertUniqueToSectionIds, this, _1, _2, _3);
            insertFromSection = std::bind(&OpenDriveRoad::insertUniqueFromSectionIds, this, _1, _2, _3);
        }
        else
        {
            insertToRoad = std::bind(&OpenDriveRoad::insertUniqueFromRoadIds, this, _1, _2, _3);
            insertFromRoad = std::bind(&OpenDriveRoad::insertUniqueToRoadIds, this, _1, _2, _3);
            insertToSection = std::bind(&OpenDriveRoad::insertUniqueFromSectionIds, this, _1, _2, _3);
            insertFromSection = std::bind(&OpenDriveRoad::insertUniqueToSectionIds, this, _1, _2, _3);
        }
        // laneSection part
        for (unsigned int i = 0; i < sections_.size(); i++)
        {
            // laneSection pointer
            RoadSection *p_sec = &sections_.at(i);
            // left lanes set
            for (unsigned int lj = 0; lj < p_sec->left_lanes_.size(); lj++)
            {
                //output lane struct
                PlannerHNS::Lane op_lane;
                //one left lane pointer
                OpenDriveLane *p_l_l = &p_sec->left_lanes_.at(lj);
                if (p_l_l->type_ == DRIVING_LANE)
                {
                    op_lane.id = (this->id_ * 100) + p_sec->id_ * 10 + p_l_l->id_ + 10;
                    // aw lane num:lane id
                    op_lane.num = p_l_l->id_;
                    op_lane.roadId = id_;
                    op_lane.length = p_l_l->length_;
                    op_lane.speed = this->max_speed;
                    op_lane.type = p_l_l->type_;
                    op_lane.laneSectionId = p_sec->id_;
                    op_lane.roadtype_ = this->road_type_;

                    // based on to road, insert successor lane id
                    if (i == 0)
                    {
                        insertToRoad(p_sec->id_, p_l_l->id_, op_lane);
                    }
                    // based on lanesection, insert successor lane id
                    else
                    {
                        insertToSection(p_sec->id_ - 1, p_l_l, op_lane);
                    }
                    // based on from road, insert predecessor lane id
                    if (i == sections_.size() - 1)
                    {
                        insertFromRoad(p_sec->id_, p_l_l->id_, op_lane);
                    }
                    else
                    {
                        insertFromSection(p_sec->id_ + 1, p_l_l, op_lane);
                    }
                    lanes_list.push_back(op_lane);
                }
            }
            // right lanes set
            for (unsigned int rj = 0; rj < p_sec->right_lanes_.size(); rj++)
            {
                PlannerHNS::Lane op_lane;
                OpenDriveLane *p_r_l = &p_sec->right_lanes_.at(rj);
                if (p_r_l->type_ == DRIVING_LANE)
                {
                    op_lane.id = (this->id_ * 100) + p_sec->id_ * 10 + p_r_l->id_ + 10;
                    op_lane.num = p_r_l->id_;
                    op_lane.roadId = id_;
                    op_lane.length = p_r_l->length_;
                    op_lane.speed = this->max_speed;
                    op_lane.type = p_r_l->type_;
                    op_lane.laneSectionId = p_sec->id_;
                    op_lane.roadtype_ = this->road_type_;

                    if (i == 0)
                    {
                        insertFromRoad(p_sec->id_, p_r_l->id_, op_lane);
                    }
                    else
                    {
                        insertFromSection(p_sec->id_ - 1, p_r_l, op_lane);
                    }
                    if (i == sections_.size() - 1)
                    {
                        insertToRoad(p_sec->id_, p_r_l->id_, op_lane);
                    }
                    else
                    {
                        insertToSection(p_sec->id_ + 1, p_r_l, op_lane);
                    }

                    lanes_list.push_back(op_lane);
                }
            }
        }
    }

    // check lane points in the same position
    void OpenDriveRoad::fixRedundantPointsLanes(PlannerHNS::Lane &_lane)
    {
        for (int ip = 1; ip < _lane.points.size(); ip++)
        {
            PlannerHNS::WayPoint *p1 = &_lane.points.at(ip - 1);
            PlannerHNS::WayPoint *p2 = &_lane.points.at(ip);
            PlannerHNS::WayPoint *p3 = nullptr;
            if (ip + 1 < _lane.points.size())
                p3 = &_lane.points.at(ip + 1);

            double d = hypot(p2->pos.y - p1->pos.y, p2->pos.x - p1->pos.x);
            if (d < g_epsilon)
            {
                p1->toIds = p2->toIds;
                if (p3 != nullptr)
                    p3->fromIds = p2->fromIds;

                _lane.points.erase(_lane.points.begin() + ip);
                ip--;

                // std::cout << "Fixed Redundant Points for Lane:" << _lane.id << ", Current: "
                // 		  << ip << ", Size: " << _lane.points.size() << std::endl;
            }
        }
    }
    
    // input: single road reference centerline point/lane list/lanesection/1
    // output:left and right lane id arrey
    std::fstream hh;
    std::fstream roi_file;
    std::fstream speed_file;
    void OpenDriveRoad::createSectionPoints(const RoadCenterInfo &ref_info,
                                            std::vector<PlannerHNS::Lane> &lanes_list,
                                            RoadSection *p_sec,
                                            int &wp_id_seq,
                                            std::vector<int> &left_lane_ids,
                                            std::vector<int> &right_lane_ids)
    {
        if (p_sec == nullptr)
            return;
        std::string inu;
        inu = "/home/chen/log/roi/roi_opendrive_" + std::to_string(id_) + ".csv";
        roi_file.open(inu, std::ios::app);
        if (append_0 == true)
        {
            hh.open("/home/chen/log/lane_center_line.csv", std::ios::app);
            speed_file.open("/home/iair/log/speed.csv", std::ios::app);
        }
        else
        {
            append_0 = true;
            hh.open("/home/chen/log/lane_center_line.csv", std::ios::out);
            speed_file.open("/home/iair/log/speed.csv", std::ios::out);
        }

        double accum_offset_width = 0;
        int frist_driving_lane = 0;
        // 施工占道打点
        bool occupation_change_flag = false;
        if (this->id_ == 42 ||
            this->id_ == 89 ||
            this->id_ == 72||
            this->id_ == 65||
            this->id_ == 519||
            this->id_ == 520)
        {
            occupation_change_flag = true;
        }
        //for every left lane
        for (unsigned int lj = 0; lj < p_sec->left_lanes_.size(); lj++)
        {
            OpenDriveLane *p_l_l = &p_sec->left_lanes_.at(lj);
            // std::cout<<lj<<" id "<<p_l_l->id_<<std::endl;
            double lane_width = p_l_l->getLaneWidth(ref_info.ds_ - p_sec->s_);
            if (p_l_l->type_ == DRIVING_LANE)
            {
                auto right_road_mark = p_l_l->getLaneMark(ref_info.ds_ - p_sec->s_);
                auto left_road_mark = SOLID_LINE_MARK;
                if (frist_driving_lane == 0)
                {
                    frist_driving_lane++;
                    OpenDriveLane *center_lane = &p_sec->center_lane_.at(0);
                    left_road_mark = center_lane->getLaneMark(ref_info.ds_ - p_sec->s_);
                }
                else
                {
                    OpenDriveLane *right_lane = &p_sec->left_lanes_.at(lj - 1);
                    left_road_mark = right_lane->getLaneMark(ref_info.ds_ - p_sec->s_);
                }

                int combined_lane_id = (this->id_ * 100) + p_sec->id_ * 10 + p_l_l->id_ + 10;
                PlannerHNS::Lane *p_op_lane = getLaneById(combined_lane_id, lanes_list);
                if (p_op_lane != nullptr)
                {
                    PlannerHNS::WayPoint p(0, 0, 0, 0);
                    double center_point_margin = accum_offset_width + (lane_width / 2.0);

                    if (lane_width > g_epsilon)
                    {
                        // create waypoint
                        p.pos = ref_info.center_p_;
                        double a = p.pos.a + M_PI_2;
                        p.pos.x += center_point_margin * cos(a);
                        p.pos.y += center_point_margin * sin(a);
                        p.laneWidth = lane_width;
                        p.id = wp_id_seq++;
                        p.v = p_l_l->speed_;

                        speed_file << "road id, " << this->id_ << "lane id, " << p_l_l->id_ << "speed, " << p_l_l->speed_ << std::endl;
                        p.enable_left_change = getLaneChangeFlag(left_road_mark);
                        p.enable_right_change = getLaneChangeFlag(right_road_mark);
                        p.enable_road_occupation_left = occupation_change_flag;

                        p_op_lane->width = lane_width;
                        p_op_lane->points.push_back(p);

                        accum_offset_width += lane_width;
                        // 将本段lanesection 中的lane id 存入
                        if (!exists(left_lane_ids, combined_lane_id))
                        {
                            left_lane_ids.push_back(combined_lane_id);
                        }
                    }
                    hh << p.pos.x << "," << p.pos.y << std::endl;
                    if (lj == p_sec->left_lanes_.size() - 1)
                    {
                        PlannerHNS::WayPoint ppp(0, 0, 0, 0);
                        ppp.pos = p.pos;
                        double a = p.pos.a + M_PI_2;
                        ppp.pos.x += lane_width / 2.0 * cos(a);
                        ppp.pos.y += lane_width / 2.0 * sin(a);

                        roi_file << ppp.pos.x << "," << ppp.pos.y << std::endl;
                    }
                }
                else
                {
                    std::cout << " >>>>> Can't Find Left Lane:  " << combined_lane_id << std::endl;
                }
            }
            else
            {
                accum_offset_width += lane_width;
            }
        }

        accum_offset_width = 0;
        frist_driving_lane = 0;
        for (unsigned int rj = 0; rj < p_sec->right_lanes_.size(); rj++)
        {
            OpenDriveLane *p_r_l = &p_sec->right_lanes_.at(rj);
            // std::cout<<rj<<" id "<<p_r_l->id_<<std::endl;
            double lane_width = p_r_l->getLaneWidth(ref_info.ds_ - p_sec->s_);

            if (p_r_l->type_ == DRIVING_LANE)
            {
                auto left_road_mark = SOLID_LINE_MARK;
                auto right_road_mark = p_r_l->getLaneMark(ref_info.ds_ - p_sec->s_);
                if (frist_driving_lane == 0)
                {
                    frist_driving_lane++;
                    OpenDriveLane *center_lane = &p_sec->center_lane_.at(0);
                    left_road_mark = center_lane->getLaneMark(ref_info.ds_ - p_sec->s_);
                }
                else
                {
                    OpenDriveLane *left_lane = &p_sec->right_lanes_.at(rj - 1);
                    left_road_mark = left_lane->getLaneMark(ref_info.ds_ - p_sec->s_);
                }

                int combined_lane_id = (this->id_ * 100) + p_sec->id_ * 10 + (p_r_l->id_ + 10);
                PlannerHNS::Lane *p_op_lane = getLaneById(combined_lane_id, lanes_list);

                if (p_op_lane != nullptr)
                {
                    PlannerHNS::WayPoint p(0, 0, 0, 0);
                    double center_point_margin = accum_offset_width + (lane_width / 2.0);

                    if (lane_width > g_epsilon)
                    {
                        p.pos = ref_info.center_p_;
                        double a = p.pos.a - M_PI_2;
                        p.pos.x += center_point_margin * cos(a);
                        p.pos.y += center_point_margin * sin(a);
                        p.laneWidth = lane_width;
                        p.id = wp_id_seq++;
                        p.v = p_r_l->speed_;

                        speed_file << "road id, " << this->id_ << "lane id, " << p_r_l->id_ << "speed, " << p_r_l->speed_ << std::endl;

                        p.enable_left_change = getLaneChangeFlag(left_road_mark);
                        p.enable_right_change = getLaneChangeFlag(right_road_mark);
                        p.enable_road_occupation_left = occupation_change_flag;

                        p_op_lane->width = lane_width;
                        p_op_lane->points.push_back(p);

                        accum_offset_width += lane_width;

                        if (!exists(right_lane_ids, combined_lane_id))
                        {
                            right_lane_ids.push_back(combined_lane_id);
                        }
                    }
                    else
                    {
                        // TODO deal lane width =0
                    }

                    hh << p.pos.x << "," << p.pos.y << std::endl;
                    if (rj == p_sec->right_lanes_.size() - 1)
                    {
                        PlannerHNS::WayPoint ppp(0, 0, 0, 0);
                        ppp.pos = p.pos;
                        double a = p.pos.a + M_PI_2;
                        ppp.pos.x += lane_width / 2.0 * cos(a);
                        ppp.pos.y += lane_width / 2.0 * sin(a);

                        roi_file << ppp.pos.x << "," << ppp.pos.y << std::endl;
                    }
                }
                else
                {
                    std::cout << " >>>>> Can't Find Right Lane:  " << combined_lane_id << std::endl;
                }
            }
            else
            {
                accum_offset_width += lane_width;
            }
        }
        hh.close();
        speed_file.close();
        roi_file.close();
    }

    void OpenDriveRoad::createAdjecntLanes(std::vector<PlannerHNS::Lane> &lanes_list)
    {
        // laneSection part
        for (unsigned int i = 0; i < sections_.size(); i++)
        {
            // laneSection pointer
            RoadSection *p_sec = &sections_.at(i);
            // left lanes set
            for (unsigned int lj = 0; lj < p_sec->left_lanes_.size(); lj++)
            {
                //one left lane pointer
                OpenDriveLane *p_l_l = &p_sec->left_lanes_.at(lj);
                if (p_l_l->type_ == DRIVING_LANE)
                {
                    int combined_lane_id = (this->id_ * 100) + p_sec->id_ * 10 + (p_l_l->id_ + 10);
                    PlannerHNS::Lane *p_op_lane = getLaneById(combined_lane_id, lanes_list);

                    if (lj - 1 >= 0 && p_op_lane->LeftLaneId == 0)
                    {
                        for (int jj = lj - 1; jj >= 0; jj--)
                        {
                            OpenDriveLane *p_lane = &p_sec->left_lanes_.at(jj);
                            if (p_lane->type_ == DRIVING_LANE)
                            {

                                p_op_lane->LeftLaneId =
                                    (this->id_ * 100) + p_sec->id_ * 10 + (p_lane->id_ + 10);
                                break;
                            }
                        }
                    }

                    if (lj + 1 <= p_sec->left_lanes_.size() && p_op_lane->RightLaneId == 0)
                    {
                        for (unsigned int jj = lj + 1; jj < p_sec->left_lanes_.size(); jj++)
                        {
                            OpenDriveLane *p_lane = &p_sec->left_lanes_.at(jj);
                            if (p_lane->type_ == DRIVING_LANE)
                            {
                                p_op_lane->RightLaneId =
                                    (this->id_ * 100) + p_sec->id_ * 10 + (p_lane->id_ + 10);
                                break;
                            }
                        }
                    }
                }
            }
            // right lanes set
            for (unsigned int rj = 0; rj < p_sec->right_lanes_.size(); rj++)
            {
                OpenDriveLane *p_r_l = &p_sec->right_lanes_.at(rj);
                if (p_r_l->type_ == DRIVING_LANE)
                {
                    int combined_lane_id = (this->id_ * 100) + p_sec->id_ * 10 + p_r_l->id_ + 10;
                    PlannerHNS::Lane *p_op_lane = getLaneById(combined_lane_id, lanes_list);
                    if (rj - 1 >= 0 && p_op_lane->LeftLaneId == 0)
                    {
                        for (int jj = rj - 1; jj >= 0; jj--)
                        {
                            OpenDriveLane *p_lane = &p_sec->right_lanes_.at(jj);
                            if (p_lane->type_ == DRIVING_LANE)
                            {
                                p_op_lane->LeftLaneId =
                                    (this->id_ * 100) + p_sec->id_ * 10 + p_lane->id_ + 10;
                                break;
                            }
                        }
                    }

                    if (rj + 1 <= p_sec->right_lanes_.size() && p_op_lane->RightLaneId == 0)
                    {
                        for (unsigned int jj = rj + 1; jj < p_sec->right_lanes_.size(); jj++)
                        {
                            OpenDriveLane *p_lane = &p_sec->right_lanes_.at(jj);
                            if (p_lane->type_ == DRIVING_LANE)
                            {
                                p_op_lane->RightLaneId =
                                    (this->id_ * 100) + p_sec->id_ * 10 + p_lane->id_ + 10;
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    void OpenDriveRoad::getRoadLanes(std::vector<PlannerHNS::Lane> &lanes_list, double resolution)
    {
        std::vector<RoadCenterInfo> ref_info;
        // creat reference center line points only based on geometry
        createRoadCenterInfo(ref_info, resolution);
        // create all lanes in this road
        createRoadLanes(lanes_list);

        createAdjecntLanes(lanes_list);

        // waypoints id in lane add 1 every time from number 1
        static int wp_id_seq = 1;
        bool flag = false;
        std::vector<int> left_lane_ids, right_lane_ids;
        //for evey reference line point
        // create center points for every lane which perpendicular reference line point
        for (unsigned int i = 0; i < ref_info.size(); i++)
        {
            RoadSection *p_sec = getMatchingSection(ref_info.at(i).ds_);
            if (p_sec != nullptr)
            {
                createSectionPoints(ref_info.at(i),
                                    lanes_list,
                                    p_sec,
                                    wp_id_seq,
                                    left_lane_ids, right_lane_ids);
                flag = false;
            }
        }

        // if keep right to drive, reverse left lane points
        if (keep_right_)
        {
            for (auto id : left_lane_ids)
            {
                PlannerHNS::Lane *p_op_lane = getLaneById(id, lanes_list);
                if (p_op_lane != nullptr)
                {
                    std::reverse(p_op_lane->points.begin(), p_op_lane->points.end());
                }
            }
        }
        else
        {
            for (auto id : right_lane_ids)
            {
                PlannerHNS::Lane *p_op_lane = getLaneById(id, lanes_list);
                if (p_op_lane != nullptr)
                {
                    std::reverse(p_op_lane->points.begin(), p_op_lane->points.end());
                }
            }
        }

        for (unsigned int i = 0; i < lanes_list.size(); i++)
        {
            fixRedundantPointsLanes(lanes_list.at(i));
        }
    }

    std::vector<Connection> OpenDriveRoad::getLastSectionConnections(OpenDriveRoad *_p_successor_road)
    {
        std::vector<Connection> connections_list;
        Connection conn;

        if (_p_successor_road == nullptr)
        {
            return connections_list;
        }

        //for right lanes
        RoadSection *p_l_sec = getLastSection();
        if (p_l_sec != nullptr)
        {
            conn.incoming_road_ = id_;
            conn.incoming_section_ = p_l_sec->id_;
            conn.outgoing_road_ = _p_successor_road->id_;
            conn.outgoing_section_ = 0;

            for (unsigned int i = 0; i < p_l_sec->right_lanes_.size(); i++)
            {
                if (p_l_sec->right_lanes_.at(i).to_lane_.size() > 0)
                {
                    int _to_id = p_l_sec->right_lanes_.at(i).to_lane_.at(0).to_lane_id;
                    conn.lane_links.push_back(std::make_pair(p_l_sec->right_lanes_.at(i).id_, _to_id));
                }

                if (conn.lane_links.size() > 0)
                {
                    connections_list.push_back(conn);
                    conn.lane_links.clear();
                }
            }
        }

        //for left lanes
        if (p_l_sec != nullptr)
        {
            conn.outgoing_road_ = id_;
            conn.outgoing_section_ = p_l_sec->id_;
            conn.incoming_road_ = _p_successor_road->id_;
            conn.incoming_section_ = 0;

            for (unsigned int i = 0; i < p_l_sec->left_lanes_.size(); i++)
            {
                if (p_l_sec->left_lanes_.at(i).from_lane_.size() > 0)
                {
                    int _from_id = p_l_sec->left_lanes_.at(i).from_lane_.at(0).from_lane_id;
                    conn.lane_links.push_back(std::make_pair(_from_id, p_l_sec->left_lanes_.at(i).id_));
                }

                if (conn.lane_links.size() > 0)
                {
                    connections_list.push_back(conn);
                    conn.lane_links.clear();
                }
            }
        }

        return connections_list;
    }

    // connect predecessor road with current road for both left and right lane
    std::vector<Connection> OpenDriveRoad::getFirstSectionConnections(OpenDriveRoad *_p_predecessor_road)
    {
        std::vector<Connection> connections_list;
        Connection conn;

        if (_p_predecessor_road == nullptr)
        {
            return connections_list;
        }

        //for right lanes
        // first laneSection in road
        RoadSection *p_f_sec = getFirstSection();
        // last lanesection in predecessor road
        RoadSection *p_prev_sec = _p_predecessor_road->getLastSection();
        // if both section exist, connect them
        if (p_prev_sec != nullptr && p_f_sec != nullptr)
        {
            conn.outgoing_road_ = this->id_;
            conn.outgoing_section_ = p_f_sec->id_;
            conn.incoming_road_ = _p_predecessor_road->id_;
            conn.incoming_section_ = p_prev_sec->id_;

            for (unsigned int i = 0; i < p_f_sec->right_lanes_.size(); i++)
            {
                if (p_f_sec->right_lanes_.at(i).from_lane_.size() > 0)
                {
                    int _from_id = p_f_sec->right_lanes_.at(i).from_lane_.at(0).from_lane_id;
                    //make pair lane id
                    conn.lane_links.push_back(std::make_pair(_from_id, p_f_sec->right_lanes_.at(i).id_));
                }

                if (conn.lane_links.size() > 0)
                {
                    connections_list.push_back(conn);
                    conn.lane_links.clear();
                }
            }
        }

        //left lanes
        if (p_prev_sec != nullptr && p_f_sec != nullptr)
        {
            conn.incoming_road_ = this->id_;
            conn.incoming_section_ = p_f_sec->id_;
            conn.outgoing_road_ = _p_predecessor_road->id_;
            conn.outgoing_section_ = p_prev_sec->id_;

            for (unsigned int i = 0; i < p_f_sec->left_lanes_.size(); i++)
            {
                if (p_f_sec->left_lanes_.at(i).to_lane_.size() > 0)
                {
                    int _to_id = p_f_sec->left_lanes_.at(i).to_lane_.at(0).to_lane_id;
                    conn.lane_links.push_back(std::make_pair(p_f_sec->left_lanes_.at(i).id_, _to_id));
                }

                if (conn.lane_links.size() > 0)
                {
                    connections_list.push_back(conn);
                    conn.lane_links.clear();
                }
            }
        }

        return connections_list;
    }

    OBJECT_TYPE OpenDriveRoad::getObjTypeFromText(const std::string &autoware_type)
    {
        if (autoware_type.compare("TRAFFIC_LIGHT"))
        {
            return TRAFFIC_LIGHT;
        }
        else if (autoware_type.compare("ROAD_SIGN"))
        {
            return ROAD_SIGN;
        }
        else if (autoware_type.compare("ROAD_MARK"))
        {
            return ROAD_MARK;
        }
        else
        {
            return UNKNOWN_OBJECT;
        }
    }

    OBJECT_TYPE OpenDriveRoad::getAutowareMainTypeFromCode(const std::string &country_code,
                                                           const std::string &type,
                                                           const std::string &sub_type)
    {
        if (p_country_signal_codes_ != nullptr)
        {
            for (unsigned int i = 0; i < p_country_signal_codes_->size(); i++)
            {
                if (p_country_signal_codes_->at(i).first.compare(country_code) == 0)
                {
                    for (unsigned int j = 0; j < p_country_signal_codes_->at(i).second.size(); j++)
                    {
                        if (p_country_signal_codes_->at(i).second.at(j).id_.compare(type) == 0 &&
                            p_country_signal_codes_->at(i).second.at(j).sub_id_.compare(sub_type) == 0)
                        {
                            return getObjTypeFromText(p_country_signal_codes_->at(i).second.at(j).gen_type_);
                        }
                    }
                }
            }
        }

        return UNKNOWN_OBJECT;
    }

    TRAFFIC_LIGHT_TYPE OpenDriveRoad::getLightTypeFromText(const std::string &autoware_type)
    {
        if (autoware_type.compare("VERTICAL_DEFAULT_LIGHT"))
        {
            return VERTICAL_DEFAULT_LIGHT;
        }
        else if (autoware_type.compare("HORIZONTAL_DEFAULTLIGHT"))
        {
            return HORIZONTAL_DEFAULTLIGHT;
        }
        else if (autoware_type.compare("PEDESTRIAN_DEFAULT_LIGHT"))
        {
            return PEDESTRIAN_DEFAULT_LIGHT;
        }
        else
        {
            return UNKNOWN_LIGHT;
        }
    }

    TRAFFIC_LIGHT_TYPE OpenDriveRoad::getAutowareLightTypeFromCode(const std::string &country_code, const std::string &type, const std::string &sub_type)
    {
        if (p_country_signal_codes_ != nullptr)
        {
            for (unsigned int i = 0; i < p_country_signal_codes_->size(); i++)
            {
                if (p_country_signal_codes_->at(i).first.compare(country_code) == 0)
                {
                    for (unsigned int j = 0; j < p_country_signal_codes_->at(i).second.size(); j++)
                    {
                        if (p_country_signal_codes_->at(i).second.at(j).id_.compare(type) == 0 &&
                            p_country_signal_codes_->at(i).second.at(j).sub_id_.compare(sub_type) == 0)
                        {
                            return getLightTypeFromText(p_country_signal_codes_->at(i).second.at(j).gen_sub_type_);
                        }
                    }
                }
            }
        }

        return UNKNOWN_LIGHT;
    }

    ROAD_MARK_TYPE OpenDriveRoad::getMarkTypeFromText(const std::string &autoware_type)
    {
        if (autoware_type.compare("STOP_LINE_MARK"))
        {
            return STOP_LINE_MARK;
        }
        else if (autoware_type.compare("WAITING_LINE_MARK"))
        {
            return WAITING_LINE_MARK;
        }
        else if (autoware_type.compare("FORWARD_DIRECTION_MARK"))
        {
            return FORWARD_DIRECTION_MARK;
        }
        else if (autoware_type.compare("LEFT_DIRECTION_MARK"))
        {
            return LEFT_DIRECTION_MARK;
        }
        else if (autoware_type.compare("RIGHT_DIRECTION_MARK"))
        {
            return RIGHT_DIRECTION_MARK;
        }
        else if (autoware_type.compare("FORWARD_LEFT_DIRECTION_MARK"))
        {
            return FORWARD_LEFT_DIRECTION_MARK;
        }
        else if (autoware_type.compare("FORWARD_RIGHT_DIRECTION_MARK"))
        {
            return FORWARD_RIGHT_DIRECTION_MARK;
        }
        else if (autoware_type.compare("ALL_DIRECTION_MARK"))
        {
            return ALL_DIRECTION_MARK;
        }
        else if (autoware_type.compare("U_TURN_DIRECTION_MARK"))
        {
            return U_TURN_DIRECTION_MARK;
        }
        else if (autoware_type.compare("NO_U_TURN_DIRECTION_MARK"))
        {
            return NO_U_TURN_DIRECTION_MARK;
        }
        else
        {
            return UNKNOWN_ROAD_MARK;
        }
    }

    ROAD_MARK_TYPE OpenDriveRoad::getAutowareRoadMarksTypeFromCode(
        const std::string &country_code,
        const std::string &type,
        const std::string &sub_type)
    {
        if (p_country_signal_codes_ != nullptr)
        {
            for (unsigned int i = 0; i < p_country_signal_codes_->size(); i++)
            {
                if (p_country_signal_codes_->at(i).first.compare(country_code) == 0)
                {
                    for (unsigned int j = 0; j < p_country_signal_codes_->at(i).second.size(); j++)
                    {
                        if (p_country_signal_codes_->at(i).second.at(j).id_.compare(type) == 0 &&
                            p_country_signal_codes_->at(i).second.at(j).sub_id_.compare(sub_type) == 0)
                        {
                            return getMarkTypeFromText(p_country_signal_codes_->at(i).second.at(j).gen_sub_type_);
                        }
                    }
                }
            }
        }

        return UNKNOWN_ROAD_MARK;
    }

    void OpenDriveRoad::getTrafficLights(
        std::unordered_map<int, PlannerHNS::TrafficLight> &all_lights)
    {
        for (unsigned int i = 0; i < road_signals_.size(); i++)
        {
            if (road_signals_.at(i).name_ == "Sign_SignalAhead")
            {
                PlannerHNS::TrafficLight tl;
                tl.id = (this->id_ * 1000) + road_signals_.at(i).id_;
                tl.roadId = this->id_;

                if (road_signals_.at(i).signal_location.compare("left") == 0)
                {
                    tl.left_or_right = 0;
                }
                else if (road_signals_.at(i).signal_location.compare("right") == 0)
                {
                    tl.left_or_right = 1;
                }

                // std::cout<<"road id "<<this->id_<<"  tl.left_or_right "<< tl.left_or_right<<std::endl;

                PlannerHNS::WayPoint p;
                if (createSingleCenterPoint(road_signals_.at(i).s_, p))
                {
                    double a = p.pos.a + M_PI_2;
                    p.pos.x += road_signals_.at(i).t_ * cos(a);
                    p.pos.y += road_signals_.at(i).t_ * sin(a);
                    tl.pos = p.pos;
                    all_lights[tl.id] = tl;
                }
            }
        }
    }

    void OpenDriveRoad::getStopLines(std::unordered_map<int, PlannerHNS::StopLine> &all_stop_lines)
    {
        for (unsigned int i = 0; i < road_signals_.size(); i++)
        {
            if (road_signals_.at(i).name_ == "StopLine")
            {
                // std::cout << "road id " << this->id_ << std::endl;
                // std::cout << "road_signals_ id " << road_signals_.at(i).id_ << std::endl;
                PlannerHNS::StopLine sl;
                sl.id = (this->id_ * 1000) + road_signals_.at(i).id_;
                sl.roadId = this->id_;
                PlannerHNS::WayPoint p;
                if (road_signals_.at(i).orientation_ == SAME_DIRECTION)
                    sl.orientation_ = 0;
                else if (road_signals_.at(i).orientation_ == OPPOSIT_DIRECTION)
                    sl.orientation_ = 1;
                else
                    sl.orientation_ = 2;

                if (createSingleCenterPoint(road_signals_.at(i).s_, p))
                {
                    double a = p.pos.a + M_PI_2;
                    p.pos.x += (road_signals_.at(i).t_) * cos(a);
                    p.pos.y += (road_signals_.at(i).t_) * sin(a);
                    sl.point = p.pos;

                    if (road_signals_.at(i).valid_lanes_ids_.size() > 0)
                    {
                        for (unsigned int j = 0; j < road_signals_.at(i).valid_lanes_ids_.size(); j++)
                        {
                            sl.laneIds.push_back(road_signals_.at(i).valid_lanes_ids_.at(j));
                            all_stop_lines[sl.id] = sl;
                        }
                    }
                    else
                    {
                        all_stop_lines[sl.id] = sl;
                    }
                }
            }
        }
    }

    void OpenDriveRoad::getCrossWalk(std::unordered_map<int, PlannerHNS::Crossing> &all_crosswalk)
    {
        for (unsigned int i = 0; i < road_objects_.size(); i++)
        {
            if (road_objects_.at(i).name_ == "Crosswalk")
            {
                PlannerHNS::Crossing cross_walk;
                cross_walk.id = (this->id_ * 1000) + road_objects_.at(i).id_;
                cross_walk.roadId = this->id_;

                if (road_objects_.at(i).corner_local.empty())
                {
                    std::cout << "corner empty" << std::endl;
                    return;
                }
                std::vector<PlannerHNS::GPSPoint> polygon_points;
                PlannerHNS::WayPoint p;
                createSingleCenterPoint(road_objects_.at(i).s_, p);
                double a = p.pos.a + M_PI_2;
                p.pos.x += road_objects_.at(i).t_ * cos(a);
                p.pos.y += road_objects_.at(i).t_ * sin(a);

                // std::cout << "road_objects_.at(i).id_ " << road_objects_.at(i).id_ << std::endl;
                // std::cout << "x " << p.pos.x << "y " << p.pos.y << "yaw " << road_objects_.at(i).yaw_
                //           << "a " << p.pos.a << std::endl;

                for (std::size_t j = 0; j < road_objects_.at(i).corner_local.size(); j++)
                {
                    PlannerHNS::GPSPoint temp_corner_point;
                    temp_corner_point.x = road_objects_.at(i).corner_local.at(j).u_;
                    temp_corner_point.y = road_objects_.at(i).corner_local.at(j).v_;
                    // std::cout << "x " << temp_corner_point.x << "y " << temp_corner_point.y << std::endl;
                    double x, y, s, t;
                    PlannerHNS::PlanningHelpers::ConvertFrameclockwise(
                        -road_objects_.at(i).yaw_, temp_corner_point.x, temp_corner_point.y, s, t);

                    PlannerHNS::PlanningHelpers::ConvertFrameclockwise(
                        -p.pos.a, s, t, x, y);
                    temp_corner_point.x = x + p.pos.x;
                    temp_corner_point.y = y + p.pos.y;
                    // std::cout << "x " << temp_corner_point.x << "y " << temp_corner_point.y << std::endl;

                    polygon_points.emplace_back(temp_corner_point);
                }

                PlannerHNS::math::Polygon2d cross_walk_polygon(polygon_points);
                cross_walk.crosswalk_box = cross_walk_polygon;

                all_crosswalk[cross_walk.id] = cross_walk;
            }
        }
    }

    void OpenDriveRoad::getParkSpot(std::unordered_map<int, PlannerHNS::ParkSpot> &all_parkspot)
    {
        for (unsigned int i = 0; i < road_objects_.size(); i++)
        {
            if (road_objects_.at(i).name_ == "parkSpace")
            {
                // std::cout << "road_objects_.at(i).id_ " << road_objects_.at(i).id_ << std::endl;

                PlannerHNS::ParkSpot park_spot;
                park_spot.id = (this->id_ * 10000) + road_objects_.at(i).id_;
                park_spot.roadId = this->id_;

                std::vector<PlannerHNS::GPSPoint> box_points;
                PlannerHNS::WayPoint p;
                createSingleCenterPoint(road_objects_.at(i).s_, p);
                double a = p.pos.a + M_PI_2;
                p.pos.x += road_objects_.at(i).t_ * cos(a);
                p.pos.y += road_objects_.at(i).t_ * sin(a);
                PlannerHNS::math::Box2d parking_box(
                    p.pos, road_objects_.at(i).yaw_ + p.pos.a, road_objects_.at(i).l_, road_objects_.at(i).w_);
                park_spot.spot_box = parking_box;

                all_parkspot[park_spot.id] = park_spot;
                // std::cout << "park_spot.id " << park_spot.id << std::endl;
            }
        }
    }

} // namespace opendrive
