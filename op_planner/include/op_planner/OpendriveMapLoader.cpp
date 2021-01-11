/* opendrive2autoware_converter_core.cpp
 *
 *  Created on: Feb 5, 2020
 *      Author: chen
 */

#include "op_planner/OpendriveMapLoader.h"

namespace opendrive
{
    // 构造函数
    OpenDriveLoader::OpenDriveLoader()
    {
    }
    //析构函数
    OpenDriveLoader::~OpenDriveLoader()
    {
    }

    void OpenDriveLoader::getFileNameInFolder(const std::string &path,
                                              std::vector<std::string> &out_list)
    {
        out_list.clear();
        DIR *dir;
        struct dirent *ent;
        dir = opendir(path.c_str());
        if (dir != NULL)
        {
            while ((ent = readdir(dir)) != NULL)
            {
                std::string str(ent->d_name);
                if (str.compare(".") != 0 && str.compare("..") != 0)
                    out_list.push_back(path + str);
            }
            closedir(dir);
        }
    }

    void OpenDriveLoader::loadCountryCods(const std::string &codes_csv_folder)
    {
        // std::cout << "codes_csv_folder.size()=" << codes_csv_folder.size() << std::endl;
        // std::cout <<codes_csv_folder<< std::endl;
        if (codes_csv_folder.size() > 0)
        {
            std::vector<std::string> files_names;

            getFileNameInFolder(codes_csv_folder, files_names);
            country_signal_codes_.clear();
            for (unsigned int i = 0; i < files_names.size(); i++)
            {
                CSV_Reader reader(files_names.at(i));
                int i_ext_dot = files_names.at(i).find('.');
                int i_last_folder = files_names.at(i).find_last_of('/') + 1;
                std::string country_code =
                    files_names.at(i).substr(i_last_folder, i_ext_dot - i_last_folder);

                std::cout << "Reading Country Codes file: " << country_code << std::endl;

                std::vector<CSV_Reader::LINE_DATA> country_data;
                reader.ReadAllData(country_data);

                country_signal_codes_.push_back(std::make_pair(country_code, country_data));
            }
        }
        else
        {
            std::cout << "No Signs will be recognized, no Country Code files at: " << codes_csv_folder << std::endl;
        }
    }

    // 参数输入赋值src_path
    // 根据国家规范定义文件格式country_codes_path
    // 目标文件目录dst_path
    // 路点间距wp_res
    // 右道优先还是左道优keep_right
    // loadOpenDRIVE(src_path, country_codes_path, map, wp_res, keep_right);
    /*
    解析opendrive地图的基本流程；
        1.通过loadOpenDRIVE()加载目标文件,解析出road信息
        2.通过connectRoads()连接道路
        3.解析road里相应的车道lane信息
        4.通过linkWayPoints()连接lane中waypoint
    */
    void OpenDriveLoader::loadOpenDRIVE(const std::string &xodr_file,
                                        const std::string &codes_folder,
                                        PlannerHNS::RoadNetwork &map,
                                        double resolution,
                                        bool keep_right)
    {
        keep_right_ = keep_right;

        std::ifstream f(xodr_file.c_str());
        if (!f.good())
        {
            std::cout << "Can't Open OpenDRIVE Map File: (" << xodr_file << ")" << std::endl;
            return;
        }

        std::cout << " >> Loading OpenDRIVE Map file ... " << std::endl;

        // 异常处理
        TiXmlDocument doc(xodr_file);
        try
        {
            doc.LoadFile();
        }
        catch (std::exception &e)
        {
            std::cout << "OpenDRIVE Custom Reader Error, Can't Load .xodr File, path is: " << xodr_file << std::endl;
            std::cout << e.what() << std::endl;
            return;
        }

        loadCountryCods(codes_folder);

        std::cout << " >> Reading Header Data from OpenDRIVE map file ... " << std::endl;
        std::vector<TiXmlElement *> elements;
        XmlHelpers::findFirstElement("header", doc.FirstChildElement(), elements);//取出header标签中的元素
        std::cout << "header Num:" << elements.size() << std::endl;

        std::cout << " >> Reading Data from OpenDRIVE map file ... " << std::endl;
        elements.clear();
        XmlHelpers::findElements("road", doc.FirstChildElement(), elements);//取出road标签中的元素
        std::cout << "Final Results Roads, Num:" << elements.size() << std::endl;

        roads_list_.clear();
        
        //通过调用OpenDriveRoad这个class里的OpenDriveRoad函数来将road信息解析出来
        for (unsigned int i = 0; i < elements.size(); i++)
        {
            roads_list_.push_back(OpenDriveRoad(elements.at(i), &country_signal_codes_, keep_right_));
        }

        elements.clear();
        XmlHelpers::findElements("junction", doc.FirstChildElement(), elements);//取出junction标签中的元素
        std::cout << "Final Results Junctions, Num:" << elements.size() << std::endl;

        junctions_list_.clear();
        for (unsigned int i = 0; i < elements.size(); i++)
        {
            junctions_list_.push_back(Junction(elements.at(i)));
        }

        //Connect Roads and junctions
        connectRoads();
        std::cout << "Finish reading opendrive xdor file and linking road network.. " << std::endl;
        // coutRoadLinks();
        // RoadSegment include id,lanes,boundary
        PlannerHNS::RoadSegment segment;
        segment.id = 1;
        map.roadSegments.push_back(segment);
        // use discrete points to build  lanes(road list)
        getMapLanes(map.roadSegments.at(0).Lanes, resolution);
        std::cout << "Finish Extracting Lanes: " << map.roadSegments.at(0).Lanes.size() << std::endl;
        std::cout << "Finish Creating center line waypoints of all lanes" << std::endl;
        getTrafficLights(map.trafficLight_map);
        std::cout << "Finish Extracting Traffic lights: " << map.trafficLight_map.size() << std::endl;
        getStopLines(map.stopLine_map);
        std::cout << "Finish Extracting stop lines: " << map.stopLine_map.size() << std::endl;
        getCrossings(map.crossing_map);
        std::cout << "Finish Extracting Crossings: " << map.crossing_map.size() << std::endl;
        getParkSpot(map.ParkSpot_map);
        std::cout << "Finish Extracting ParkSpot: " << map.ParkSpot_map.size() << std::endl;

        linkWayPoints(map);
        std::cout << "Finish Creating discrete directed weighted graph ... " << std::endl;
        FindAdjacentLanes(map);
        std::cout << " >>Finish lingking left and right waypoints ... " << std::endl;
        LinkMissingBranchingWayPointsV2(map);
        std::cout << " >>Finish Linking Front waypoints... " << std::endl;
        LinkStopLines(map);
        LinkTrafficLights(map);
        std::cout << " >>Finish Link StopLines and Traffic Lights... " << std::endl;
        LinkCrossWalks(map);
        std::cout << " >>Finish Link Crosswalks... " << std::endl;
        // LinkParkSpot(map);
        // std::cout << " >>Finish Link ParkingSpace... " << std::endl;
        findAsterArea(map);
        std::cout << " >>Finish Find Aster... " << std::endl;
        saveMapData(map);
        std::cout << " >>Finish saving lane data... " << std::endl;
    }

    void OpenDriveLoader::LinkMissingBranchingWayPointsV2(PlannerHNS::RoadNetwork &map)
    {
        for (unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
        {
            // for every lanes in map
            for (unsigned int i = 0; i < map.roadSegments.at(rs).Lanes.size(); i++)
            {
                PlannerHNS::Lane *pLane = &map.roadSegments.at(rs).Lanes.at(i);

                for (unsigned int p = 0; p < pLane->points.size(); p++)
                {
                    PlannerHNS::WayPoint *pWP = &pLane->points.at(p);

                    if (p + 1 == pLane->points.size()) // Last Point in Lane
                    {
                        // push every front waypoints to first waypoints in lane
                        for (unsigned int j = 0; j < pLane->toLanes.size(); j++)
                        {
                            if (pLane->toLanes.at(j)->points.size() > 0)
                                pWP->pFronts.push_back(&pLane->toLanes.at(j)->points.at(0));
                        }
                    }
                    else
                    {
                        // waypoint in lane only have one successor waypoint
                        if (pWP->toIds.size() > 1)
                        {
                            std::cout << "Error Error Erro ! Lane: " << pWP->laneId
                                      << std::endl;
                        }
                        else
                        {
                            pWP->pFronts.push_back(&pLane->points.at(p + 1));
                        }
                    }
                }
            }
        }
    }

    //未使用
    void OpenDriveLoader::coutRoadLinks(void)
    {
        for (unsigned int i = 0; i < roads_list_.size(); i++)
        {
            std::cout << "Road ID: " << roads_list_.at(i).id_ << std::endl;
            std::cout << "From: ";
            for (unsigned int j = 0; j < roads_list_.at(i).from_roads_.size(); j++)
            {
                std::cout << "(" << roads_list_.at(i).from_roads_.at(j).incoming_road_ << "|";
                for (unsigned int k = 0; k < roads_list_.at(i).from_roads_.at(j).lane_links.size(); k++)
                {
                    std::cout << roads_list_.at(i).from_roads_.at(j).lane_links.at(k).first << ", "
                              << roads_list_.at(i).from_roads_.at(j).lane_links.at(k).second << " ; ";
                }
                std::cout << ")";
            }

            std::cout << std::endl;
            std::cout << "To : ";

            for (unsigned int j = 0; j < roads_list_.at(i).to_roads_.size(); j++)
            {
                std::cout << "(" << roads_list_.at(i).to_roads_.at(j).outgoing_road_ << "|";
                for (unsigned int k = 0; k < roads_list_.at(i).to_roads_.at(j).lane_links.size(); k++)
                {
                    std::cout << roads_list_.at(i).to_roads_.at(j).lane_links.at(k).first << ", "
                              << roads_list_.at(i).to_roads_.at(j).lane_links.at(k).second << " ; ";
                }
                std::cout << ")";
            }

            std::cout << std::endl
                      << std::endl;
        }
    }
    //未使用
    std::vector<OpenDriveRoad *> OpenDriveLoader::getRoadsBySuccId(int _id)
    {
        std::vector<OpenDriveRoad *> _ret_list;
        for (unsigned int i = 0; i < roads_list_.size(); i++)
        {
            for (unsigned int j = 0; j < roads_list_.at(i).successor_road_.size(); j++)
            {
                if (roads_list_.at(i).successor_road_.at(j).to_road_id_ == _id)
                {
                    _ret_list.push_back(&roads_list_.at(i));
                }
            }
        }

        return _ret_list;
    }
    //未使用
    std::vector<OpenDriveRoad *> OpenDriveLoader::getRoadsByPredId(int _id)
    {
        std::vector<OpenDriveRoad *> _ret_list;
        for (unsigned int i = 0; i < roads_list_.size(); i++)
        {
            for (unsigned int j = 0; j < roads_list_.at(i).predecessor_road_.size(); j++)
            {
                if (roads_list_.at(i).predecessor_road_.at(j).from_road_id_ == _id)
                {
                    _ret_list.push_back(&roads_list_.at(i));
                }
            }
        }

        return _ret_list;
    }

    OpenDriveRoad *OpenDriveLoader::getRoadById(int _id)
    {
        for (unsigned int i = 0; i < roads_list_.size(); i++)
        {
            if (roads_list_.at(i).id_ == _id)
            {
                return &roads_list_.at(i);
            }
        }

        return nullptr;
    }

    Junction *OpenDriveLoader::getJunctionById(int _id)
    {
        for (unsigned int i = 0; i < junctions_list_.size(); i++)
        {
            if (junctions_list_.at(i).id_ == _id)
            {
                return &junctions_list_.at(i);
            }
        }

        return nullptr;
    }

    //连接road之间的逻辑
    void OpenDriveLoader::connectRoads()
    {
        int size = 0;
        LINK_TYPE lt = ROAD_LINK;
        // for every road
        // --------------------insert to roads and from roads（road与road相连）-----------------
        for (unsigned int i = 0; i < roads_list_.size(); i++)
        {
            // deal with predecessor
            size = roads_list_.at(i).predecessor_road_.size();
            if (size > 0)
            {
                // every road only has one predecessor and one successor
                // predecessor or successor is road or junction
                lt = roads_list_.at(i).predecessor_road_.at(0).link_type_;
                //connect normal roads , junctions will be handeled alone
                if (lt == ROAD_LINK)
                {
                    // only exist one predecessor road id
                    OpenDriveRoad *p_pre_road =
                        getRoadById(roads_list_.at(i).predecessor_road_.at(0).from_road_id_);
                    if (p_pre_road != nullptr)
                    {
                        // return both left and right lane connection
                        std::vector<Connection> pre_conn_list =
                            roads_list_.at(i).getFirstSectionConnections(p_pre_road);
                        for (unsigned k = 0; k < pre_conn_list.size(); k++)
                        {
                            // every road only has one predecessor and one successor!!!!!
                            // for right lanes
                            if (pre_conn_list.at(k).outgoing_road_ == roads_list_.at(i).id_)
                            {
                                roads_list_.at(i).insertUniqueFromConnection(pre_conn_list.at(k));
                            }
                            // for left lanes
                            else if (pre_conn_list.at(k).incoming_road_ == roads_list_.at(i).id_)
                            {
                                roads_list_.at(i).insertUniqueToConnection(pre_conn_list.at(k));
                            }
                        }
                    }
                }
            }
            // deal with successor
            size = roads_list_.at(i).successor_road_.size();
            if (size > 0)
            {
                lt = roads_list_.at(i).successor_road_.at(0).link_type_;
                if (lt == ROAD_LINK)
                {
                    OpenDriveRoad *p_suc_road =
                        getRoadById(roads_list_.at(i).successor_road_.at(0).to_road_id_);
                    if (p_suc_road != nullptr)
                    {
                        std::vector<Connection> suc_conn_list =
                            roads_list_.at(i).getLastSectionConnections(p_suc_road);

                        for (unsigned k = 0; k < suc_conn_list.size(); k++)
                        {
                            // for left lanes
                            if (suc_conn_list.at(k).outgoing_road_ == roads_list_.at(i).id_)
                            {
                                roads_list_.at(i).insertUniqueFromConnection(suc_conn_list.at(k));
                            }
                            // for right lanes
                            else if (suc_conn_list.at(k).incoming_road_ == roads_list_.at(i).id_)
                            {
                                roads_list_.at(i).insertUniqueToConnection(suc_conn_list.at(k));
                            }
                        }
                    }
                }
            }
        }
        // only Link Junctions for road link type is junction（road与junction相连）
        for (unsigned int i = 0; i < roads_list_.size(); i++)
        {
            size = roads_list_.at(i).predecessor_road_.size();
            //处理predecessor
            if (size > 0)
            {
                lt = roads_list_.at(i).predecessor_road_.at(0).link_type_;
                if (lt == JUNCTION_LINK)
                {
                    // get predecessor junction
                    Junction *p_junction =
                        getJunctionById(roads_list_.at(i).predecessor_road_.at(0).from_road_id_);
                    if (p_junction != nullptr)
                    {
                        //get connection which include this road as incoming road
                        for (const auto junction_connection :
                             p_junction->getConnectionsByRoadId(roads_list_.at(i).id_))//范围循环
                        {
                            // get connectingRoad road
                            OpenDriveRoad *incoming_road =
                                getRoadById(junction_connection.outgoing_road_);
                            if (incoming_road == nullptr)
                                continue;

                            RoadSection *outgoing_section = roads_list_.at(i).getLastSection();
                            RoadSection *incoming_section = nullptr;
                            if (junction_connection.contact_point_ == "end")
                                incoming_section = incoming_road->getLastSection();
                            else
                                incoming_section = incoming_road->getFirstSection();

                            Connection connection;
                            connection.incoming_road_ = junction_connection.outgoing_road_;
                            connection.outgoing_road_ = roads_list_.at(i).id_;
                            if (incoming_section != nullptr)
                                connection.incoming_section_ = incoming_section->id_;
                            if (outgoing_section != nullptr)
                                connection.outgoing_section_ =
                                    roads_list_.at(i).getLastSection()->id_;
                            connection.lane_links = junction_connection.lane_links;

                            if (!connection.lane_links.empty())
                            {
                                if (connection.lane_links.size() == 1)
                                {
                                    if ((keep_right_ && connection.lane_links.at(0).first > 0) ||
                                        (!keep_right_ && connection.lane_links.at(0).first < 0))
                                    {
                                        connection.flipRoad();
                                        roads_list_.at(i).insertUniqueToConnection(connection);
                                    }
                                    else
                                    {
                                        roads_list_.at(i).insertUniqueFromConnection(connection);
                                    }
                                }
                                else
                                {
                                    bool bReady = false;
                                    unsigned int index = 0;
                                    int pre_sign = 0;
                                    int cur_sign = 0;

                                    for (unsigned int i = 1; i < connection.lane_links.size(); i++)
                                    {
                                        cur_sign = sign(connection.lane_links.at(i).first);
                                        pre_sign = sign(connection.lane_links.at(i - 1).first);
                                        if (cur_sign != pre_sign)
                                        {
                                            bReady = true;
                                            index = i;
                                            break;
                                        }
                                    }

                                    if (bReady)
                                    {
                                        std::vector<Connection> connection_list;
                                        Connection conn;
                                        conn = connection;

                                        conn.lane_links.clear();
                                        // std::cout << "road id" << roads_list_.at(i).id_ << std::endl;
                                        // std::cout<<"index= "<<index<<std::endl;
                                        for (unsigned int i = 0; i < index; i++)
                                        {
                                            conn.lane_links.push_back(connection.lane_links.at(i));
                                        }
                                        connection_list.push_back(conn);
                                        conn.lane_links.clear();
                                        for (unsigned int i = index; i < connection.lane_links.size(); i++)
                                        {
                                            conn.lane_links.push_back(connection.lane_links.at(i));
                                        }
                                        connection_list.push_back(conn);

                                        for (unsigned k = 0; k < connection_list.size(); k++)
                                        {

                                            //flip appropriate lane depending on keep_right flag
                                            // if from in lane link is positive
                                            if ((keep_right_ && connection_list.at(k).lane_links.at(0).first > 0) ||
                                                (!keep_right_ && connection_list.at(k).lane_links.at(0).first < 0))
                                            {

                                                connection_list.at(k).flipRoad();
                                                roads_list_.at(i).insertUniqueToConnection(connection_list.at(k));
                                            }
                                            else
                                            {
                                                roads_list_.at(i).insertUniqueFromConnection(connection_list.at(k));
                                            }
                                        }
                                    }
                                    else
                                    {
                                        if ((keep_right_ && connection.lane_links.at(0).first > 0) ||
                                            (!keep_right_ && connection.lane_links.at(0).first < 0))
                                        {
                                            connection.flipRoad();
                                            roads_list_.at(i).insertUniqueToConnection(connection);
                                        }
                                        else
                                        {
                                            roads_list_.at(i).insertUniqueFromConnection(connection);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            size = roads_list_.at(i).successor_road_.size();
            //处理successor
            if (size > 0)
            {
                // the successor of the road is a junction
                // the successor means the contactpoint is the end of the road
                // so the road is the predecessor of the junction
                lt = roads_list_.at(i).successor_road_.at(0).link_type_;
                if (lt == JUNCTION_LINK)
                {
                    Junction *p_junction =
                        getJunctionById(roads_list_.at(i).successor_road_.at(0).to_road_id_);
                    if (p_junction != nullptr)
                    {
                        for (const auto junction_connection : p_junction->getConnectionsByRoadId(roads_list_.at(i).id_))
                        {
                            OpenDriveRoad *outgoing_road =
                                getRoadById(junction_connection.outgoing_road_);
                            if (outgoing_road == nullptr)
                                continue;

                            RoadSection *incoming_section = roads_list_.at(i).getLastSection();
                            RoadSection *outgoing_section = nullptr;
                            if (junction_connection.contact_point_ == "end")
                                outgoing_section = outgoing_road->getLastSection();
                            else
                                outgoing_section = outgoing_road->getFirstSection();
                            if (incoming_section == nullptr || outgoing_section == nullptr)
                                continue;

                            Connection connection;
                            connection.incoming_road_ = roads_list_.at(i).id_;
                            connection.outgoing_road_ = junction_connection.outgoing_road_;
                            connection.incoming_section_ = incoming_section->id_;
                            connection.outgoing_section_ = outgoing_section->id_;
                            connection.lane_links = junction_connection.lane_links;

                            //flip appropriate lane depending on keep_right flag
                            if (!connection.lane_links.empty())
                            {
                                if (connection.lane_links.size() == 1)
                                {
                                    if ((keep_right_ && connection.lane_links.at(0).first > 0) ||
                                        (!keep_right_ && connection.lane_links.at(0).first < 0))
                                    {
                                        connection.flip();
                                        roads_list_.at(i).insertUniqueFromConnection(connection);
                                    }
                                    else
                                    {
                                        roads_list_.at(i).insertUniqueToConnection(connection);
                                    }
                                }
                                else
                                {
                                    bool bReady = false;
                                    unsigned int index = 0;
                                    int pre_sign = 0;
                                    int cur_sign = 0;

                                    for (unsigned int i = 1; i < connection.lane_links.size(); i++)
                                    {
                                        cur_sign = sign(connection.lane_links.at(i).first);
                                        pre_sign = sign(connection.lane_links.at(i - 1).first);
                                        if (cur_sign != pre_sign)
                                        {
                                            bReady = true;
                                            index = i;
                                            break;
                                        }
                                    }

                                    if (bReady)
                                    {
                                        std::vector<Connection> connection_list;
                                        Connection conn;
                                        conn = connection;

                                        conn.lane_links.clear();
                                        for (unsigned int i = 0; i < index; i++)
                                        {
                                            conn.lane_links.push_back(connection.lane_links.at(i));
                                        }
                                        connection_list.push_back(conn);

                                        conn.lane_links.clear();
                                        for (unsigned int i = index; i < connection.lane_links.size(); i++)
                                        {
                                            conn.lane_links.push_back(connection.lane_links.at(i));
                                        }
                                        connection_list.push_back(conn);

                                        for (unsigned k = 0; k < connection_list.size(); k++)
                                        {

                                            //flip appropriate lane depending on keep_right flag
                                            // if from in lane link is positive
                                            if ((keep_right_ && connection_list.at(k).lane_links.at(0).first > 0) ||
                                                (!keep_right_ && connection_list.at(k).lane_links.at(0).first < 0))
                                            {
                                                connection_list.at(k).flip();
                                                roads_list_.at(i).insertUniqueFromConnection(connection_list.at(k));
                                            }
                                            else
                                            {
                                                connection.flag_flip = false;
                                                roads_list_.at(i).insertUniqueToConnection(connection_list.at(k));
                                            }
                                        }
                                    }
                                    else
                                    {
                                        if ((keep_right_ && connection.lane_links.at(0).first > 0) ||
                                            (!keep_right_ && connection.lane_links.at(0).first < 0))
                                        {
                                            connection.flip();
                                            roads_list_.at(i).insertUniqueFromConnection(connection);
                                        }
                                        else
                                        {
                                            connection.flag_flip = false;
                                            roads_list_.at(i).insertUniqueToConnection(connection);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    } // namespace opendrive

    void OpenDriveLoader::getMapLanes(std::vector<PlannerHNS::Lane> &all_lanes, double resolution)
    {
        // for every road
        for (unsigned int i = 0; i < roads_list_.size(); i++)
        {
            roads_list_.at(i).getRoadLanes(all_lanes, resolution);
        }
    }

    void OpenDriveLoader::getTrafficLights(
        std::unordered_map<int, PlannerHNS::TrafficLight> &all_lights)
    {
        for (unsigned int i = 0; i < roads_list_.size(); i++)
        {
            roads_list_.at(i).getTrafficLights(all_lights);
        }
    }

    void OpenDriveLoader::getStopLines(
        std::unordered_map<int, PlannerHNS::StopLine> &all_stop_lines)
    {
        for (unsigned int i = 0; i < roads_list_.size(); i++)
        {
            roads_list_.at(i).getStopLines(all_stop_lines);
        }
    }

    void OpenDriveLoader::getCrossings(
        std::unordered_map<int, PlannerHNS::Crossing> &all_crossings)
    {
        for (unsigned int i = 0; i < roads_list_.size(); i++)
        {
            roads_list_.at(i).getCrossWalk(all_crossings);
        }
    }

    void OpenDriveLoader::getParkSpot(
        std::unordered_map<int, PlannerHNS::ParkSpot> &all_parkings)
    {
        for (unsigned int i = 0; i < roads_list_.size(); i++)
        {
            roads_list_.at(i).getParkSpot(all_parkings);
        }
    }
    
    //first link every lane
    //second link lane waypoints
    void OpenDriveLoader::linkWayPoints(PlannerHNS::RoadNetwork &map)
    {
        // for every map segment

        //???将waypoint分类到各自的lane中
        for (unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
        {
            //for every lane in map
            for (unsigned int i = 0; i < map.roadSegments.at(rs).Lanes.size(); i++)
            {
                PlannerHNS::Lane *pL = &map.roadSegments.at(rs).Lanes.at(i);
                // get every incoming lane by id
                for (unsigned int j = 0; j < pL->fromIds.size(); j++)
                {
                    for (unsigned int l = 0; l < map.roadSegments.at(rs).Lanes.size(); l++)
                    {
                        if (map.roadSegments.at(rs).Lanes.at(l).id == pL->fromIds.at(j))
                        {
                            pL->fromLanes.push_back(&map.roadSegments.at(rs).Lanes.at(l));
                        }
                    }
                }
                // get outcoming lane by id
                for (unsigned int j = 0; j < pL->toIds.size(); j++)
                {
                    for (unsigned int l = 0; l < map.roadSegments.at(rs).Lanes.size(); l++)
                    {
                        if (map.roadSegments.at(rs).Lanes.at(l).id == pL->toIds.at(j))
                        {
                            pL->toLanes.push_back(&map.roadSegments.at(rs).Lanes.at(l));
                        }
                    }
                }
                // link every points in one lane to its owner
                for (unsigned int j = 0; j < pL->points.size(); j++)
                {
                    pL->points.at(j).pLane = pL;
                }
            }
        }
        //???将每条lane中的waypoint按照前驱后继顺序连接起来
        for (unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
        {
            for (unsigned int i = 0; i < map.roadSegments.at(rs).Lanes.size(); i++)
            {
                PlannerHNS::Lane *pL = &map.roadSegments.at(rs).Lanes.at(i);

                // caculate every lane point angle and its cost
                PlannerHNS::PlanningHelpers::CalcAngleAndCost(pL->points, 0, 0);
                PlannerHNS::PlanningHelpers::FixAngleOnly(pL->points);

                for (unsigned int iwp = 0; iwp < pL->points.size(); iwp++)
                {
                    // link waypoint to next
                    if (iwp < pL->points.size() - 1)
                    {
                        pL->points.at(iwp).toIds.push_back(pL->points.at(iwp + 1).id);
                    }
                    else
                    {
                        for (unsigned int k = 0; k < pL->toLanes.size(); k++)
                        {
                            if (pL->toLanes.at(k) != nullptr && pL->toLanes.at(k)->points.size() > 0)
                            {
                                pL->points.at(iwp).toIds.push_back(pL->toLanes.at(k)->points.at(0).id);
                            }
                        }
                    }
                }
            }
        }
    }

    void OpenDriveLoader::LinkStopLines(PlannerHNS::RoadNetwork &map)
    {
        for (auto st_it : map.stopLine_map)
        {
            bool enable_lane_id = false;
            double min_distance = 1000;
            for (unsigned int i = 0; i < map.roadSegments.at(0).Lanes.size(); i++)
            {
                if (map.roadSegments.at(0).Lanes.at(i).roadId ==
                    st_it.second.roadId)
                {
                    PlannerHNS::WayPoint p;
                    p.pos = st_it.second.point;
                    auto dist_ = MatchDistance(
                        map.roadSegments.at(0).Lanes.at(i).points, p.pos.x, p.pos.y);
                    if (min_distance > dist_)
                    {
                        min_distance = dist_;
                        CopyLaneSign(
                            map.roadSegments.at(0).Lanes.at(i).num, enable_lane_id);
                    }
                }
            }

            for (unsigned int i = 0; i < map.roadSegments.at(0).Lanes.size(); i++)
            {
                if (map.roadSegments.at(0).Lanes.at(i).roadId ==
                    st_it.second.roadId)
                {
                    bool sign_1;
                    CopyLaneSign(
                        map.roadSegments.at(0).Lanes.at(i).num, sign_1);
                    if (enable_lane_id == sign_1)
                    {
                        // find waypoint nearest stopline in lane
                        PlannerHNS::WayPoint p;
                        p.pos = st_it.second.point;
                        auto index_ = MatchIndex(
                            map.roadSegments.at(0).Lanes.at(i).points, p.pos.x, p.pos.y);
                        auto *pWP =
                            &map.roadSegments.at(0).Lanes.at(i).points.at(index_);
                        // -----------------------core-----------------
                        pWP->stopLineID = st_it.second.id;
                        // break;
                    }
                }
            }
        }
    }

    void OpenDriveLoader::LinkTrafficLights(PlannerHNS::RoadNetwork &map)
    {
        for (unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
        {
            for (unsigned int i = 0; i < map.roadSegments.at(rs).Lanes.size(); i++)
            {
                for (auto tl_it : map.trafficLight_map)
                {
                    if (map.roadSegments.at(rs).Lanes.at(i).roadId == tl_it.second.roadId)
                    {
                        // find waypoint nearest traffic in lane
                        PlannerHNS::RelativeInfo info;
                        PlannerHNS::WayPoint p;
                        p.pos = tl_it.second.pos;
                        PlannerHNS::PlanningHelpers::GetRelativeInfo(
                            map.roadSegments.at(rs).Lanes.at(i).points, p, info);
                        auto *pWP =
                            &map.roadSegments.at(rs).Lanes.at(i).points.at(info.iFront);
                        // -----------------------core-----------------
                        pWP->trafficLigntID = tl_it.second.id;
                        break;
                    }
                }
            }
        }
    }

    void OpenDriveLoader::findAsterArea(PlannerHNS::RoadNetwork &map)
    {
        auto &road_segment = map.roadSegments.at(0);
        for (unsigned int i = 0; i < road_segment.Lanes.size(); i++)
        {
            if (road_segment.Lanes.at(i).roadtype_.compare("rural") == 0)
            {
                std::cout << " >>Astar  Road Id: " << road_segment.Lanes.at(i).roadId << std::endl;
                for (unsigned int isl = 0; isl < road_segment.Lanes.at(i).points.size(); isl++)
                {
                    road_segment.Lanes.at(i).points.at(isl).task = PlannerHNS::ASTAR;
                }
            }
        }
    }
    
    //cause every crosswalk in junction, only consider lanes in same junction
    void OpenDriveLoader::LinkCrossWalks(PlannerHNS::RoadNetwork &map)
    {
        auto &road_segment = map.roadSegments.at(0);
        for (auto cross_it : map.crossing_map)
        {
            auto crosswalk_roadid = cross_it.second.roadId;
            // std::cout << "cross_it.second.roadId " << cross_it.second.roadId << std::endl;
            auto crosswalk_in_junction_id =
                getRoadById(crosswalk_roadid)->junction_id_;
            for (unsigned int i = 0; i < road_segment.Lanes.size(); i++)
            {
                auto junction_id =
                    getRoadById(road_segment.Lanes.at(i).roadId)->junction_id_;

                if (crosswalk_in_junction_id == junction_id)
                {
                    for (std::size_t pt = 0; pt < road_segment.Lanes.at(i).points.size(); pt++)
                    {
                        if (cross_it.second.crosswalk_box.IsPointIn(
                                road_segment.Lanes.at(i).points.at(pt).pos))
                        {
                            // std::cout<<"road_segment.Lanes.at(i).roadId "<<road_segment.Lanes.at(i).roadId<<std::endl;
                            cross_it.second.laneIds.emplace_back(
                                road_segment.Lanes.at(i).id);
                            auto *pWP =
                                &road_segment.Lanes.at(i).points.at(pt);
                            // -----------------------core-----------------
                            pWP->crosswalkID = cross_it.second.id;
                            break;
                        }
                    }
                }
            }
        }
    }

    void OpenDriveLoader::LinkParkSpot(PlannerHNS::RoadNetwork &map)
    {
        auto &road_segment = map.roadSegments.at(0);
        for (unsigned int i = 0; i < road_segment.Lanes.size(); i++)
        {
        }
    }

    void OpenDriveLoader::FindAdjacentLanes(PlannerHNS::RoadNetwork &map)
    {
        for (unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
        {
            for (unsigned int i = 0; i < map.roadSegments.at(rs).Lanes.size(); i++)
            {
                PlannerHNS::Lane *pL = &map.roadSegments.at(rs).Lanes.at(i);
                for (unsigned int i2 = 0; i2 < map.roadSegments.at(rs).Lanes.size(); i2++)
                {
                    PlannerHNS::Lane *pL_adjacent = &map.roadSegments.at(rs).Lanes.at(i2);
                    if (pL->id == pL_adjacent->id)
                        continue;
                    if (pL->LeftLaneId == pL_adjacent->id)
                    {
                        pL->pLeftLane = pL_adjacent;

                        for (unsigned int i_internal = 0; i_internal < pL->points.size(); i_internal++)
                        {

                            if (pL->points.size() == 0)
                                continue;
                            if (pL_adjacent->points.size() == 0)
                                continue;

                            PlannerHNS::WayPoint *pWP = &pL->points.at(i_internal);
                            PlannerHNS::RelativeInfo info;
                            PlannerHNS::PlanningHelpers::GetRelativeInfo(
                                pL_adjacent->points, *pWP, info);

                            if (fabs(info.to_front_distance) < 5.0 && info.angle_diff < 90)
                            {
                                PlannerHNS::WayPoint *pWP2 = &pL_adjacent->points.at(info.iFront);

                                if (pWP->pLeft == nullptr && pWP->enable_left_change)
                                // if (pWP->pLeft == nullptr)
                                {
                                    pWP->pLeft = pWP2;
                                    pWP->LeftPointId = pWP2->id;
                                    pWP->LeftLnId = pL_adjacent->id;
                                }

                                // if (pWP2->pRight == nullptr && pWP2->enable_right_change)
                                if (pWP2->pRight == nullptr)
                                {
                                    pWP2->pRight = pWP->pLeft;
                                    pWP2->RightPointId = pWP->id;
                                    pWP2->RightLnId = pL->id;
                                }
                            }
                        }
                    }
                    else if (pL->RightLaneId == pL_adjacent->id)
                    {
                        pL->pRightLane = pL_adjacent;

                        for (unsigned int i_internal = 0; i_internal < pL->points.size(); i_internal++)
                        {
                            if (pL->points.size() == 0)
                                continue;
                            if (pL_adjacent->points.size() == 0)
                                continue;

                            PlannerHNS::WayPoint *pWP = &pL->points.at(i_internal);
                            PlannerHNS::RelativeInfo info;
                            PlannerHNS::PlanningHelpers::GetRelativeInfo(
                                pL_adjacent->points, *pWP, info);

                            if (fabs(info.to_front_distance) < 5.0 && info.angle_diff < 90)
                            {
                                PlannerHNS::WayPoint *pWP2 = &pL_adjacent->points.at(info.iFront);

                                // if (pWP->pRight == nullptr && pWP->enable_right_change)
                                if (pWP->pRight == nullptr)
                                {
                                    pWP->pRight = pWP2;
                                    pWP->RightPointId = pWP2->id;
                                    pWP->RightLnId = pL_adjacent->id;
                                }

                                if (pWP2->pLeft == nullptr && pWP2->enable_left_change)
                                // if (pWP2->pLeft == nullptr)
                                {
                                    pWP2->pLeft = pWP;
                                    pWP2->LeftPointId = pWP->id;
                                    pWP2->LeftLnId = pL->id;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    void OpenDriveLoader::saveMapData(PlannerHNS::RoadNetwork &map)
    {
        /*
        将各种属性存到了mapLine，这是个什么样的变量，后面如何用。
        */
        std::vector<std::string> roadMapLaneData;
        for (unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
        {
            for (unsigned int i = 0; i < map.roadSegments.at(rs).Lanes.size(); i++)
            {
                // for every waypoints in one lane
                PlannerHNS::Lane *pL = &map.roadSegments.at(rs).Lanes.at(i);
                if (pL != nullptr)
                {
                    std::ostringstream mapLane;

                    mapLane << pL->id << ","
                            << pL->roadId << ","
                            << pL->laneSectionId << ","
                            << pL->speed << ","
                            << pL->length << ","
                            << pL->dir << ","
                            << pL->type << ","
                            << pL->width << ","
                            << pL->num << ","
                            << pL->points.size() << ","
                            << pL->trafficlights.size() << ","
                            << pL->stopLines.size() << ","
                            << pL->fromLanes.size() << ","
                            << pL->toLanes.size() << ",";
                    if (pL->pLeftLane != nullptr)
                    {
                        mapLane << (pL->pLeftLane)->id << ",";
                    }
                    else
                    {
                        mapLane << "null,";
                    }
                    if (pL->pRightLane != nullptr)
                        mapLane << (pL->pRightLane)->id << ",";
                    else
                    {
                        mapLane << "null,";
                    }
                    roadMapLaneData.push_back(mapLane.str());

                    // for (auto wp : map.roadSegments.at(rs).Lanes.at(i).points)
                    // {
                    //     std::ostringstream lane_wp;
                    //     lane_wp << wp.id << ","
                    //      << wp.enable_right_change << ","
                    //      << wp.enable_left_change << ",";

                    //     // if (wp.pRight != nullptr)
                    //     // {
                    //     //     lane_wp << wp.enable_right_change << ","<< wp.RightLnId << ",";
                    //     // }
                    //     // else
                    //     // {
                    //     //     lane_wp << "null,";
                    //     // }
                    //     // if (wp.pLeft != nullptr)
                    //     // {
                    //     //     lane_wp << wp.enable_left_change << ","<< wp.LeftLnId << ",";
                    //     // }
                    //     // else
                    //     // {
                    //     //     lane_wp << "null,";
                    //     // }
                    //     roadMapLaneData.push_back(lane_wp.str());
                    // }
                }
                else
                {
                    std::cout << "bad lane," << i << std::endl;
                }
            }
        }
        std::string header = "Id,roadId,laneSectionId,speed,length,dir,type,width,"
                             "num,waypoints size,trafficlights.size,"
                             "stopLines.size,fromLanes.size,toLanes.size,"
                             "pLeftLane id,pRightLane id";
        std::ostringstream fileName;
        fileName << UtilityHNS::UtilityH::GetHomeDirectory() +
                        UtilityHNS::DataRW::LoggingMainfolderName +
                        UtilityHNS::DataRW::MapInfoFolderName;
        fileName << "MapLaneData.csv";
        std::ofstream f(fileName.str().c_str());
        if (f.is_open())
        {
            if (header.size() > 0)
                f << header << "\r\n";
            for (unsigned int i = 0; i < roadMapLaneData.size(); i++)
                f << roadMapLaneData.at(i) << "\r\n";
        }
        f.close();
    }
} // namespace opendrive
