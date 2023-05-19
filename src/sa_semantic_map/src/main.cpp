#include <chrono>
#include <functional>
#include <memory>
#include <string>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "sa_msgs/msg/pose.hpp"
#include "sa_msgs/msg/feature_info.hpp"
#include "sa_msgs/msg/pose_array.hpp"
#include "sa_msgs/srv/query_visibility.hpp"
#include "sa_msgs/msg/visibility_array.hpp"

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

#include <vector>

#include <sstream>
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <limits>
#include <algorithm>
#include <iomanip>

//#include "../include/SemanticMap.h"
//#include "../include/CDT_update.h"


#include "../include/sa_semantic_map/Planner.h"
#include "../include/sa_semantic_map/UTM.h"




#include "../include/sa_semantic_map/rapidjson/writer.h"
#include "../include/sa_semantic_map/rapidjson/reader.h"
#include "../include/sa_semantic_map/rapidjson/stringbuffer.h"
#include "../include/sa_semantic_map/rapidjson/document.h"
#include "../include/sa_semantic_map/rapidjson/istreamwrapper.h"
#include "../include/sa_semantic_map/rapidjson/ostreamwrapper.h"

#include <boost/filesystem.hpp>
//#include <boost/geometry.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/mersenne_twister.hpp>

using namespace std;
using namespace rapidjson;

using boost::geometry::append;
using boost::geometry::correct;

boost::random::mt19937 rng;
boost::random::uniform_int_distribution<> color(10, 255);

SemMap SM;
point_type origin;
auto feture_defination_msg = sa_msgs::msg::FeatureInfo();
auto line_list = visualization_msgs::msg::Marker();

class MyNode : public rclcpp::Node
{
    public:
        MyNode() : Node("sml_VisibilityQuery_Service")
        {
            timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&MyNode::timerCallback, this));
        }
    private:
        void timerCallback()
        {
            //RCLCPP_INFO(this->get_logger(), "Hello from ROS2");
        }


        rclcpp::TimerBase::SharedPtr timer_;
};

class MapPolygonPublisher : public rclcpp::Node
{
  public:
    MapPolygonPublisher()
    : Node("sml_mapPolygon_Publisher"), count_(0)
    {
      publisher_1 = this->create_publisher<visualization_msgs::msg::Marker>("FeatureMap_Viz", 10);
      //publisher_2 = this->create_publisher<geometry_msgs::msg::PolygonStamped>("FeatureMap_p", 10);
      publisher_3 = this->create_publisher<sa_msgs::msg::FeatureInfo>("FeatureMap_BIL", 10);

      //rclcpp::Service<sa_msgs::srv::QueryVisibility>::SharedPtr service =                 // CHANGE
  //      this->create_service<sa_msgs::srv::QueryVisibility>("uav1/visibility_query",  &MapPolygonPublisher::visibility_query);

      timer_ = this->create_wall_timer(500ms, std::bind(&MapPolygonPublisher::timer_callback, this));
    }

  private:
    void timer_callback( )
    {
      RCLCPP_INFO(this->get_logger(), "Publishing map ");

      auto points = visualization_msgs::msg::Marker();
      auto line_strip = visualization_msgs::msg::Marker();
      //auto line_list = visualization_msgs::msg::Marker();
      points.header.frame_id = "map";
      points.header.stamp = rclcpp::Node::now() ;
      line_strip.header.frame_id = "map";
      line_strip.header.stamp = rclcpp::Node::now() ;
      line_list.header.frame_id = "map";
      line_list.header.stamp = rclcpp::Node::now() ;

            points.ns = line_strip.ns = line_list.ns = "points_and_lines";
            points.action = line_strip.action = line_list.action = visualization_msgs::msg::Marker::ADD;
            points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

            points.id = 0;
            line_strip.id = 1;
            line_list.id = 2;

            //points.type = visualization_msgs::msg::Marker::POINTS;
            //line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
            // POINTS markers use x and y scale for width/height respectively
            //points.scale.x = 0.2;
            //points.scale.y = 0.2;
            // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            //line_strip.scale.x = 0.1;
            line_list.scale.x = 0.5;

            // Points are green
            //points.color.g = 1.0f;
            //points.color.a = 1.0;

            // Line strip is blue
            //line_strip.color.b = 1.0;
            //line_strip.color.a = 1.0;

            // Line list is red
            line_list.color.r = 1.0;
            line_list.color.a = 1.0;

            //publisher_1->publish(points);
            //publisher_1->publish(line_strip);
            publisher_1->publish(line_list);
            publisher_3->publish(feture_defination_msg);
    }


    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<sa_msgs::msg::FeatureInfo>::SharedPtr publisher_3;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_1;
    size_t count_;
};

void visibility_query(const std::shared_ptr<sa_msgs::srv::QueryVisibility::Request> request,
    std::shared_ptr<sa_msgs::srv::QueryVisibility::Response> response)
    {
        int sz_rq = request->traj_array.size();

        for(int i =0; i<sz_rq; i++)
        {
            auto traj_i  = request->traj_array[i];
            int sz_traj = traj_i.traj.size();

            auto op = sa_msgs::msg::VisibilityArray();

            for(int j = 0 ; j < sz_traj ; j++)
            {
                auto pose_pt = traj_i.traj[i];
                auto pose_x = pose_pt.x;
                auto pose_y = pose_pt.y;
                auto pose_th = pose_pt.yaw;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pose: %f, %f, %f ", pose_x, pose_y, pose_th);
                //cout << pose_x << " " << pose_y << " " << pose_th  << " " << endl;

                point_type point(pose_x, pose_y);
                bool visi = SM.VisibilityQuery(point);
                op.visibilities.push_back(visi);
            }
            response->visibility_array.push_back(op);
        }
    }



int main(int argc, char* argv[])
{

  rclcpp::init(argc, argv);

  stringstream ss;

//  string file_name_json = "/home/anant/sa_ros2_ws/src/situational_awareness/sa_semantic_map/data/Features_quarry_V6.json";
//   string file_name_json = "/home/anant/sa_ros2_ws/src/situational_awareness/sa_semantic_map/data/map_v1.json";
  string file_name_json = "/home/reza/git/ros2_wrk/src/situational_awareness/sa_semantic_map/data/map_v1.json";

  ifstream file(file_name_json);
  if (file)
  {
      ss << file.rdbuf();
      file.close();
  }
  else
  {
      throw std::runtime_error("!! Unable to open json file");
  }
  Document doc;
  if (doc.Parse<0>(ss.str().c_str()).HasParseError())
      throw std::invalid_argument("json parse error");

  std::string type = doc["type"].GetString();
  std::string generator = doc["generator"].GetString();
  cout << type << endl;
  cout << generator << endl;
  const Value& array = doc["features"];
  cout << array.Size() << endl;

  vector<Feature> inputs;
  vector<point_type> boundary_ballpark;

  int idx_counter = 0;
  map<int, string> idx2name;
  map<string, int> name2idx;



  vector<string> feature_name_list;

cout << "No of features " << array.Size() << endl;

for (rapidjson::SizeType i = 0; i < array.Size(); i++)
    {
        //cout << feature.key() << "\n";
        cout << i << "\n";
        auto pose_array = sa_msgs::msg::PoseArray();

        auto& val = array[i];

        Feature ip;
        ip.index = idx_counter;

        const Value& prop = val["properties"];
        ip.featureName = prop["Name"].GetString();
        if (prop.HasMember("Parent"))
            ip.parent = prop["Parent"].GetString();
        cout << i  << "  " << ip.featureName << "\n";
        ip.sensorVisibility.fromAbove = prop["sensor_visibility_above"].GetString();
        ip.sensorVisibility.fromSide = prop["sensor_visibility_side"].GetString();
        string val_temp = prop["traversability_av"].GetString();
        ip.traversability.av = stoi(val_temp);
        val_temp = prop["traversability_gv"].GetString();
        ip.traversability.gv = stoi(val_temp);
        val_temp = prop["traversability_ped"].GetString();
        ip.traversability.pedestrian = stoi(val_temp);

        val_temp = prop["traversability_tank"].GetString();
        int temp_tank = stoi(val_temp);

        ip.type = "bb";
        const Value& geo = val["geometry"];
        const Value& geo_cord = geo["coordinates"];

        if (ip.featureName == "Ballpark") {
            float x;
            float y;
            float lon = geo_cord[0][0].GetFloat();
            float lat = geo_cord[0][1].GetFloat();

            //int zone = LatLonToUTMXY(lat, lon, 14, x, y);
            x = lon;
            y = lat;

            //cout << zone << endl;
            origin.set<0>(x);
            origin.set<1>(y);
            SM.env.origin.set<0>(x);
            SM.env.origin.set<0>(y);
        }

        for (unsigned int i = 0; i < geo_cord.Size(); i++)
        {
            auto pose = sa_msgs::msg::Pose();
            auto p = geometry_msgs::msg::Point();
            float x;
            float y;
            float lon = geo_cord[i][0].GetFloat();
            float lat = geo_cord[i][1].GetFloat();

            //int zone = LatLonToUTMXY(lat, lon, 14, x, y);
            x = lon;
            y = lat;

            //cout << zone << endl;
            x = x ;//- origin.get<0>();
            y = y ;//- origin.get<1>();
            pose.x = x;
            pose.y = y;

            p.x = x;
            p.y = y;

            pose.yaw = 0.0;
            cout << pose.x  << "    " << pose.y << endl;
            ip.shapeParams.boundary.push_back(point_type(x,y));
            pose_array.traj.push_back(pose);
            line_list.points.push_back(p);
            if(i>0){
                line_list.points.push_back(p);
            }
        }

        auto p = geometry_msgs::msg::Point();
        float x;
        float y;
        float lon = geo_cord[0][0].GetFloat();
        float lat = geo_cord[0][1].GetFloat();
        x = lon;
        y = lat;
        p.x = x;
        p.y = y;
        line_list.points.push_back(p);

        if (ip.featureName == "Ballpark")
            boundary_ballpark = ip.shapeParams.boundary;

        if (ip.featureName.find("grass") != std::string::npos) {
            ip.traversability.gv = ip.traversability.gv - 15;
            //std::cout << "found!" << '\n';
        }
        append(ip.shapeParams.Polygon, ip.shapeParams.boundary);
        correct(ip.shapeParams.Polygon);

        string temp_str = "gv_untraversable";
        if (ip.traversability.gv < 15) {
            if (name2idx.find(temp_str) == name2idx.end()) {
                name2idx.insert(pair<string, int>(temp_str, idx_counter));
                idx2name.insert(pair<int, string>(idx_counter, temp_str));
                idx_counter++;
            }
            else {
                int idx_temp = name2idx[temp_str];
                //name2idx.insert(pair<string, int>(temp_str, idx_temp));
                //idx2name.insert(pair<int, string>(idx_temp, temp_str));
                ip.index = idx_temp;
            }
        }
        else {
            string name_temp;
            size_t pos = ip.featureName.find("_");
            string str3 = ip.featureName.substr(0, pos+1);
            name_temp = str3 + std::to_string(ip.traversability.gv);
            if (name2idx.find(name_temp) == name2idx.end()) {
                name2idx.insert(pair<string, int>(name_temp, idx_counter));
                idx2name.insert(pair<int, string>(idx_counter, name_temp));
                idx_counter++;
            }
            else {
                int idx_temp = name2idx[name_temp];
                //name2idx.insert(pair<string, int>(temp_str, idx_temp));
                //idx2name.insert(pair<int, string>(idx_temp, temp_str));
                ip.index = idx_temp;
            }
        }

        feture_defination_msg.feature_name.push_back(ip.featureName);
        feture_defination_msg.polygon_shape_list.push_back(pose_array);
        feture_defination_msg.traversability_gv_car.push_back(ip.traversability.gv);
        feture_defination_msg.traversability_gv_tank.push_back(temp_tank);
        feture_defination_msg.visibility_av.push_back(ip.sensorVisibility.fromAbove);
        feture_defination_msg.visibility_av.push_back(prop["terrain"].GetString());


        inputs.push_back(ip);
    }

    for (std::pair<string, int> element : name2idx)
    {
        // cout << (element.first) << "     "  << element.second << endl;

    }
    for (std::pair<int, string> element : idx2name)
    {
        // cout << (element.first) << "     "  << element.second << endl;

    }

FeatureBase FB;
//    SemMap SM;

    // Defining Ballpark in the map

    FB.currentFeature = inputs[0];
    SM.AddFeature(FB.currentFeature);
    FB.ResetFeature();

    for (int i = 1; i < inputs.size(); i++)
    {
        //cout << "val of i = "<< i << "       "<< inputs[i].featureName << "    Parent is   ";
        string parent;
        int zoom_parent = 0;
        polygon currentPolygon;
        currentPolygon = inputs[i].shapeParams.Polygon;
        SM.CalMaxZoomLevel();
        int break_bool = false;
        for (int j = SM.maxZoomLevel; j >= 0; j--)
        {
            zoom_parent = j;
            vector<string> nodes_0 = SM.FindZoomLevelNodes(j);
            for (int k = 0; k < nodes_0.size(); k++)
            {
                parent = nodes_0[k];
                polygon tempPolygon;
                tempPolygon = SM.fm[nodes_0[k]].Polygon;
                std::vector<polygon> diffPolygons;
                boost::geometry::difference(currentPolygon, tempPolygon, diffPolygons);
                if (diffPolygons.size() == 0)
                {
                    break_bool = true;
                    break;
                }
            }
            if (break_bool)
                break;
        }
        //cout << parent << "   " <<  zoom_parent << endl;
        FB.currentFeature = inputs[i];
        FB.currentFeature.parent = parent;
        FB.currentFeature.zoomLevel = zoom_parent+1;
        SM.AddFeature(FB.currentFeature);
        FB.ResetFeature();
        if ((zoom_parent) == SM.maxZoomLevel && SM.maxZoomLevel!=0)
        {
        // cout << zoom_parent << "    " << SM.maxZoomLevel << endl;
        //SM.CreateRestObjects();
        }
    }
    SM.CalMaxZoomLevel();

    // std::ofstream svg("my_map.svg");
    // boost::geometry::svg_mapper<point_type> mapper(svg, 200, 200);

    int count = 0;

    if(SM.fm_dirty == 1)
    {
        SM.CreateMesh();
        SM.fm_dirty == 0;
    }

    cout << SM.fm["Road 3"].nodeType->currentFeature.sensorVisibility.fromAbove << endl;
    //cout << SM.fm["Water 1"].nodeType->currentFeature.shapeParams.Polygon.outer << endl;

    cout << "350" << endl;

    auto node = std::make_shared<MyNode>();
    rclcpp::Service<sa_msgs::srv::QueryVisibility>::SharedPtr service =                 // CHANGE
    node->create_service<sa_msgs::srv::QueryVisibility>("uav1/visibility_query",  &visibility_query);     // CHANGE


    // std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("sml_server");  // CHANGE
    // rclcpp::Service<sa_msgs::srv::QueryVisibility>::SharedPtr service =                 // CHANGE
    // node->create_service<sa_msgs::srv::QueryVisibility>("uav1/visibility_query",  &visibility_query);     // CHANGE

    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add three ints.");      // CHANGE
    //rclcpp::spin(node);

    //auto node2 = std::make_shared<MyNode>();
    //rclcpp::spin(std::make_shared<MinimalPublisher>());

    auto node3 = std::make_shared<MyNode>();
    rclcpp::spin(std::make_shared<MapPolygonPublisher>());


    rclcpp::shutdown();




    /*auto x_max = *std::max_element(boundary_ballpark.begin(), boundary_ballpark.end(),
        [](point_type a1, point_type a2) {
            return a1.get<0>() < a2.get<0>();
        });

    cout << "266 " << endl;
    auto y_max = *std::max_element(boundary_ballpark.begin(), boundary_ballpark.end(),
        [](point_type a1, point_type a2) {
            return a1.get<1>() < a2.get<1>();
        });

        cout << "267 " << endl;
    auto x_min = *std::max_element(boundary_ballpark.begin(), boundary_ballpark.end(),
        [](point_type a1, point_type a2) {
            return a1.get<0>() > a2.get<0>();
        });

        cout << "268" << endl;
    auto y_min = *std::max_element(boundary_ballpark.begin(), boundary_ballpark.end(),
        [](point_type a1, point_type a2) {
            return a1.get<1>() > a2.get<1>();
        });
        cout << "269 " << endl;

    vector<double> max_min;
    max_min.push_back(x_max.get<0>());
    max_min.push_back(x_min.get<0>());
    max_min.push_back(y_max.get<1>());
    max_min.push_back(y_min.get<1>());

    vector<point_type> corners_lat_lon;
    vector<point_type> corners_en;

    for (int i = 2; i <= 3; i++)
    {
        int zone = 14;
        float lat, lon;
        bool south_hemis = false;

        float y = max_min[i] + origin.get<1>();
        for (int j = 1; j >= 0; j--)
        {
            point_type lat_lon, xy;

            float x = max_min[j] + origin.get<0>();

            UTMXYToLatLon(x, y, zone, south_hemis, lat, lon);
            //lat_lon.set<0>(lat * 180 / M_PI);
            //lat_lon.set<1>(lon * 180 / M_PI);
            lat_lon.set<0>(x);
            lat_lon.set<1>(y);
            xy.set<0>(max_min[j]);
            xy.set<1>(max_min[i]);

            // cout << x << "," << y << endl;
            // cout << max_min[j] << "," << max_min[i] <<endl;
            corners_lat_lon.push_back(lat_lon);
            corners_en.push_back(xy);
        }
    }

    cout << "280 " << endl;
    */


    // for (int jk = 0; jk < SM.dtm.size();jk++){
    //     cout << SM.dtm[jk].Triangle_info.feature << endl;
    //     cout << "Nigh. of Triang " << jk << " is,   " ;
    //     if (SM.dtm[jk].neighbors[0]!= -1)
    //         cout << "Triang " << SM.dtm[jk].neighbors[0] <<"  1,   " ;
    //     if (SM.dtm[jk].neighbors[1]!= -1)
    //         cout << "Triang " << SM.dtm[jk].neighbors[1] <<"  2,   " ;
    //     if (SM.dtm[jk].neighbors[2]!= -1)
    //         cout << "Triang " << SM.dtm[jk].neighbors[2] << "  3";
    //     cout<<endl;

    // }

    /*
    string path;
    //n.getParam("/samap_/path",path);
    //cout << path << endl;

    two_int_vect pgmimg;
    if(use_ogmap == 0)
    {
        pgmimg = createPGM(reso, &SM, max_min, idx2name, path);
        cout << "PGM_Done" << endl;
    }

    else
    {
        int numrows = 0, numcols = 0;
        string file_name_ogmap;
        n.getParam("/samap_/file_name_ogmap",file_name_ogmap);
        ifstream file_ogmap(file_name_ogmap);
        string line;
        if(file_ogmap)
        {
            getline(file_ogmap,line);
            getline(file_ogmap,line);
            istringstream iss(line);
            string subs;
            iss >> subs;
            //cout << "rows" << subs << endl;
            numrows = stoi(subs);
            iss >> subs;
            //cout << "cols" << subs << endl;
            numcols = stoi(subs);

            pgmimg.a.push_back(numrows);
            pgmimg.a.push_back(numcols);

            pgmimg.ori[0] = x_min.get<0>();
            pgmimg.ori[1] = y_min.get<1>();
            //cout << "step1" << endl;
            getline(file_ogmap,line);

            while(getline(file_ogmap,line))
            {
                pgmimg.a.push_back(stoi(line));
               // cout<<line<<endl;
            }
            //cout << "step1" << endl;
            file_ogmap.close();
        }
    }

    //cout <<pgmimg.a.size() << endl;
    //cout <<pgmimg.b.size() << endl;

    float x =  x_min.get<0>() + origin.get<0>();
    float y =  y_min.get<1>() + origin.get<1>();
    nav_msgs::OccupancyGrid myMap;
    //cout << x << "  " << y << endl;
    // Map data
    myMap.info.map_load_time = ros::Time(0);
    myMap.info.resolution = reso;
    myMap.info.width = pgmimg.a[0];
    myMap.info.height = pgmimg.a[1];
    myMap.info.origin.position.x = pgmimg.ori[0] + origin.get<0>();
    myMap.info.origin.position.y = pgmimg.ori[1] + origin.get<1>();
    myMap.info.origin.position.z = 0;
    myMap.info.origin.orientation.x = 0;
    myMap.info.origin.orientation.y = 0;
    myMap.info.origin.orientation.z = 0;
    myMap.info.origin.orientation.w = 1;

    for(int i = 2; i < pgmimg.a.size() ; i++)
    {
        myMap.data.push_back(pgmimg.a[i]);
    }

    myMap.header.frame_id = "world";
    map_pub.publish(myMap);
    cout << "published cost map image" << endl;
    */


  return 0;


}
