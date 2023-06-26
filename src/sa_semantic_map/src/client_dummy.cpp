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

//#include "geometry_msgs/Point.h" 

using namespace std::chrono_literals;

#include <vector>

#include <sstream>
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <limits>
#include <algorithm>
#include <iomanip> 


using namespace std;

int main(int argc, char* argv[])
{    

  rclcpp::init(argc, argv);

  stringstream ss;
  
//  string file_name_json = "/home/anant/sa_ros2_ws/src/situational_awareness/sa_semantic_map/data/Features_quarry_V6.json";
  string file_name_json = "/home/anant/sa_ros2_ws/src/situational_awareness/sa_semantic_map/data/map_v1.json";
 
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
            float x;
            float y;
            float lon = geo_cord[i][0].GetFloat();
            float lat = geo_cord[i][1].GetFloat();

            //int zone = LatLonToUTMXY(lat, lon, 14, x, y);
            x = lon;
            y = lat;

            //cout << zone << endl;
            x = x - origin.get<0>();
            y = y - origin.get<1>();
            pose.x = x;
            pose.y = y;
            pose.yaw = 0.0;

            ip.shapeParams.boundary.push_back(point_type(x,y)); 
            pose_array.traj.push_back(pose); 
        }  
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
        for (int j =
  stringstream ss;
  
//  string file_name_json = "/home/anant/sa_ros2_ws/src/situational_awareness/sa_semantic_map/data/Features_quarry_V6.json";
  string file_name_json = "/home/anant/sa_ros2_ws/src/situational_awareness/sa_semantic_map/data/map_v1.json";
 
  ifstream file(file_name_json); 
  if (file) 
  {
      ss << file.rdbuf();
      file.close();
  } 
  else 
  {
      thr

  return 0;

    
}

