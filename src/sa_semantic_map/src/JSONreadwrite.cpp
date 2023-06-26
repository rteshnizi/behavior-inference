// read and write json files:

#include "../include/sa_semantic_map_pkg/JSONreadwrite.h"


// read JSON if polygons in latitude and longitude
/*vector<Feature> readjson_lla()
{
    ifstream i("C:/Users/anant/source/repos/SemanticMap/Semantic_Map/rellis_map.json");

    json j = json::parse(i);
    vector<Feature> inputs;
    vector<point_type> boundary_ballpark;
    point_type origin;

    for (const auto& feature : j["features"].items())
    {
        cout << feature.key() << "\n";
        auto val = feature.value();
        cout << val["properties"]["Name"] << endl;
        Feature ip;
        ip.featureName = val["properties"]["Name"];
        if (val["properties"].contains(string("Parent")))
            ip.parent = val["properties"]["Parent"];
        ip.sensorVisibility.fromAbove = val["properties"]["sensor_visibility_above"];
        ip.sensorVisibility.fromSide = val["properties"]["sensor_visibility_side"];
        string val_temp = val["properties"]["traversability_av"];
        ip.traversability.av = stoi(val_temp);
        val_temp = val["properties"]["traversability_gv"];
        ip.traversability.gv = stoi(val_temp);
        val_temp = val["properties"]["traversability_ped"];
        ip.traversability.pedestrian = stoi(val_temp);
     
        ip.shapeParams.type = val["properties"]["type"];
        ip.type = ip.shapeParams.type;
        auto geo = val["geometry"]["coordinates"];

        // Defining origin 
        if (ip.featureName == "Ballpark")
        {
            findOrigin(origin, geo[0][1], geo[0][0]);
        }

        // converting lat long to xy and saving boundary
        for (int i = 0; i < geo.size(); i++)
        {
            point_type coordinate = lla2enu(origin, geo[i][1], geo[i][0]);
            if (ip.shapeParams.type == "trail")
            {
                ip.shapeParams.waypoint.points_enu.push_back(coordinate);
                ip.shapeParams.waypoint.points_lla.push_back(point_type(geo[i][0], geo[i][1]));
            }
            else if (ip.shapeParams.type == "bb")
            {
                ip.shapeParams.boundary_enu.push_back(coordinate);
                ip.shapeParams.boundary_lla.push_back(point_type(geo[i][0], geo[i][1]));
            }
        }
        if (ip.shapeParams.type == "trail")
        {
            string wd = val["properties"]["width"];
            ip.shapeParams.waypoint.width = stod(wd);

            vector<point_type> op = trail2polygon(ip.shapeParams.waypoint.width, ip.shapeParams.waypoint.points_enu);
            ip.shapeParams.boundary_enu = op;
        }

        if (ip.featureName == "Ballpark")
            boundary_ballpark = ip.shapeParams.boundary_enu;

        append(ip.shapeParams.Polygon, ip.shapeParams.boundary_enu);
        correct(ip.shapeParams.Polygon);
        inputs.push_back(ip);
    }


    // find xy coordinate limits

    vector<double> max_min = xy_Limits(boundary_ballpark);

    return(inputs);
}*/


void findOrigin(point_type &origin, double lat, double lon)
{
    double x;
    double y;
    //int zone = LatLonToUTMXY(lat, lon, Area_ZONE_UTC, x, y);
    //origin.set<0>(x);
    //origin.set<1>(y);
    origin.set<0>(lon);
    origin.set<1>(lat);
}

/*point_type lla2enu(point_type &origin, double lat, double lon)
{
    double x;
    double y;
    //int zone = LatLonToUTMXY(lat, lon, Area_ZONE_UTC, x, y);
    x = x - origin.get<0>();
    y = y - origin.get<1>();
    return(point_type(x, y));
}*/

vector<double> xy_Limits(vector<point_type> boundary_ballpark)
{
    // x max, x min, y max, ymin  
    auto x_max = *std::max_element(boundary_ballpark.begin(), boundary_ballpark.end(),
        [](point_type a1, point_type a2) {
            return a1.get<0>() < a2.get<0>();
        });

    auto y_max = *std::max_element(boundary_ballpark.begin(), boundary_ballpark.end(),
        [](point_type a1, point_type a2) {
            return a1.get<1>() < a2.get<1>();
        });

    auto x_min = *std::max_element(boundary_ballpark.begin(), boundary_ballpark.end(),
        [](point_type a1, point_type a2) {
            return a1.get<0>() > a2.get<0>();
        });

    auto y_min = *std::max_element(boundary_ballpark.begin(), boundary_ballpark.end(),
        [](point_type a1, point_type a2) {
            return a1.get<1>() > a2.get<1>();
        });

    vector<double> max_min;
    max_min.push_back(x_max.get<0>());
    max_min.push_back(x_min.get<0>());
    max_min.push_back(y_max.get<1>());
    max_min.push_back(y_min.get<1>());

    return(max_min);
}


/*vector<point_type>  trail2polygon(double width, vector<point_type> points)
{
    points.erase(points.end() - 1);
    vector<point_type3D> t_xyz;
    vector<point_type3D> t_n;
    point_type3D z(0, 0, 1);
    for (int i = 1; i < points.size(); i++)
    {
        point_type3D pt(points[i].get<0>() - points[i - 1].get<0>(),
            points[i].get<1>() - points[i - 1].get<1>(),
            0);
        t_xyz.push_back(pt);
        point_type3D r1;
        r1 = boost::geometry::cross_product(pt, z);
        double l2_norm = sqrt(pow(r1.get<0>(), 2) + pow(r1.get<1>(), 2) +
            pow(r1.get<2>(), 2));
        point_type3D r2(r1.get<0>() / l2_norm, r1.get<1>() / l2_norm,
            r1.get<2>() / l2_norm);
        t_n.push_back(r2);
    }
    vector<point_type3D> t_n2;
    vector<point_type> np1, np2;

    for (int i = 0; i <= t_n.size(); i++)
    {
        point_type new_pt;
        double plus_minus_x, plus_minus_y, x, y;
        if (i == 0)
        {
            t_n2.push_back(t_n[i]);
            x = t_n[i].get<0>();
            y = t_n[i].get<1>();
            plus_minus_x = x * width;
            plus_minus_y = y * width;
            new_pt.set<0>(points[i].get<0>() + plus_minus_x);
            new_pt.set<1>(points[i].get<1>() + plus_minus_y);
            np1.push_back(new_pt);
            new_pt.set<0>(points[i].get<0>() - plus_minus_x);
            new_pt.set<1>(points[i].get<1>() - plus_minus_y);
            np2.push_back(new_pt);
        }
        else if (i == t_n.size())
        {
            t_n2.push_back(t_n[i - 1]);
            x = t_n[i - 1].get<0>();
            y = t_n[i - 1].get<1>();
            plus_minus_x = x * width;
            plus_minus_y = y * width;
            new_pt.set<0>(points[i].get<0>() + plus_minus_x);
            new_pt.set<1>(points[i].get<1>() + plus_minus_y);
            np1.push_back(new_pt);
            new_pt.set<0>(points[i].get<0>() - plus_minus_x);
            new_pt.set<1>(points[i].get<1>() - plus_minus_y);
            np2.push_back(new_pt);
        }
        else
        {
            point_type3D pt1 = t_n[i - 1];
            point_type3D pt2 = t_n[i];
            point_type3D pt3((pt1.get<0>() + pt1.get<0>()) / 2,
                (pt1.get<1>() + pt1.get<1>()) / 2,
                (pt1.get<2>() + pt1.get<2>()) / 2);
            t_n2.push_back(pt3);

            x = (t_n[i - 1].get<0>() + t_n[i].get<0>()) / 2;
            y = (t_n[i - 1].get<1>() + t_n[i].get<1>()) / 2;
            plus_minus_x = x * width;
            plus_minus_y = y * width;
            new_pt.set<0>(points[i].get<0>() + plus_minus_x);
            new_pt.set<1>(points[i].get<1>() + plus_minus_y);
            np1.push_back(new_pt);
            new_pt.set<0>(points[i].get<0>() - plus_minus_x);
            new_pt.set<1>(points[i].get<1>() - plus_minus_y);
            np2.push_back(new_pt);
        }
    }

    for (int i = np2.size() - 1; i >= 0; i--)
    {
        np1.push_back(np2[i]);
    }
    np1.push_back(np1[0]);

    return(np1);
}*/

/*void enu_boundary2lla(point_type& origin, vector<point_type> boundary_enu, vector<point_type>& boundary_lla)
{
    for (int j = 0; j < boundary_enu.size(); j++)
    {
        point_type lat_lon;
        double x = boundary_enu[j].get<0>() + origin.get<0>();
        double y = boundary_enu[j].get<1>() + origin.get<1>();
        double lat, lon;
        UTMXYToLatLon(x, y, Area_ZONE_UTC, sounthern_hemisphere, lat, lon);
        lat_lon.set<0>(lon * 180 / M_PI);
        lat_lon.set<1>(lat * 180 / M_PI);
        boundary_lla.push_back(lat_lon);
    }
}*/

