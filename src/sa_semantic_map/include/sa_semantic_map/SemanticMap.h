#pragma once
#include <unordered_map>
#include "BoundaryMap.h"

#include <bits/stdc++.h>
//#include "geometry_msgs/Polygon.h" 
//#include "geometry_msgs/Point.h" 

#include "CDT_update.h"

struct NodeInfo {

    std::vector<std::string> childNodes;
    FeatureBase* nodeType = nullptr;
    int zoomLevel;
    std::vector<point_type> boundary;
    polygon Polygon;
    std::vector<int> consistsOf;
};


struct Environment {
    point_type3D origin = { 0,0,0 };
    int Tamb = 0;
};

struct TerrainInfo{
    point_type origin = {0,0};
    int zoomLevel = 0;

};

struct triangle_node {
    std::array<int,3> vertexIDs;
    std::array<point_type3D,3> vertexCoordinates;
    point_type centroid;
    std::string feature;
    std::string name;
    polygon Polygon;
};

struct DelaunayTriangulation{
    int triangle_id;
	triangle_node Triangle_info;
    // std::vector<int> neighbors;
    std::array<int,3> neighbors = {-1,-1-1};
};

class SemMap {
public:
    std::unordered_map<std::string, NodeInfo> fm;
    std::vector<DelaunayTriangulation> dtm;
    std::unordered_map<std::string, GCM_s> GCMs;
	SemMap();
    //void AddGCM(GCM_s inputGCM);
    void AddFeature(Feature inputFeature);
    void CreateRestObjects();
    void CreateRestObjects_end();
    void CalMaxZoomLevel();
    std::vector<std::string> FindZoomLevelNodes(int zoomLevel);
    void AddTerrainInfo(int x, int y, int zoomLevel);
    void CreateMap();
    void UpdatePtsConstraints();
    void CreateMesh();
    
    bool VisibilityQuery(point_type pt);
    
    Environment env;
    int maxZoomLevel = 0;
    int fm_dirty;   // 1 is yes and 0 is no 
    
private:
    Triangulation cdt = Triangulation(CDT::FindingClosestPoint::ClosestRandom, 10);
    std::unordered_map<std::string, NodeInfo> sm;
    DelaunayTriangulation DTz;
    std::vector<TerrainInfo> allTerrainInfo;

    int l_ti;
    int DTz_dirty;
    int env_dirty;
    std::vector<V2d> points;
    std::vector<Edge> edges;
};

// DelaunayTriangulation CreateMesh(std::unordered_map<std::string, NodeInfo> fm);