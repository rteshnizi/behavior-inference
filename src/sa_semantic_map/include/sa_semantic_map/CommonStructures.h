#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <array>
#include <boost/geometry.hpp>
#include <boost/geometry/geometry.hpp>


// #include "SemanticMap.h"
//#include <boost/geometry/geometries/point_xy.hpp>
//#include <boost/geometry/geometries/polygon.hpp>

typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_type;
typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> point_type3D;
typedef boost::geometry::model::polygon<point_type > polygon;

struct state_s {
	int gv, pedestrian, av;
};

struct sensorVisibility_s {
	std::string fromSide, fromAbove;
};

struct waypnt {
	std::vector<point_type> xy;
	int width;
};

struct shape_s {
	point_type origin;
	std::vector<point_type> boundary;
	polygon Polygon;
	waypnt waypoint;
	double r;
	std::string type;
	point_type centroid;
};

struct Feature {

	std::string featureName;
	std::string type;
	int zoomLevel;
	std::string parent;
	state_s traversability;
	state_s occupiability;
	sensorVisibility_s sensorVisibility;
	shape_s shapeParams;
	int index;
};
// struct DelaunayTriangulation {
// 	sam_node Triangle_info;
//     // std::vector<int> neighbors;
//     int neighbors[3] = {-1,-1-1};
// };

struct GCM_s {

	std::string GCMName;
	std::string GCMType;
	bool isReference;
	std::string referencedEntity;
	std::string referenceQualifier;
	int priority;
	shape_s shapeParams;
};


