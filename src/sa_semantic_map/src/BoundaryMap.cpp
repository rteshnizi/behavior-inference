#include "../include/sa_semantic_map/BoundaryMap.h"
#include <math.h>

namespace trans = boost::geometry::strategy::transform;
typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_type;

# define M_PI           3.14159265358979323846  /* pi */

FeatureBase::FeatureBase() {
	
}

void FeatureBase::ResetFeature() {
	currentFeature.featureName = "default";
	currentFeature.zoomLevel = 0;
	currentFeature.parent = currentFeature.featureName;

	currentFeature.traversability.gv = 0;
	currentFeature.traversability.av = 100;
	currentFeature.traversability.pedestrian = 0;

	currentFeature.sensorVisibility.fromSide = "no";
	currentFeature.sensorVisibility.fromAbove = "yes";

	currentFeature.occupiability.gv = 0;
	currentFeature.occupiability.pedestrian = 100;
	currentFeature.occupiability.gv = 0;

	currentFeature.shapeParams.boundary.clear();
	boost::geometry::clear(currentFeature.shapeParams.Polygon);
}


void FeatureBase::FeatureShape(point_type ori, std::string type, std::vector<double> details) {
	currentFeature.shapeParams.origin = ori;
	if (type.compare("bb")) {
		int xmin = details[0];
		int xmax = details[1];
		int ymin = details[2];
		int ymax = details[3];

		point_type xy(xmin, ymin);
		currentFeature.shapeParams.boundary.push_back(xy);
		xy.set<0>(xmax);
		xy.set<1>(ymin);
		currentFeature.shapeParams.boundary.push_back(xy);
		xy.set<0>(xmax);
		xy.set<1>(ymax);
		currentFeature.shapeParams.boundary.push_back(xy);
		xy.set<0>(xmin);
		xy.set<1>(ymax);
		currentFeature.shapeParams.boundary.push_back(xy);
	}
	else if (type.compare("hex")) {
		int num_sides = 6;
		currentFeature.shapeParams.r = details[0];
		for (int i = 0; i < 6; i++) {
			double angle = (i * 2 * M_PI / num_sides) ;
			point_type xy(cos(angle) * currentFeature.shapeParams.r, sin(angle) * currentFeature.shapeParams.r);
			currentFeature.shapeParams.boundary.push_back(xy);
		}
	}
	else if (type.compare("wayPoints")) {
		int wP = details[0];
		currentFeature.shapeParams.waypoint.width = details[1];


	}

}

void FeatureBase::shiftOrigin(double x, double y) {
	point_type p1(currentFeature.shapeParams.origin.get<0>() , currentFeature.shapeParams.origin.get<1>());
	point_type p2;
	trans::translate_transformer<double, 2, 2> translate(x, y);
	boost::geometry::transform(p1, p2, translate);
}
