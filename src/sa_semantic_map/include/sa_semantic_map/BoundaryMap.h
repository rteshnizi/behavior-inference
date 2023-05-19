#pragma once

#include "CommonStructures.h"


class FeatureBase {
public:
	Feature currentFeature;
	
	FeatureBase(); 
	void ResetFeature();
	void FeatureShape(point_type ori, std::string type, std::vector<double> details);
	void shiftOrigin(double x, double y);
};

class FeatureShape {
public:
	//shape_s shapeParams;

	//FeatureShape(point_type ori, std::string type, std::vector<double> details);
	//void shiftOrigin(double x, double y);
};

class FeatureTree : public FeatureBase {


};

class FeatureRoad : public FeatureBase {


};

class FeatureTrail : public FeatureBase {


};

class FeatureRock : public FeatureBase {


};

class FeatureUnknown : public FeatureBase {


};
