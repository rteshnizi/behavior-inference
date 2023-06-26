#pragma once
#include <math.h> 
#include <vector>
#include <array>
#include <string>
#include <map>

#include "../include/rapidjson/writer.h"
#include "../include/rapidjson/reader.h"
#include "../include/rapidjson/stringbuffer.h"
#include "../include/rapidjson/document.h" 
#include "../include/rapidjson/istreamwrapper.h"
#include "../include/rapidjson/ostreamwrapper.h"
#include "SemanticMap.h"
#include <stdio.h>

using namespace std;
using namespace rapidjson;

#include "UTM.h"
//#define M_PI 3.141592653589793238

struct NAI_details {
	vector<array<double, 2>> corners;
	array<double, 2> center;
};

struct PL_details {
	vector<array<double, 2>> vertices;
};


map<int, array<double, 2>> readCP(point_type origin);
map<int, NAI_details> readNAI(point_type origin, string file_name);
map<int, PL_details> readPL(point_type origin, string file_name);
//void generate_data(Triangulation *cdt1);
