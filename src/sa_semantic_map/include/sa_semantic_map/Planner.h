#pragma once
#include <math.h> 
#include <vector>
#include <string>
#include <map>
#include "SemanticMap.h"

struct two_int_vect{
std::vector<int> a;
std::vector<int> b;
std::array<float,2> ori;
};

two_int_vect createPGM(double resolution, SemMap *map, std::vector<double> max_min, std::map<int, std::string> idx2name, std::string path);

