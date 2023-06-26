#include "../include/sa_semantic_map/Planner.h"
#include <stdio.h>
#include <vector>
#include <string>

using namespace std;

two_int_vect createPGM(double resolution, SemMap *map, vector<double> max_min, std::map<int, std::string> idx2name,string path)
{
	// max_min has max x, min x, max y, min y 
	/*double ymax = 760;
	double ymin = 5;
	double xmax = -10;
	double xmin = -830;*/
	
	two_int_vect op;
	
	for (std::pair<std::string, NodeInfo> element : map->fm)
	{
		cout << element.first <<  "    " << element.second.nodeType->currentFeature.index << endl;
	}
	FILE* lookup_table;
	string name2 = path + "/data/lookup_table_test" + std::to_string(resolution) + ".csv";
	const char* c2 = name2.c_str();
	lookup_table = fopen(c2, "wb");

	std::map<int, std::string>::iterator itr;
	array<int, 3> trav_vals;
	for (itr = idx2name.begin(); itr != idx2name.end(); ++itr)
	{
		size_t pos = itr->second.find("_");
		string str3 = itr->second.substr(pos + 1);
		int trav_temp;
		
		if (str3 == "untraversable")
			trav_temp = 0;
		else
			trav_temp = stoi(str3);
		//trav_vals[0] = map->fm[itr->second].nodeType->currentFeature.traversability.pedestrian;
		//trav_vals[1] = map->fm[itr->second].nodeType->currentFeature.traversability.gv;
		//trav_vals[2] = map->fm[itr->second].nodeType->currentFeature.traversability.av;
		
		fprintf(lookup_table, "%d,", itr->first);
		//	fprintf(lookup_table, ",");
		fprintf(lookup_table, "%d", trav_temp);
		//	fprintf(lookup_table, ",");
		//	fprintf(lookup_table, "%d", trav_vals[1]);
		//	fprintf(lookup_table, ",");
		//	fprintf(lookup_table, "%d", trav_vals[2]);
		fprintf(lookup_table, "\n");
	}
	fclose(lookup_table);

	double xmax = max_min[0];
	double xmin = max_min[1];
	double ymax = max_min[2];
	double ymin = max_min[3];

	//int dim = 100 / resolution;
	int dim_x = abs(xmax - xmin) / resolution;
	int dim_y = abs(ymax - ymin) / resolution;

	FILE* pgmimg;
	string name = path + "/data/ogmap_" + std::to_string(resolution) + ".pgm";
	const char* c = name.c_str();
	pgmimg = fopen(c, "wb");
	// Writing Magic Number to the File 
	fprintf(pgmimg, "P2\n");
    // Writing Width and Height 
	fprintf(pgmimg, "%d %d\n", dim_x, dim_y);
	// Writing the maximum gray value 
	fprintf(pgmimg, "255\n");

	FILE* pgmimg1;
	string name1 = path + "/data/pgmimg_" + std::to_string(resolution) + ".pgm";
	const char* c1 = name1.c_str();
	pgmimg1 = fopen(c1, "wb");
	fprintf(pgmimg1, "P2\n");
	fprintf(pgmimg1, "%d %d\n", dim_x, dim_y);
	fprintf(pgmimg1, "255\n");
	cout << "pgm_wirte_" << endl;

	int count = 0;
	int def_val_img = 0;
	int def_val = 0;

    vector<int> map_og;
	vector<int> img_pgm;
	map_og.push_back(dim_x);
	img_pgm.push_back(dim_x);
	map_og.push_back(dim_y);
	img_pgm.push_back(dim_y);
	cout << "step1" << endl;
	
	//for (double y = 50 - resolution; y >= -50; y = y - resolution)
	for (double y = ymax - resolution; y >= ymin; y = y - resolution)
	{
		//for (double x = -50; x <= 50 - resolution; x = x + resolution) 
		for (double x = xmin; x <= xmax - resolution; x = x + resolution)
		{
			int val_pgm;
			int val_pgm_img;

			point_type point(x + resolution / 2, y + resolution / 2);
			if (boost::geometry::within(point, map->fm["Ballpark"].Polygon))
			{
				vector<int> values;
				vector<int> values_img;
				for (std::pair<std::string, NodeInfo> element : map->fm)
				{
					string name_temp; 
            		size_t pos = element.first.find("_"); 
					string str3 = element.first.substr(0, pos);
					//cout << str3 << endl;
					if ((element.first) != "Ballpark" && str3 != "rest")
					{
						double traversability = element.second.nodeType->currentFeature.traversability.gv;
						if (boost::geometry::within(point, element.second.Polygon))
						{
							int val = floor(traversability * (255) / 100);
							values_img.push_back(val);
							values.push_back(element.second.nodeType->currentFeature.index);
						}
					}
					else
					{
						values.push_back(def_val);
						values_img.push_back(def_val_img);
					}
					//fprintf(pgmimg, "%d", def_val);
				}
				val_pgm = *max_element(values.begin(), values.end());
				val_pgm_img = *max_element(values_img.begin(), values_img.end());

				//fprintf(pgmimg, "%d ", val);
				//break;

				//std::cout << (element.first) << "trav � " << traversability << endl;
			}
			else
			{
				val_pgm = def_val;
				val_pgm_img = def_val_img;
			}
			fprintf(pgmimg, "%d", val_pgm);
			fprintf(pgmimg, "\n");
			fprintf(pgmimg1, "%d", val_pgm_img);
			fprintf(pgmimg1, "\n");

			map_og.push_back(val_pgm);
			img_pgm.push_back(val_pgm_img);
		}
	}
	cout << "step2" << endl;
	fclose(pgmimg);
	fclose(pgmimg1);

	op.a = map_og;
	op.b = img_pgm;
	cout << "step3" << endl;

	op.ori[0] = xmin;
	op.ori[1] = ymin;
return(op);
}