#include "../include/sa_semantic_map/SemanticMap.h"
using namespace std;
using boost::geometry::append;

// offset the generated map by (offset_x, offset_y) - primarily to be used for debugging. nominal values should (0.0, 0.0)
float offset_x = 1.0;
float offset_y = 1.0;
float offset_z = 0.0;



SemMap::SemMap() {

    //sa.fm = graph();
    //sa.sm = graph();
    l_ti = 0;
    env.origin.set<0>(0);
    env.origin.set<1>(0);
    env.origin.set<2>(0);
    env.Tamb = 25;
    fm_dirty = 1;
    DTz_dirty = 1;
    env_dirty = 1;
}


bool SemMap::VisibilityQuery(point_type pt){
    int visit = 0;
    for (std::pair<std::string, NodeInfo> element : fm)
    {
        if ((element.first) != "Ballpark")
        {                
            string visibility = element.second.nodeType->currentFeature.sensorVisibility.fromAbove;
            if (boost::geometry::within(pt, element.second.Polygon))
            {
                cout << "   " << element.first << endl;
                visit = 1;
                if (visibility == "yes"){
                    return(true); 
                }
                else if (visibility == "no") {
                    return(false);
                }
                break;
            }        
        }
    }
    if (visit == 0){
        return(false);
    }
}

void SemMap::AddFeature(Feature inputFeature) {

    if (inputFeature.zoomLevel == 0 && inputFeature.featureName != inputFeature.parent) {
        unordered_map<string, NodeInfo>::const_iterator got = fm.find(inputFeature.parent);
        if (got == fm.end()) {
            inputFeature.zoomLevel = 0;
        }
        else {
            inputFeature.zoomLevel = got->second.zoomLevel + 1;
            fm[got->first].childNodes.push_back(inputFeature.parent);
        }
    }

    NodeInfo tempNode;
    tempNode.zoomLevel = inputFeature.zoomLevel;
    tempNode.boundary = inputFeature.shapeParams.boundary;
    tempNode.Polygon = inputFeature.shapeParams.Polygon;
    if (inputFeature.type == "featureRoad")
        tempNode.nodeType = new FeatureRoad;
    if (inputFeature.type == "featureTree")
        tempNode.nodeType = new FeatureTree;
    if (inputFeature.type == "featureTrail")
        tempNode.nodeType = new FeatureTrail;
    if (inputFeature.type == "featureRock")
        tempNode.nodeType = new FeatureRock;
    else
    {
        tempNode.nodeType = new FeatureUnknown;
        tempNode.nodeType->currentFeature = inputFeature;
    }
    fm[inputFeature.featureName] = tempNode;

    fm_dirty = 1;
}

void SemMap::CreateRestObjects() {

    CalMaxZoomLevel();
    for (int i = maxZoomLevel-2; i < maxZoomLevel - 1; i++)  // ie only for maxzoomlevel-1 and maxzoomlevel-2 pair
    {
        vector<string> nodes_0 = FindZoomLevelNodes(i);
        vector<string> nodes_1 = FindZoomLevelNodes(i+1);
        for (int j = 0; j < nodes_0.size(); j++) 
        {
            polygon polygon_0;

            polygon_0 = fm[nodes_0[j]].Polygon;
            std::vector<polygon> diffPolygons;
            std::vector<polygon> tempdiffPolygon;
            std::vector<polygon> tempdiffPolygon2;
            float areaSum = 0;
            
            std::ofstream svg("my_map.svg");
            boost::geometry::svg_mapper<point_type> mapper(svg, 1000, 1000);
            mapper.add(polygon_0);
            mapper.map(polygon_0, "fill-opacity:0.5;fill:rgb(153,204,0);stroke:rgb(153,204,0);stroke-width:2", 5);
           
            for (int k = 0; k < nodes_1.size(); k++)
            {
                polygon tempPolygon;
                tempPolygon = fm[nodes_1[k]].Polygon;
                areaSum += boost::geometry::area(tempPolygon);
                if (k == 0)
                    boost::geometry::difference(polygon_0, tempPolygon, diffPolygons);
                else
                {
                    int idx_break;
                    for(int n = 0; n < diffPolygons.size(); n++)
                    {
                        idx_break = n; 
                        polygon temp2poly = diffPolygons[n];
                        boost::geometry::difference(temp2poly, tempPolygon, tempdiffPolygon);
                        
                        if (boost::geometry::equals(tempdiffPolygon[0], temp2poly) == false) 
                            break;
                        tempdiffPolygon.clear();
                    }
                    for (int n = 0; n < diffPolygons.size(); n++)
                    {
                        if (n == idx_break)
                        {
                            for (int t = 0; t < tempdiffPolygon.size(); t++)
                                tempdiffPolygon2.push_back(tempdiffPolygon[t]);
                        }
                        else
                            tempdiffPolygon2.push_back(diffPolygons[n]);
                    }
                    diffPolygons.clear();
                    for (int t = 0; t < tempdiffPolygon2.size(); t++)
                        if(boost::geometry::area(tempdiffPolygon2[t]) > 0.000009 )
                            diffPolygons.push_back(tempdiffPolygon2[t]);

                }
                tempdiffPolygon2.clear();
                tempdiffPolygon.clear();
            }
            
            if (abs(areaSum) < abs(boost::geometry::area(polygon_0))) {
                
                for (int m = 0; m < diffPolygons.size(); m++) {

                    mapper.add(diffPolygons[m]);
                    mapper.map(diffPolygons[m], "fill-opacity:0.7;fill:rgb(153,204,0);stroke:rgb(153,204,0);stroke-width:1", 1);
                    if (m == 2) {
                        //cout << "area_ballpark = " << boost::geometry::area(polygon_0) << endl;
                        //cout << "area_small = " << boost::geometry::area(diffPolygons[m]) << endl;
                        mapper.add(diffPolygons[m]);
                        mapper.map(diffPolygons[m], "fill-opacity:1;fill:rgb(51,51,153);stroke:rgb(153,204,0);stroke-width:1", 1);
                    }
                    // one for loop to be implemented TODO
                    NodeInfo tempNode;
                    tempNode.nodeType = new FeatureUnknown;
                    tempNode.boundary = diffPolygons[m].outer();
                    tempNode.Polygon = diffPolygons[m];
                    tempNode.zoomLevel = i + 1;
                    string restName = "rest_" + to_string(i+1) + "_" + to_string(j) + "_" + to_string(m);// create name for rest TODO
                    fm[restName] = tempNode;
                }
            }
   
        }
    }
}

void SemMap::CalMaxZoomLevel() {
    int tempMaxZoomLevel = 0;
    for (std::pair<std::string, NodeInfo> element : fm)
    {
        if (element.second.zoomLevel > tempMaxZoomLevel){
            tempMaxZoomLevel = element.second.zoomLevel;
        }
    }
    maxZoomLevel = tempMaxZoomLevel;
}

vector<string> SemMap::FindZoomLevelNodes(int zoomLevel) {
    vector<string> nodesList;
    for (std::pair<std::string, NodeInfo> element : fm)
    {
        if (element.second.zoomLevel == zoomLevel) {
            nodesList.push_back(element.first);
        }
    }
    return nodesList;
}

void SemMap::AddTerrainInfo(int x, int y, int zoomLevel) {
    l_ti++;
    TerrainInfo tempTerrainInfo;
    tempTerrainInfo.origin.set<0>(x);
    tempTerrainInfo.origin.set<1>(y);
    tempTerrainInfo.zoomLevel = zoomLevel;
    
    // one part remaining TODO

    allTerrainInfo.push_back(tempTerrainInfo);
      
}

void SemMap::CreateMap() {
    if(fm_dirty)
        CreateRestObjects();
    // else
        // DTz = CreateMesh(fm);
    
}

void SemMap::UpdatePtsConstraints(){  
    points.clear();
    edges.clear();  
    V2d pt;
    int featureCounter = 0;
    for (std::pair<std::string, NodeInfo> element : fm)
    {
        int sizeBoundary = element.second.boundary.size();
        featureCounter = points.size();
        for (int i = 0; i < sizeBoundary; i++) {
            pt.x = element.second.boundary[i].get<0>();
            pt.y = element.second.boundary[i].get<1>();
            points.push_back(pt);
            if (i != 0) {
                Edge eg(i - 1 + featureCounter, i + featureCounter);
                edges.push_back(eg);
            }
            if (i == sizeBoundary - 1) {
                Edge eg(i + featureCounter, featureCounter);
                edges.push_back(eg);
            }
        }
    }
}

void SemMap::CreateMesh(){
    
    std:vector<triangle_node> SM_Node;
    UpdatePtsConstraints();
    cdt = updateCDT(cdt, points, edges);

    point_type centroid;
    std::vector<bool> inBallpark;

    for (std::pair<std::string, NodeInfo> element : fm){
        element.second.consistsOf.clear();
    }

    cout << " DTM SIZE :::::: " << dtm.size() << endl;
    cout << " cd size ::::::  " << cdt.triangles.size() << endl;

    for (int i = 0; i < cdt.triangles.size(); i++) 
    {
        DelaunayTriangulation temp_info;
        inBallpark.push_back(false);
        int a = cdt.triangles[i].vertices[0];
        int b = cdt.triangles[i].vertices[1];
        int c = cdt.triangles[i].vertices[2];

        float x1 = cdt.vertices[a].pos.x;
        float y1 = cdt.vertices[a].pos.y;
        float x2 = cdt.vertices[b].pos.x;
        float y2 = cdt.vertices[b].pos.y;
        float x3 = cdt.vertices[c].pos.x;
        float y3 = cdt.vertices[c].pos.y;

        point_type3D vertex1;
        point_type3D vertex2;
        point_type3D vertex3;

        vertex1.set<0>(x1);
        vertex1.set<1>(y1);
        vertex1.set<2>(0);
        vertex2.set<0>(x2);
        vertex2.set<1>(y2);
        vertex2.set<2>(0);
        vertex3.set<0>(x3);
        vertex3.set<1>(y3);
        vertex3.set<2>(0);

        temp_info.neighbors[0] = cdt.triangles[i].neighbors[0];
        temp_info.neighbors[1] = cdt.triangles[i].neighbors[1];
        temp_info.neighbors[2] = cdt.triangles[i].neighbors[2];

        /*if ((x1) > x_max.get<0>() || (x2) > x_max.get<0>() || (x3) > x_max.get<0>()) 
            continue;
        
        if ((x1) < x_min.get<0>() || (x2) < x_min.get<0>() || (x3) < x_min.get<0>()) 
            continue;
        if ((y1) > y_max.get<1>() || (y2) > y_max.get<1>() || (y3) > y_max.get<1>())
            continue;

        if ((y1) < y_min.get<1>() || (y2) < y_min.get<1>() || (x3) < y_min.get<1>())
            continue;*/

        std::vector<point_type> temp;
        temp.push_back(point_type(x1, y1));
        temp.push_back(point_type(x2, y2));
        temp.push_back(point_type(x3, y3));
        temp.push_back(point_type(x1, y1));

        polygon polygon_0;
        append(polygon_0, temp);
        boost::geometry::centroid(polygon_0, centroid);
        int index_traingle = 0;
        int count = 0;

        for (std::pair<std::string, NodeInfo> element : fm)
        {
            if (boost::geometry::within(centroid, element.second.Polygon) && element.second.nodeType->currentFeature.featureName != "Ballpark")
            {
                element.second.consistsOf.push_back(i);
                triangle_node temp_node;
                temp_node.centroid = centroid;
                temp_node.feature = element.first;
                temp_node.vertexIDs[0] = a;
                temp_node.vertexIDs[1] = b;
                temp_node.vertexIDs[2] = c;
                temp_node.vertexCoordinates[0] = vertex1;
                temp_node.vertexCoordinates[1] = vertex2;
                temp_node.vertexCoordinates[2] = vertex3;
                temp_node.Polygon = polygon_0;
                temp_node.name = element.first  + "." + std::to_string(i);

                SM_Node.push_back(temp_node);
                index_traingle = count;
                
                inBallpark[i] = true;
                temp_info.triangle_id = i;
                temp_info.Triangle_info = temp_node;
                dtm.push_back(temp_info);
                
                break;
            }
            count++;
        }
    }
    cout << " DTM SIZE 2nd :::::: " << dtm.size() << endl;

    for(int jk = 0 ; jk < dtm.size(); jk++){
        
        //cout << dtm[jk].Triangle_info.feature <<endl;
        //cout << "Nigh    " << jk << ",   " ;
        
        if (inBallpark[dtm[jk].neighbors[0]] == false){
            //cout << "1 outside" << endl;
            dtm[jk].neighbors[0] = -1;
        }
        if (inBallpark[dtm[jk].neighbors[1]] == false){
            // cout << "2 outside" << endl;
            dtm[jk].neighbors[1] = -1;
        }
        if (inBallpark[dtm[jk].neighbors[2]] == false){
            // cout << "3 outside" << endl;
            dtm[jk].neighbors[2] = -1;
        }
        sort(dtm[jk].neighbors.begin(), dtm[jk].neighbors.end(), greater<int>());
        // if (dtm[jk].neighbors[0]!= -1)
        //     cout << dtm[jk].Triangle_info.vertexCoordinates[0].get<0>() <<"  1,   " ;
        // if (dtm[jk].neighbors[1]!= -1)
        //     cout << dtm[jk].neighbors[1] <<"  2,   " ;
        // if (dtm[jk].neighbors[2]!= -1)
        //     cout << dtm[jk].neighbors[2] << "  3" << endl ; 

    } 
    cout << " DTM SIZE 3rd :::::: " << dtm.size() << endl;
    cout << "Triangulation Done" << endl;
}

// DelaunayTriangulation CreateMesh(std::unordered_map<std::string, NodeInfo> fm) {
//     DelaunayTriangulation DT;
//     return DT;
// }


void SemMap::CreateRestObjects_end() {
    for (int i = 0; i < 1; i++)  // ie only for maxzoomlevel-1 and maxzoomlevel-2 pair
    {
        vector<string> nodes_0 = FindZoomLevelNodes(i);
        vector<string> nodes_1 = FindZoomLevelNodes(i + 1);
        for (int j = 0; j < nodes_0.size(); j++)
        {
            polygon polygon_0;

            polygon_0 = fm[nodes_0[j]].Polygon;
            std::vector<polygon> diffPolygons;
            std::vector<polygon> tempdiffPolygon;
            std::vector<polygon> tempdiffPolygon2;
            float areaSum = 0;

            std::ofstream svg("my_map.svg");
            boost::geometry::svg_mapper<point_type> mapper(svg, 1000, 1000);
            mapper.add(polygon_0);
            mapper.map(polygon_0, "fill-opacity:0.5;fill:rgb(153,204,0);stroke:rgb(153,204,0);stroke-width:2", 5);

            for (int k = 0; k < nodes_1.size(); k++)
            {
                polygon tempPolygon;
                tempPolygon = fm[nodes_1[k]].Polygon;
                areaSum += boost::geometry::area(tempPolygon);
                if (k == 0)
                    boost::geometry::difference(polygon_0, tempPolygon, diffPolygons);
                else
                {
                    int idx_break;
                    for (int n = 0; n < diffPolygons.size(); n++)
                    {
                        idx_break = n;
                        polygon temp2poly = diffPolygons[n];
                        boost::geometry::difference(temp2poly, tempPolygon, tempdiffPolygon);

                        if (boost::geometry::equals(tempdiffPolygon[0], temp2poly) == false)
                            break;
                        tempdiffPolygon.clear();
                    }
                    for (int n = 0; n < diffPolygons.size(); n++)
                    {
                        if (n == idx_break)
                        {
                            for (int t = 0; t < tempdiffPolygon.size(); t++)
                                tempdiffPolygon2.push_back(tempdiffPolygon[t]);
                        }
                        else
                            tempdiffPolygon2.push_back(diffPolygons[n]);
                    }
                    diffPolygons.clear();
                    for (int t = 0; t < tempdiffPolygon2.size(); t++)
                        if (boost::geometry::area(tempdiffPolygon2[t]) > 0.000009)
                            diffPolygons.push_back(tempdiffPolygon2[t]);

                }
                tempdiffPolygon2.clear();
                tempdiffPolygon.clear();
            }

            if (abs(areaSum) < abs(boost::geometry::area(polygon_0))) {

                for (int m = 0; m < diffPolygons.size(); m++) {

                    mapper.add(diffPolygons[m]);
                    mapper.map(diffPolygons[m], "fill-opacity:0.7;fill:rgb(153,204,0);stroke:rgb(153,204,0);stroke-width:1", 1);
                    if (m == 2) {
                        //cout << "area_ballpark = " << boost::geometry::area(polygon_0) << endl;
                        //cout << "area_small = " << boost::geometry::area(diffPolygons[m]) << endl;
                        mapper.add(diffPolygons[m]);
                        mapper.map(diffPolygons[m], "fill-opacity:1;fill:rgb(51,51,153);stroke:rgb(153,204,0);stroke-width:1", 1);
                    }
                    // one for loop to be implemented TODO
                    NodeInfo tempNode;
                    tempNode.nodeType = new FeatureUnknown;
                    tempNode.boundary = diffPolygons[m].outer();
                    tempNode.Polygon = diffPolygons[m];
                    tempNode.zoomLevel = i + 1;
                    string restName = "rest_" + to_string(i + 1) + "_" + to_string(j) + "_" + to_string(m);// create name for rest TODO
                    fm[restName] = tempNode;
                }
            }

        }
    }
}

/*
// === from genSWM()
        samap::grounds swmFeature;
        swmFeature.groundsName = element.second.nodeType->currentFeature.featureName;
        swmFeature.type = element.second.nodeType->currentFeature.type;

        //cout << "name: " << element.second.nodeType->currentFeature.featureName << "  zoom level = " << element.second.zoomLevel << endl;
        //cout << "size of consistsOf: " << element.second.consistsOf.size() << endl;
        for (int ichild = 0; ichild < element.second.consistsOf.size(); ichild++) 
        {
            int idx = element.second.consistsOf[ichild];
            // cout << "x[" << idx << "] = " << dtm[idx]Triangle_info.vertexCoordinates[0].get<0>() << endl;
            cout << "idx = " << idx << "ichild = " << ichild << endl;
        }


        swmFeature.traversability.gv = element.second.nodeType->currentFeature.traversability.gv;
        swmFeature.traversability.pedestrian = element.second.nodeType->currentFeature.traversability.pedestrian;
        swmFeature.traversability.av = element.second.nodeType->currentFeature.traversability.av;

        swmFeature.occupiability.gv = element.second.nodeType->currentFeature.occupiability.gv;
        swmFeature.occupiability.pedestrian = element.second.nodeType->currentFeature.occupiability.pedestrian;
        swmFeature.occupiability.av = element.second.nodeType->currentFeature.occupiability.av;

        swmFeature.observability.fromSides = element.second.nodeType->currentFeature.sensorVisibility.fromSide;
        swmFeature.observability.fromAbove = element.second.nodeType->currentFeature.sensorVisibility.fromAbove;

        swmFeature.shapeParams.origin.x = element.second.nodeType->currentFeature.shapeParams.origin.get<0>() + env.origin.get<0>() + offset_x;
        swmFeature.shapeParams.origin.y = element.second.nodeType->currentFeature.shapeParams.origin.get<1>() + env.origin.get<1>() + offset_y;
        swmFeature.shapeParams.origin.z = 0.0 + offset_z;

        swmFeature.isFeature = true;

        //std:vector<geometry_msgs::Point> vecp;
        geometry_msgs::Point p1;


        for (int i = 0; i < element.second.nodeType->currentFeature.shapeParams.boundary.size(); i++)
        {
            p1.x = element.second.nodeType->currentFeature.shapeParams.boundary[i].get<0>() + env.origin.get<0>() + offset_x;
            p1.y = element.second.nodeType->currentFeature.shapeParams.boundary[i].get<1>() + env.origin.get<1>() + offset_y;
            p1.z = 0.0 + offset_z;
            //vecp.push_back(p1);
            // cout << " boundary point [" << i << "] " << element.second.nodeType->currentFeature.shapeParams.boundary[i].get<0>();
            swmFeature.shapeParams.boundary.push_back(p1);
        }

        //swmFeature.shapeParams.boundary = vecp;
*/


/*
==== from genSWM() - assigning ground patches
        NodeInfo element = fm[dtm[jk].Triangle_info.feature];
        
        swmGroundPatch.groundsName = dtm[jk].Triangle_info.feature + "." + std::to_string(jk);
        swmGroundPatch.type = element.nodeType->currentFeature.type;

        //cout << "name: " << swmGroundPatch.groundsName << "  zoom level = " << element.zoomLevel << endl;

        swmGroundPatch.traversability.gv = element.nodeType->currentFeature.traversability.gv;
        swmGroundPatch.traversability.pedestrian = element.nodeType->currentFeature.traversability.pedestrian;
        swmGroundPatch.traversability.av = element.nodeType->currentFeature.traversability.av;

        swmGroundPatch.occupiability.gv = element.nodeType->currentFeature.occupiability.gv;
        swmGroundPatch.occupiability.pedestrian = element.nodeType->currentFeature.occupiability.pedestrian;
        swmGroundPatch.occupiability.av = element.nodeType->currentFeature.occupiability.av;

        swmGroundPatch.observability.fromSides = element.nodeType->currentFeature.sensorVisibility.fromSide;
        swmGroundPatch.observability.fromAbove = element.nodeType->currentFeature.sensorVisibility.fromAbove;

        swmGroundPatch.shapeParams.origin.x = element.nodeType->currentFeature.shapeParams.origin.get<0>() + env.origin.get<0>() + offset_x;
        swmGroundPatch.shapeParams.origin.y = element.nodeType->currentFeature.shapeParams.origin.get<1>() + env.origin.get<1>() + offset_y;
        swmGroundPatch.shapeParams.origin.z = 0.0 + offset_z;

        swmGroundPatch.isFeature = false;
        swmGroundPatch.parent = dtm[jk].Triangle_info.feature;


        //std:vector<geometry_msgs::Point> vecp;
        geometry_msgs::Point p1;

        for (int idx_v = 0; idx_v < dtm[jk].Triangle_info.vertexIDs.size(); idx_v++)
        {
            p1.x = dtm[jk].Triangle_info.vertexCoordinates[idx_v].get<0>() + env.origin.get<0>() + offset_x;
            p1.y = dtm[jk].Triangle_info.vertexCoordinates[idx_v].get<1>() + env.origin.get<1>() + offset_y;
            p1.z = dtm[jk].Triangle_info.vertexCoordinates[idx_v].get<2>() + offset_z;
            //vecp.push_back(p1);
            // cout << " boundary point [" << i << "] " << element.second.nodeType->currentFeature.shapeParams.boundary[i].get<0>();
            swmGroundPatch.shapeParams.boundary.push_back(p1);
        }

        //swmGroundPatch.shapeParams.boundary = vecp;

        swm.SWMFeatures.groundsVec.push_back(swmGroundPatch);
        swm.SWMFeatures.groundsKeyVec.push_back(swmGroundPatch.groundsName);
        swm.SWMFeatures.numGrounds++;
        // cout << "numGrounds (with GP) = " << (int) swm.SWMFeatures.numGrounds << endl;


*/

/*
=== from genSWM() extracting GCMs ...

        GCM_item.GCMName = element.second.GCMName;
        GCM_item.isReference = element.second.isReference;
        GCM_item.GCMType=element.second.GCMType;
        GCM_item.referencedEntity=element.second.referencedEntity;
        GCM_item.referenceQualifier=element.second.referenceQualifier;

        GCM_item.priority = element.second.priority;
        cout << element.second.GCMName << endl;

        GCM_item.shapeParams.origin.x = element.second.shapeParams.origin.get<0>() + env.origin.get<0>() + offset_x;
        GCM_item.shapeParams.origin.y = element.second.shapeParams.origin.get<1>() + env.origin.get<1>() + offset_y;
        GCM_item.shapeParams.origin.z = 0.0;

        //std:vector<geometry_msgs::Point> vecp;
        geometry_msgs::Point p1;


        for (int i = 0; i < element.second.shapeParams.boundary.size(); i++)
        {
            p1.x = element.second.shapeParams.boundary[i].get<0>() + env.origin.get<0>() + offset_x;
            p1.y = element.second.shapeParams.boundary[i].get<1>() + env.origin.get<1>() + offset_y;
            p1.z = 0.0 + offset_z;
            GCM_item.shapeParams.boundary.push_back(p1);
        }

        swm.GCMs.GCMVec.push_back(GCM_item);
        swm.GCMs.GCMKeyVec.push_back(GCM_item.GCMName);
        swm.GCMs.numGCMs++;



*/