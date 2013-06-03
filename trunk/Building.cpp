#include "Building.h"
#include "globals.h"
#include "polybuilder.h"

class polybuilder;
struct Plane_equation;

/*########################################################
##########################################################
The following functions are geometric
##########################################################
########################################################*/

vector<Point_3> subtractMin (vector<Point_3> points) {
	double minX = 100000000000000,
		minY = 100000000000000,
		minZ = 100000000000000;

	vector<Point_3> newPoints;

	for (int i = 0; i != points.size(); ++i ) {
		double x = CGAL::to_double(points[i].x()),
			y = CGAL::to_double(points[i].y()),
			z = CGAL::to_double(points[i].z());
		if (x < minX) minX = x;
		if (y < minY) minY = y;
		if (z < minZ) minZ = z;
	}

	for (int i = 0; i != points.size(); ++i ) {
		double x = CGAL::to_double(points[i].x()),
			y = CGAL::to_double(points[i].y()),
			z = CGAL::to_double(points[i].z());
		x -= minX;
		y -= minY;
		z -= minZ;
		newPoints.push_back(Point_3(x,y,z));
	}
	return newPoints;
}

void  determineAngles (double & theta, double & phi, vector<Point_3> points, bool deg = false) {
	double const pi = 3.14159265359;
	points = subtractMin(points);
	Triangle_3 tri(points[0],points[1],points[2]);
	Transformation scale (CGAL::SCALING, 4);
	tri.transform(scale);
	Vector_3 normal = tri.supporting_plane().orthogonal_vector();

	double x = CGAL::to_double(normal.x()), y = CGAL::to_double(normal.y()), z = CGAL::to_double(normal.z());
	double r = sqrt(CGAL::to_double(normal.squared_length()));

	theta = asin(z/r);

	if (theta != theta) {
		if (z/r < -1) theta = -pi/2;
		else if (z/r > 1) theta = pi/2;
		else theta = 0;
	}

	if (abs(x) > 1e-6 && abs(y) > 1e-6) {
		phi = atan2(x,y);
	} else phi = 0;

	if (deg) {
		theta *= 180/pi;
		phi *= 180/pi;
	}
}

double Building::getMinHeight() {
	double minZ = 1000000;
	Polyhedron::Vertex_iterator vit;
	for (vit = polyBuilding.vertices_begin(); vit != polyBuilding.vertices_end(); vit++) {
		double z = CGAL::to_double(vit->point().z());
		if (z < minZ) minZ = z;
	}
	return minZ;
}

double Building::getMaxHeight() {
	double maxZ = -1000000;
	Polyhedron::Vertex_iterator vit;
	for (vit = polyBuilding.vertices_begin(); vit != polyBuilding.vertices_end(); vit++) {
		double z = CGAL::to_double(vit->point().z());
		if (z > maxZ) maxZ = z;
	}
	return maxZ;
}

double Building::getHeight() {
	/*Gets the height of the building relative to zero*/
	double minZ = getMinHeight(), maxZ = getMaxHeight();
	return maxZ-minZ;
}

vector<Point_2>  Building::getConvexHull2D() {
	/* Projects all the points of the building onto the x-y plane
	and calculates the convex hull of that*/ 
	int nVert = polyBuilding.size_of_vertices();
	vector<Point_2> points2;
	vector<Point_2> hull;
	Polyhedron::Vertex_iterator vit;
	for (vit = polyBuilding.vertices_begin(); vit != polyBuilding.vertices_end(); ++vit) {
		Point_3 pnt = vit->point();
		points2.push_back(Point_2(pnt.x(), pnt.y()));
	}

	CGAL::convex_hull_2(points2.begin(),points2.end(),back_inserter(hull));
	return hull;
}

Polyhedron Building::createSplitPolyhedron (vector<Point_2> points, double minHeight, double maxHeight) {
	/* Creates a polyhedron from a convex hull and a minimum and a maximum height*/
	Polyhedron poly;
	vector<Point_3> points3;
	for (int i = 0; i != points.size(); ++i) {
		points3.push_back(Point_3(points[i].x(),points[i].y(),minHeight));
		points3.push_back(Point_3(points[i].x(),points[i].y(),maxHeight));
	}
	CGAL::convex_hull_3(points3.begin(),points3.end(),poly);
	return poly;
}

void Building::createUnitBox () {
	stringstream robot;
	robot << "OFF\n8 6 0\n-1 1 1\n-1 -1 1\n1 1 1\n1 -1 1\n-1 1 -1\n-1 -1 -1\n" <<
		"1 1 -1\n1 -1 -1\n4 2 3 1 0\n4 4 6 2 0\n4 1 5 4 0\n4 6 4 5 7\n4 3 2 6 7\n4 5 1 3 7\n"; 
	//ifstream robot("D:\\Dropbox\\Data\\robot.off");
	CGAL::OFF_to_nef_3(robot,nefBox);
	//robot.close();
}

Nef_polyhedron Building::makeRobot (string polyType, vector<Point_3> points) {
	Nef_polyhedron box = nefBox;
	double length, theta, phi;

	if (polyType == "CeilingSurface") length = ceilingThickness;
	else if (polyType == "RoofSurface") length = roofThickness;
	else if (polyType == "WallSurface")  length = wallThickness;
	else if (polyType == "InnerWallSurface") length = sharedWallThickness;
	else if (polyType == "FloorSurface") length = floorThickness;
	else if (polyType == "GroundSurface") length = 0.01;

	determineAngles(theta, phi, points);
	theta = - theta; //Pitch downwards is defined as positive rotation

	Transformation scale(CGAL::SCALING, length);
	
	Transformation rot_z (cos(phi),sin(phi),0,
		-sin(phi),cos(phi),0,
		0,0,1);

	Transformation rot_x (1, 0, 0,
		0, cos(theta), sin(theta),
		0, -sin(theta), cos(theta));

	box.transform(scale);
	box.transform(rot_x);
	box.transform(rot_z);
	return box;
}

string Building::determinePolyType (vector<Point_3> points, bool outer = false, int floor=0) {
	string polyType;
	double theta,phi;
	determineAngles(theta,phi,points, true);
	if (!outer) { //Final classification for CityGML output
		if (theta < -70) {
			polyType = "FloorSurface";
		} else if ( abs(theta) < 5) {
			polyType = "InteriorWallSurface";
		} else polyType = "CeilingSurface";
	} else { //First classification for right offset
		if (theta < -70) {
			if (floor == 0) polyType = "GroundSurface";
			else polyType = "FloorSurface";
		} else if ( abs(theta) < 5) {
			polyType = "WallSurface";
		} else if (theta > 85 && theta < 95) {
			if (floor == highFloor) polyType = "RoofSurface";
			else polyType = "CeilingSurface";
		}
		else polyType = "RoofSurface";
	}

	if (polyType == "WallSurface") { //Check whether a wall is a shared wall
		if (sharedWalls.size()>0) {
			vector<Line_2>::iterator it;
			vector<Point_3>::iterator pit;
			for (it = sharedWalls.begin(); it != sharedWalls.end(); ++it){
				Line_2 line = *it;
				int pointsTrue = 0;
				for (pit = points.begin(); pit != points.end(); ++pit) {
					double px = CGAL::to_double(pit->x()),
						py = CGAL::to_double(pit->y());
					Point_2 P(px,py);
					if (CGAL::squared_distance(line,P)<0.01) {
						++pointsTrue;
					}
				}
				if (pointsTrue == 3) {
					polyType = "InnerWallSurface";
					break;
				}
			}
		}
	}
	return polyType;
}

string determineExtPolyType (vector<Point_3> points) {
	string polyType;
	double theta,phi;
	determineAngles(theta,phi,points, true);
	if (theta < -70) polyType = "GroundSurface";
	else if ( abs(theta) < 5) polyType = "WallSurface";
	else polyType = "RoofSurface";
	return polyType;
}

void Building::exteriorToFinalPolygons () {
	string polyType;
	Polyhedron::Facet_iterator fit;
	Polyhedron::Facet::Halfedge_around_facet_circulator itHDS;
	int j = 0;
	for (fit = polyBuilding.facets_begin(); fit != polyBuilding.facets_end(); ++fit) {
		Vector_3 normal = fit->plane().orthogonal_vector();
		vector<Point_3> tempPoints;

		itHDS = fit->facet_begin();
		do {
			typedef Polyhedron::Vertex::Point_3 vertex;
			vertex v = itHDS->vertex()->point();
			tempPoints.push_back(v);
		} while (++itHDS != fit->facet_begin());
		polyType = determineExtPolyType(tempPoints);
		finalPolygons["exterior"][0][polyType][j] = tempPoints;
		++j;
	}
}

void Building::addPolyType (int i, bool finalStorey=false) {
	string polyType;
	Polyhedron::Facet_iterator fit;
	Polyhedron::Facet::Halfedge_around_facet_circulator itHDS;
	int j = 0;
	for (fit = storey.facets_begin(); fit != storey.facets_end(); ++fit) {
		Vector_3 normal = fit->plane().orthogonal_vector();
		vector<Point_3> tempPoints;

		itHDS = fit->facet_begin();
		do {
			typedef Polyhedron::Vertex::Point_3 vertex;
			vertex v = itHDS->vertex()->point();
			tempPoints.push_back(v);
		} while (++itHDS != fit->facet_begin());
		polyType = determinePolyType(tempPoints);
		finalPolygons["interior"][i][polyType][j] = tempPoints;
		++j;
	}
}

void Building::setThickness(double wall,double sharedWall,double floorCeiling) {
	wallThickness = wall/100.00;
	sharedWallThickness = sharedWall/100.00;
	ceilingThickness = floorCeiling/2.00;
	floorThickness = floorCeiling/2.00;
	roofThickness = 0.30;
}

void Building::removeExtrudedFacets (Nef_polyhedron& Nef_storey, int floor) {
	Polyhedron::Facet::Halfedge_around_facet_circulator itHDS;
	Polyhedron::Facet_iterator fit;
	//std::transform( storey.facets_begin(), storey.facets_end(), storey.planes_begin(), Plane_equation()); 
	for (fit = storey.facets_begin(); fit != storey.facets_end(); ++fit) {
		itHDS = fit->facet_begin();
		//Vector_3 normal = fit->plane().orthogonal_vector();

		vector<Point_3> tempPoints;
		do {
			typedef Polyhedron::Vertex::Point_3 vertex;
			vertex v = itHDS->vertex()->point();
			tempPoints.push_back(v);
		} while (++itHDS != fit->facet_begin());

		if (!isDegen(tempPoints,1e-2)){ // Check degeneracy, if degenerate continue with new face) {
			string polyType = determinePolyType(tempPoints,true,floor);
			Nef_polyhedron face(tempPoints.begin(),tempPoints.end()); 
			Nef_polyhedron robot = makeRobot(polyType,tempPoints);
			Nef_polyhedron diff = CGAL::minkowski_sum_3(robot,face);
			//createConvexHullNef(diff);
			Nef_storey -= diff;
		} 
	}
}

void Building::determineAreaNetHeight(int i){
	Nef_polyhedron Nef_storey(storey);
	//writeOffFile(Nef_storey,"D:\\Dropbox\\Data\\Nef_storey.off");
	map<int,vector<Point_3>> floorPolygons = finalPolygons["interior"][i]["FloorSurface"];
	map<int,vector<Point_3>>::iterator fit;
	for (fit = floorPolygons.begin(); fit != floorPolygons.end(); ++fit){
		vector<Point_3> pointsVector = fit->second;
		vector<Point_3> tempPoints;
		vector<Point_3>::iterator vit;
		tempPoints.clear();
		for (vit = pointsVector.begin(); vit != pointsVector.end(); ++ vit) {
			double x = CGAL::to_double(vit->x()), y = CGAL::to_double(vit->y()), z = CGAL::to_double(vit->z());
			double newZ = z + 1.5;
			tempPoints.push_back(*vit);
			tempPoints.push_back(Point_3(x,y,newZ));
		}
		Polyhedron polyHull;
		CGAL::convex_hull_3(tempPoints.begin(),tempPoints.end(),polyHull);
		Nef_polyhedron nefHull (polyHull);
		//writeOffFile(nefHull,"D:\\Dropbox\\Data\\extrtriangle.off");

		Nef_polyhedron diff = nefHull - Nef_storey;
		vector<Nef_polyhedron> diffs = splitNefs(diff);
		vector<Nef_polyhedron>::iterator vnit;
		for (vnit = diffs.begin(); vnit != diffs.end(); ++ vnit) {
			if (vnit->is_simple()){
				Polyhedron polyDiff;
				vnit->convert_to_polyhedron(polyDiff);
				Polyhedron::Facet::Halfedge_around_facet_circulator itHDS;
				Polyhedron::Facet_iterator pit;
				vector<Point_3> trianglePoints;
				for (pit = polyDiff.facets_begin(); pit != polyDiff.facets_end(); ++pit) {
					trianglePoints.clear();
					itHDS = pit->facet_begin();
					do {
						typedef Polyhedron::Vertex::Point_3 vertex;
						vertex v = itHDS->vertex()->point();
						trianglePoints.push_back(v);
					} while (++itHDS != pit->facet_begin());
					Point_2 p1(trianglePoints[0].x(),trianglePoints[0].y());
					Point_2 p2(trianglePoints[1].x(),trianglePoints[1].y());
					Point_2 p3(trianglePoints[2].x(),trianglePoints[2].y());
					Triangle_2 tempTri(p1,p2,p3);

					if (! tempTri.is_degenerate()) {
						double tArea = CGAL::to_double(tempTri.area());
						//cout << tArea << endl;

						netHeightArea += abs(tArea)/2.0;

					}
				}
			}
		}
	}
	/*if (Nef_storey.is_simple()) {
	Nef_storey.convert_to_polyhedron(storey);
	std::transform( storey.facets_begin(), storey.facets_end(), storey.planes_begin(),Plane_equation()); 
	Nef_storey.clear();
	}*/
	//cout << endl << endl << netHeightArea << endl << endl;
}

vector<double> Building::getMarkedHeights() {
	vector<double> markedHeights;
	double maxHeight = getMaxHeight(), minRoofHeight= 10000000;

	map<int,vector<Point_3>> roofPolygons = finalPolygons["exterior"][0]["RoofSurface"];
	map<int,vector<Point_3>>::iterator itRP;
	for (itRP = roofPolygons.begin(); itRP != roofPolygons.end(); ++itRP) {
		vector<Point_3> tempPoints = itRP->second;
		vector<Point_3>::iterator pointIT;
		Point_3 pnt;
		vector<Point_3> tempPlanePoints;
		tempPlanePoints.clear();
		double tempMinRoofHeight = 10000;
		for (pointIT = tempPoints.begin(); pointIT != tempPoints.end(); ++pointIT) {
			pnt = *pointIT;
			tempPlanePoints.push_back(pnt);
			if (CGAL::to_double(pnt.z()) < tempMinRoofHeight) tempMinRoofHeight = CGAL::to_double(pnt.z());
		}
		vector<Point_3> tempPlane (tempPlanePoints.begin(), tempPlanePoints.begin()+3);
		double theta, phi;
		determineAngles(theta, phi, tempPlane,true);
		if ((abs(theta)/90.0) > 0.90 && (abs(theta)/90.0) < 1.10) {
			if ((abs(tempMinRoofHeight/maxHeight)*100) < 90 ) {
				markedHeights.push_back(tempMinRoofHeight);
			}
		}
		else if (tempMinRoofHeight < minRoofHeight) minRoofHeight = tempMinRoofHeight;

	}
	markedHeights.push_back(minRoofHeight);
	return markedHeights;
}

vector<double> Building::setSplitHeights () {
	
	double levels = storeys;
	double height = getHeight()-roofThickness;
	double minHeight = getMinHeight();
	double maxHeight = getMaxHeight();
	
	vector<double> storeyHeights;
	for (int i = 0; i != storeys; ++i) {
		double lower = i, upper = i+1;
		double lowHeight = lower/levels*height+minHeight, highHeight = upper/levels*height + minHeight;
		storeyHeights.push_back(lowHeight);
		if (i == (storeys - 1)) storeyHeights.push_back(maxHeight);
	}

	vector<double> markedHeights = getMarkedHeights();
	vector<double>::iterator vdIT, vdIT2, vdIT3;
	double lastHeight = 0;
	int nrOfStorToChange = 0;
	int counter = 0;
	for (vdIT = storeyHeights.begin(); vdIT != storeyHeights.end()-1; ++vdIT) {
		for (vdIT2 = markedHeights.begin(); vdIT2 != markedHeights.end(); ++ vdIT2){
			if ((*vdIT - *vdIT2)<0.5 && (*vdIT - *vdIT2)>-0.5)  {
				*vdIT = *vdIT2;
				double curHeight = *vdIT;
				for (vdIT3 = storeyHeights.begin(); vdIT3!= vdIT+1; ++vdIT3){
					if (*vdIT3 > (lastHeight+0.05) && *vdIT3 < (curHeight-0.05)) {
						++nrOfStorToChange;
					}
				}
				int k = 0;
				double newLowStoreyHeight = (curHeight - lastHeight)/ static_cast<double>(nrOfStorToChange+1);
				double newHighStoreyHeight = (maxHeight - curHeight - roofThickness)/ static_cast<double>(storeys-counter);
				for (vdIT3 = storeyHeights.begin(); vdIT3!= storeyHeights.end(); ++vdIT3){
					if (*vdIT3 > (lastHeight+0.05) && *vdIT3 < (curHeight-0.05)) {
						*vdIT3 = lastHeight + static_cast<double>(k)*newLowStoreyHeight;
					} else if (*vdIT3 > (curHeight+0.05) && *vdIT3 < (height - 0.05)) {
						*vdIT3 = curHeight + static_cast<double>(k-counter)*newHighStoreyHeight;
					}
					++k;
				}
				lastHeight = curHeight;
			}
		}
		++counter;
	}
	return storeyHeights;
}

void Building::createLoD2Plus() {
	createUnitBox (); // Create robot box of size 2
	netHeightArea = 0;
	vector<Nef_polyhedron> firstNefs, secondNefs;
	vector<Nef_polyhedron>::iterator nefIt1, nefIt2;
	vector<double> storeyHeights = setSplitHeights();
	vector<Point_2> hull = getConvexHull2D();
	vector<double>::iterator vdIT;
	int i = 0, j = 0;

	for (vdIT = storeyHeights.begin(); vdIT!= storeyHeights.end()-1;++vdIT) {
		double lowHeight = *vdIT, highHeight = *(vdIT+1);
		Polyhedron tempPoly = createSplitPolyhedron(hull,lowHeight,highHeight);
		Nef_polyhedron tempNef(tempPoly);
		Nef_polyhedron Nef_storeys = Nef_building*tempNef; //Intersect complete building with storey convex hull
		firstNefs = splitNefs(Nef_storeys); //Intersection may produce separated volumes, handle separately
		for (nefIt1 = firstNefs.begin(); nefIt1 != firstNefs.end(); ++ nefIt1) {
			storey.clear();
			nefIt1->regularization().convert_to_polyhedron(storey);
			storey.keep_largest_connected_components(1);
			int floor = lowFloor+i;
			removeExtrudedFacets((*nefIt1),floor);
			
			secondNefs = splitNefs(*nefIt1); //Erosion may produce separated volumes, handle separately
			for (nefIt2 = secondNefs.begin(); nefIt2 != secondNefs.end(); ++ nefIt2) {
				storey.clear();
				nefIt2->regularization().convert_to_polyhedron(storey);
				storey.keep_largest_connected_components(1);
				//fix_degenerate(storey);
				addPolyType(j,true);
				determineAreaNetHeight(j);
				++j;
			}
		}
		cout << "."; ++i;
	}
}

void Building::calcArea() {
	area = 0;
	map<int,map<string,map<int,vector<Point_3>>>> interiorPolygons = finalPolygons["interior"];
	map<int,map<string,map<int,vector<Point_3>>>>::iterator itShells;
	map<string,map<int,vector<Point_3>>>::iterator itPolyType;
	map<int,vector<Point_3>>::iterator itPolyNR;
	for (itShells = interiorPolygons.begin(); itShells != interiorPolygons.end(); ++itShells) {
		map<string,map<int,vector<Point_3>>> shells = itShells->second;
		for (itPolyType = shells.begin(); itPolyType != shells.end(); ++itPolyType){
			string type = itPolyType -> first;
			map<int,vector<Point_3>> polyTypes = itPolyType->second;
			if (type == "FloorSurface") {
				for (itPolyNR = polyTypes.begin(); itPolyNR != polyTypes.end(); ++ itPolyNR) {
					vector<Point_3> points = itPolyNR->second;
					Triangle_3 t(points[0],points[1],points[2]);
					if (!t.is_degenerate()) {
						double tarea = sqrt(CGAL::to_double(t.squared_area()));
						if (tarea<1000) {
							area += tarea;
						}
					}
				}
			}
		}
	}
	area-=netHeightArea;
}

//void Building::calcGrossVolume() {
//	Nef_building.regularization();
//	if(Nef_building.is_simple()) {
//		try {
//			volume = 0;
//			Polyhedron building;
//			Nef_building.convert_to_polyhedron(building);
//			Point_3 inf(-1000, -1000, -1000);
//			Polyhedron::Facet::Halfedge_around_facet_circulator itHDS;
//			Polyhedron::Facet_iterator fit;
//			for (fit = building.facets_begin(); fit != building.facets_end(); ++fit) {
//				itHDS = fit->facet_begin();
//				vector<Point_3> tempPoints;
//				do {
//					typedef Polyhedron::Vertex::Point_3 vertex;
//					vertex v = itHDS->vertex()->point();
//					tempPoints.push_back(v);
//				} while (++itHDS != fit->facet_begin());
//				CGAL::Tetrahedron_3<Kernel> t(tempPoints[0],tempPoints[1],tempPoints[2],inf);
//				if (!t.is_degenerate()) {
//					double tvolume = -CGAL::to_double(t.volume());
//					volume += tvolume;
//				}
//			}
//		} catch (...) {
//			volume = 9999;
//		}
//	} else volume = 9999;
//}

/*########################################################
##########################################################
The following functions contain methods to write buildings
to CityGML! 
##########################################################
########################################################*/

void Building::writeBuildingHeader(ofstream& output){
	tabs = 0;
	output	<<	stringIndents(1)	<<	"<cityObjectMember>\n";
	output	<<	stringIndents(1)			<<	"<bldg:Building gml:id=\"BAG" << bagID << "\">\n";
	output	<<	stringIndents(1)		<<	"<gml:name>" << bagID << "</gml:name>\n";
	output	<< stringIndents(0)		<<	"<gen:doubleAttribute name=\"floorArea\" xmlns:gen=\"http://www.opengis.net/citygml/generics/1.0\">\n\t\t\t\t<gen:value>"<< static_cast<int>(area) << "</gen:value>\n\t\t\t</gen:doubleAttribute>\n";
	output	<< stringIndents(0)		<<	"<gen:doubleAttribute name=\"BAG use Area\" xmlns:gen=\"http://www.opengis.net/citygml/generics/1.0\">\n\t\t\t\t<gen:value>"<< bagUseArea << "</gen:value>\n\t\t\t</gen:doubleAttribute>\n";
	//output	<< stringIndents(0)		<<	"<gen:doubleAttribute name=\"Gross Building Volume\" xmlns:gen=\"http://www.opengis.net/citygml/generics/1.0\">\n\t\t\t\t<gen:value>"<< static_cast<int>(volume) << "</gen:value>\n\t\t\t</gen:doubleAttribute>\n";
	output	<< stringIndents(0)		<<	"<gen:doubleAttribute name=\"Net height area\" xmlns:gen=\"http://www.opengis.net/citygml/generics/1.0\">\n\t\t\t\t<gen:value>"<< static_cast<int>(netHeightArea) << "</gen:value>\n\t\t\t</gen:doubleAttribute>\n";
	output	<<	stringIndents(0)	<<	"<gml:MultiSolid>\n";
}

void Building::writeGeometry(ofstream& output){
	itBuildingPolys itBP;
	map<string,int> counter;
	counter["RoofSurface"] = 0;
	counter["WallSurface"] = 0;
	counter["CeilingSurface"] = 0;
	counter["InteriorWallSurface"] = 0;
	counter["FloorSurface"] = 0;
	counter["GroundSurface"] = 0;
	map<string,map<int,map<string,map<int,vector<Point_3>>>>>::iterator itPolys;
	++tabs;
	for (itPolys = finalPolygons.begin(); itPolys != finalPolygons.end(); ++ itPolys) { // interior / exterior
		string extint = itPolys->first;
		map<int,map<string,map<int,vector<Point_3>>>> extintPolys = itPolys->second;
		map<int,map<string,map<int,vector<Point_3>>>>::iterator itEIP;
		
		for (itEIP = extintPolys.begin(); itEIP != extintPolys.end(); ++itEIP) { // shell number
			int shellNR = itEIP->first;
			map<string,map<int,vector<Point_3>>> bldng = itEIP->second;
			map<string,map<int,vector<Point_3>>>::iterator itEP;

			output << stringIndents(0) << "<gml:solidMember>\n";
			if (extint == "interior"){
				output << stringIndents(1) << "<bldg:Storey>\n";
			} else {

			}
			output << stringIndents(1) << "<bldg:lod2Solid>\n";
			output << stringIndents(1) << "<gml:Solid>\n";
			output << stringIndents(1) << "<gml:exterior>\n";

			output << stringIndents(1) << "<gml:CompositeSurface srsName=\"EPSG:28992\" srsDimension=\"3\">\n";
			tabs+=1;
			for (itEP = bldng.begin(); itEP != bldng.end(); ++itEP) { // string polytype
				string polyType = itEP->first;
				map<int,vector<Point_3>> polygons = itEP->second;
				map<int,vector<Point_3>>::iterator itP;
				output	<<	stringIndents(0)		<<	"<bldg:boundedBy>\n";
				output	<<	stringIndents(1)		<<	"<bldg:" << polyType << ">\n";
				++tabs;
				// Write all coordinates for all faces of the building
				for (itP = polygons.begin(); itP != polygons.end(); ++itP) { //Poly number / vector of points
					stringstream iss;
					iss << itP->first;
					vector<Point_3> pointVector = itP->second;

					if (isDegen(pointVector,1e-8)) continue; // Check degeneracy, if degenerate continue with new face

					string polyNR = iss.str();
					counter[polyType]++;
					int plNR = counter[polyType];
					string s;
					stringstream out;
					out << plNR;
					s = out.str();
					out.str(string());
					out.clear();
					string t;
					out << polyNR;
					t = out.str();
					string gmlID = bagID + "_" + polyType + "_" + t + "_" + s;
					gmlIDs[polyType].push_back(gmlID);

					output <<	stringIndents(0)		<<	"<gml:surfaceMember>\n";
					output <<	stringIndents(1)		<<	"<gml:Polygon xmlns:gml=\"http://www.opengis.net/gml\" gml:id=\"BAG" << gmlID << "\">\n";
					output <<	stringIndents(1)		<<	"<gml:exterior>\n";
					output <<	stringIndents(1)		<<	"<gml:LinearRing>\n";
					output <<	stringIndents(1)		<<	"<gml:posList>";
					int i = 0;
					double x0,y0,z0;
					vector<Point_3>::iterator itVP;
					for (itVP = pointVector.begin(); itVP != pointVector.end(); ++itVP) {
						double x = CGAL::to_double(itVP->x()), y = CGAL::to_double(itVP->y()), z = CGAL::to_double(itVP->z());
						output.precision(30);
						output << dblToStr(x,4) << " " << dblToStr(y,4) << " " << dblToStr(z,4) << " ";
						if (i == 0) {
							x0=x, y0=y, z0=z;
							i = 9;
						}
					}
					output << dblToStr(x0,4) << " " << dblToStr(y0,4) << " " << dblToStr(z0,4) << "</gml:posList>\n";
					output <<	stringIndents(-1)	<<	"</gml:LinearRing>\n";
					output <<	stringIndents(-1)		<<	"</gml:exterior>\n";
					output <<	stringIndents(-1)		<<	"</gml:Polygon>\n";
					output <<	stringIndents(-1)			<<	"</gml:surfaceMember>\n";
				}

				output	<<	stringIndents(-1)		<<	"</bldg:" << polyType << ">\n";
				output	<<	stringIndents(-1)		<<	"</bldg:boundedBy>\n";

			}

			output << stringIndents(-1) << "</gml:CompositeSurface>\n";
			output << stringIndents(-1) << "</gml:exterior>\n";
			output << stringIndents(-1) << "</gml:Solid>\n";
			output << stringIndents(-1) << "</bldg:lod2Solid>\n";
			if (extint == "interior"){
				output << stringIndents(-1) << "</bldg:Storey>\n";
			} else {

			}
			output << stringIndents(-1) << "</gml:solidMember>\n";
		}
	}
	output	<<	stringIndents(-1)	<<	"</gml:MultiSolid>\n";
}

void Building::writeGeometry_old(ofstream& output){
	itBuildingPolys itBP;
	map<string,int> counter;
	counter["RoofSurface"] = 0;
	counter["WallSurface"] = 0;
	counter["CeilingSurface"] = 0;
	counter["InteriorWallSurface"] = 0;
	counter["FloorSurface"] = 0;
	counter["GroundSurface"] = 0;
	map<string,map<int,map<string,map<int,vector<Point_3>>>>>::iterator itPolys;

	for (itPolys = finalPolygons.begin(); itPolys != finalPolygons.end(); ++ itPolys) { // interior / exterior
		string extint = itPolys->first;
		map<int,map<string,map<int,vector<Point_3>>>> extintPolys = itPolys->second;
		map<int,map<string,map<int,vector<Point_3>>>>::iterator itEIP;

		for (itEIP = extintPolys.begin(); itEIP != extintPolys.end(); ++itEIP) { // shell number
			int shellNR = itEIP->first;
			map<string,map<int,vector<Point_3>>> bldng = itEIP->second;
			map<string,map<int,vector<Point_3>>>::iterator itEP;
			if (extint == "interior"){
				output << "<bldg:consistsOfBuildingPart>\n";
				output << "<bldg:BuildingPart gml:id=\"" << bagID << "storey_"<< shellNR << "\">\n";
			} else {
				output << "<bldg:consistsOfBuildingPart>\n";
				output << "<bldg:BuildingPart gml:id=\"" << bagID << "_exterior\">\n";
			}

			for (itEP = bldng.begin(); itEP != bldng.end(); ++itEP) { // string polytype
				string polyType = itEP->first;
				map<int,vector<Point_3>> polygons = itEP->second;
				map<int,vector<Point_3>>::iterator itP;
				output	<<	"\t\t\t"		<<	"<bldg:boundedBy>\n";
				output	<<	"\t\t\t\t"		<<	"<bldg:" << polyType << ">\n";
				output	<<	"\t\t\t\t\t"	<<	"<bldg:lod2MultiSurface>\n";
				output	<<	"\t\t\t\t\t\t"	<<	"<gml:MultiSurface srsName=\"EPSG:28992\" srsDimension=\"3\">\n";

				// Write all coordinates for all faces of the building
				for (itP = polygons.begin(); itP != polygons.end(); ++itP) { //Poly number / vector of points
					stringstream iss;
					iss << itP->first;
					vector<Point_3> pointVector = itP->second;
					if (pointVector.size()<3) {
						cout << "Degenerate face \n ";
						continue;
					}
					if (extint == "interior"){
						//reverse(pointVector.begin(),pointVector.end());
					}
					Point_3 pointA = pointVector[0];
					Point_3 pointB = pointVector[1];
					Point_3 pointC = pointVector[2];

					Triangle_3 testTri(pointA,pointB,pointC);
					Line_3 lineA(pointA,pointB);
					Line_3 lineB(pointB,pointC);
					double tol = 1e-10;

					if (testTri.is_degenerate()) continue;

					//if (CGAL::compare (squared_distance(lineA, pointC), tol) == CGAL::SMALLER ||
					//	CGAL::compare (squared_distance(lineB, pointA), tol) == CGAL::SMALLER) {
					//	continue;
					//}
					string polyNR = iss.str();
					counter[polyType]++;
					int plNR = counter[polyType];
					string s;
					stringstream out;
					out << plNR;
					s = out.str();
					out.str(string());
					out.clear();
					string t;
					out << polyNR;
					t = out.str();
					string gmlID = bagID + "_" + polyType + "_" + t + "_" + s;
					gmlIDs[polyType].push_back(gmlID);

					output <<	"\t\t\t\t\t\t\t"		<<	"<gml:surfaceMember>\n";
					output <<	"\t\t\t\t\t\t\t\t"		<<	"<gml:Polygon xmlns:gml=\"http://www.opengis.net/gml\" gml:id=\"" << gmlID << "\">\n";
					output <<	"\t\t\t\t\t\t\t\t\t"		<<	"<gml:exterior>\n";
					output <<	"\t\t\t\t\t\t\t\t\t\t"		<<	"<gml:LinearRing>\n";
					output <<	"\t\t\t\t\t\t\t\t\t\t\t"		<<	"<gml:posList>";
					int i = 0;
					double x0,y0,z0;
					vector<Point_3>::iterator itVP;
					for (itVP = pointVector.begin(); itVP != pointVector.end(); ++itVP) {
						double x = CGAL::to_double(itVP->x()), y = CGAL::to_double(itVP->y()), z = CGAL::to_double(itVP->z());
						output.precision(30);
						output << x << " " << y << " " << z << " ";
						if (i == 0) {
							x0=x, y0=y, z0=z;
							i = 9;
						}
					}
					output << x0 << " " << y0 << " " << z0 << "</gml:posList>\n";
					output <<	"\t\t\t\t\t\t\t\t\t\t"	<<	"</gml:LinearRing>\n";
					output <<	"\t\t\t\t\t\t\t\t\t"		<<	"</gml:exterior>\n";
					output <<	"\t\t\t\t\t\t\t\t"		<<	"</gml:Polygon>\n";
					output <<	"\t\t\t\t\t\t\t"			<<	"</gml:surfaceMember>\n";
				}
				output	<<	"\t\t\t\t\t\t"	<<	"</gml:MultiSurface>\n";
				output	<<	"\t\t\t\t\t"		<<	"</bldg:lod2MultiSurface>\n";
				output	<<	"\t\t\t\t"		<<	"</bldg:" << polyType << ">\n";
				output	<<	"\t\t\t"		<<	"</bldg:boundedBy>\n";
			}
			output << "</bldg:BuildingPart>\n";
			output << "</bldg:consistsOfBuildingPart>\n";

		}
	}
}

void Building::writeAddress(ofstream& output){
	output	<<	stringIndents(0)		<<	"<bldg:address>\n";
	output	<<	stringIndents(1)		<<	"<Address>\n";
	output	<<	stringIndents(1)		<<	"<xalAddress>\n";
	output	<<	stringIndents(1)		<<	"<xAL:AddressDetails>\n";
	output	<<	stringIndents(1)		<<	"<xAL:Country>\n"; 
	output	<<	stringIndents(1)		<<	"<xAL:CountryName>The Netherlands</xAL:CountryName>\n"; 
	output	<<	stringIndents(0)	<<	"<xAL:Locality Type=\"Town\">\n"; 
	output	<<	stringIndents(1)		<<	"<xAL:LocalityName>Rotterdam</xAL:LocalityName>\n"; 
	output	<<	stringIndents(0)		<<	"<xAL:Thoroughfare Type=\"Street\">\n"; 
	output	<<	stringIndents(1)		<<	"<xAL:ThoroughfareNumber>" << houseNR << "</xAL:ThoroughfareNumber>\n"; 
	output	<<	stringIndents(0)		<<	"<xAL:ThoroughfareName>" << streetName << "</xAL:ThoroughfareName>\n";
	output	<<	stringIndents(-1)		<<	"</xAL:Thoroughfare>\n"; 
	output	<<	stringIndents(0)		<<	"<xAL:PostalCode>\n"; 
	output	<<	stringIndents(1)		<<	"<xAL:PostalCodeNumber>" << postalCode << "</xAL:PostalCodeNumber>\n"; 
	output	<<	stringIndents(-1)		<<	"</xAL:PostalCode>\n"; 
	output	<<	stringIndents(-1)		<<	"</xAL:Locality>\n"; 
	output	<<	stringIndents(-1)		<<	"</xAL:Country>\n";
	output	<<	stringIndents(-1)		<<	"</xAL:AddressDetails>\n"; 
	output	<<	stringIndents(-1)		<<	"</xalAddress>\n";
	output	<<	stringIndents(-1)		<<	"</Address>\n";
	output	<<	stringIndents(-1)		<<	"</bldg:address>\n";
}

void Building::writeAppearance(ofstream& output){
	map<string,vector<string>>::iterator it;
	string transp, color;
	for (it = gmlIDs.begin(); it != gmlIDs.end(); ++it) {
		string polyType = it->first;
		if (polyType == "RoofSurface") {
			color = "1 0 0";
			transp = "0.5";
		} else if (polyType == "WallSurface") {
			color = "0.9 0.9 0.4";
			transp = "0.5";
		} else if (polyType == "GroundSurface") {
			color = "0 1 0";
			transp = "0.5";
		} else if (polyType == "InteriorWallSurface") {
			color = "0.7 0.7 0.7";
			transp = "0";
		} else if (polyType == "CeilingSurface") {
			color = "0.8 0.2 0.2";
			transp = "0";
		} else if (polyType == "FloorSurface") {
			color = "0 0 1";
			transp = "0";
		}
		output << stringIndents(0)						<<	"<app:appearanceMember>\n";
		output << stringIndents(1)					<<	"<app:Appearance>\n";
		output << stringIndents(1)				<<	"<app:surfaceDataMember>\n";
		output << stringIndents(1)				<<	"<app:X3DMaterial>\n";
		output << stringIndents(1)				<<	"<app:transparency>" << transp << "</app:transparency>\n";
		output << stringIndents(0)				<<	"<app:diffuseColor>" << color << "</app:diffuseColor>\n";

		for (int v = 0; v != gmlIDs[it->first].size(); ++v) {
			output << stringIndents(0)			<<	"<app:target>BAG" << gmlIDs[it->first][v] << "</app:target>\n";
		}
		output << stringIndents(-1)				<<	"</app:X3DMaterial>\n";
		output << stringIndents(-1)					<<	"</app:surfaceDataMember>\n";
		output << stringIndents(-1)					<<	"</app:Appearance>\n";
		output << stringIndents(-1)						<<	"</app:appearanceMember>\n";
	}

}

void Building::writeBuildingFooter(ofstream& output){
	/*Writes the last two lines for the building*/

	output	<<	stringIndents(-1)			<<	"</bldg:Building>\n";
	output	<<	stringIndents(-1)			<<	"</cityObjectMember>\n";
	output.flush();
}

bool Building::writeToCityGML(ofstream& output, bool oldGeometry, bool writeExterior, bool alwaysWrite) {
	if ((finalPolygons["interior"].size() >= (size_t) storeys && area > 0) || alwaysWrite) {
		/*Write the building to the CityGML file*/
		writeBuildingHeader(output);
		if (oldGeometry) writeGeometry_old(output);
		else writeGeometry(output);
		writeAddress(output);
		writeAppearance(output);
		writeBuildingFooter(output);
		return true;
	}
	else {
		cout << "\t\t !! Not writing to CityGML, not succeeded!!\n"; 
		return false;
	}
}

string Building::stringIndents(int i) {
	string s;
	tabs+=i;
	for (int j = 0; j < tabs; ++ j) {
		s.push_back('\t');
	}
	
	return s;
}


/*########################################################
##########################################################
The following function can write a building to the .OFF
file format. For debugging purposes.
##########################################################
########################################################*/

void Building::writeOff(string buildingID) {
	string validity;
	string closed;

	if (polyBuilding.is_closed()) {
		closed = "_closed";
	}
	else closed = "_not_closed";

	if (polyBuilding.is_valid()) {
		validity = "_valid";
	}
	else validity = "_not_valid";

	boost::filesystem::path dir("D:\\Data\\Temp\\" + buildingID + closed + validity + "\\" + streetName + "_" + houseNR );
	boost::filesystem::create_directories(dir);
	ofstream out;
	out.open("D:\\Data\\Temp\\" + buildingID + closed + validity + "\\" + streetName + "_" + houseNR +  "\\exterior.off");
	out.precision(15);
	out << polyBuilding;
	out.close();
}

/*########################################################
########################################################*/

