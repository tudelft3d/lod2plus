#include "globals.h"
#include "polybuilder.h"

void simplifyPolyhedron (Polyhedron& poly);

class Building
	/* The class building contains the building polyhedron and other building information
	and the functions to be able to make the interior storey volumes of the building */
{
private:
	map<string,map<int,vector<Point_3>>> buildingPolygons;
	Polyhedron polyBuilding, storey;
	Nef_polyhedron Nef_building; //, Nef_storey;
	vector<Nef_polyhedron> nefStoreyVector;
	map<string,map<int,map<string,map<int,vector<Point_3>>>>>  finalPolygons;
	map<string,vector<string>> gmlIDs;
	vector<Line_2> sharedWalls;
	Nef_polyhedron nefBox;

	string type, bagID, buildingID, streetName, postalCode, place, houseNR, bagUseArea;
	int storeys, lowFloor, highFloor, builtYear, tempNR, tabs;
	double area, volume, wallThickness, ceilingThickness, floorThickness, roofThickness, sharedWallThickness, netHeightArea;
	bool valid;


public:
	/*Forward declarations*/
	

	//Geometric functions
	void createLoD2Plus();
	Polyhedron createSplitPolyhedron (vector<Point_2> points, double minHeight, double maxHeight);
	vector<Point_2> getConvexHull2D();
	double getHeight();
	void setThickness(double wallThickness,double sharedWallThickness,double floorCeilingThickness=0.2);
	void removeExtrudedFacets(Nef_polyhedron& Nef_storey, int floor);
	Nef_polyhedron makeRobot (string polyType, vector<Point_3> points);
	double getMinHeight();
	double getMaxHeight();
	string determinePolyType (vector<Point_3> points, bool outer, int floor);
	void exteriorToFinalPolygons ();
	void determineAreaNetHeight(int i);
	vector<double> getMarkedHeights();
	vector<double> Building::setSplitHeights ();
	void createUnitBox ();


	//CityGML writers
	bool writeToCityGML(ofstream& output, bool old = false, bool writeExterior=false, bool alwaysWrite=false);
	void writeBuildingFooter(ofstream& output);
	void writeAppearance(ofstream& output);
	void writeAddress(ofstream& output);
	void writeGeometry(ofstream& output);
	void writeGeometry_old(ofstream& output);
	void writeBuildingHeader(ofstream& output);
	string stringIndents(int = 0);

	void addPolyType (int i, bool finalStorey);
	void calcArea();
	void calcGrossVolume();

	//OFF writer
	void writeOff(string buildingID);

	//Building constructor
	Building (polyTypeMap polygons, string bag, int nrOfStoreys, string buildingType="", int buildingYear=1990)
		: buildingPolygons(polygons),bagID(bag), builtYear(buildingYear), type(buildingType),storeys(nrOfStoreys){
			tempNR = 0;
	}

	void createBuilding() {
		try {
			polyBuilding = buildSurface(buildingPolygons,0.0);
			stringstream iss;
			iss.precision(20);
			iss << polyBuilding;

			CGAL::OFF_to_nef_3(iss, Nef_building);
			/*Polyhedron pb;
			Nef_building.convert_to_polyhedron(pb);
			ofstream offTemp ("D:\\Data\\building.off");
			offTemp << setprecision(30) << pb;
			offTemp.close();
			cin.clear();cin.get();*/
			Nef_building = Nef_building.regularization();

			if (lowFloor < 0) {
				cout << "\t ## Extending building downwards for basement\n";
				double maxHeight = getMaxHeight();
				double extrudeDown = -(static_cast<double>(storeys)/(static_cast<double>(highFloor)+1)*maxHeight-maxHeight);
				polyBuilding = buildSurface(buildingPolygons,extrudeDown);
				stringstream iss;
				iss.precision(20);
				iss << polyBuilding;
				CGAL::OFF_to_nef_3(iss, Nef_building);
				Nef_building = Nef_building.regularization();
				try {
					Nef_building.convert_to_polyhedron(polyBuilding);
					exteriorToFinalPolygons();
				} catch (...) {
					finalPolygons["exterior"][0]=buildingPolygons;
				}
			}
			else finalPolygons["exterior"][0]=buildingPolygons;
		} catch(...) {
			cerr << "Could not create polyhedron from building polygons\n";
		}
	}

	~Building(){}

	void clearAll() {
		finalPolygons.clear();
		polyBuilding.clear();
		storey.clear();
	}

	void setBuildingInfo(string hsNR, string street, string town, string pstCd, string bagArea, int lowestFloor, int highestFloor) 
	{
		/*Attaches the address to the building*/
		houseNR = hsNR;
		streetName = street;
		place = town;
		postalCode = pstCd;
		bagUseArea = bagArea;
		lowFloor = lowestFloor;
		highFloor = highestFloor;
	}

	void setSharedWalls(vector<Line_2> SW) {
		sharedWalls = SW;
	}

	bool curValid () {
		/* Returns to the user whether the building currently in stack
		is valid or not*/
		if (polyBuilding.is_closed() && polyBuilding.is_valid(false,3)) return true;
		else return false;
	}

	string getArea() {
		int intArea = static_cast<int>(area);
		stringstream ss;
		ss << intArea;
		return ss.str();
	}

	string getVolume() {
		int intVol = static_cast<int>(volume);
		stringstream ss;
		ss << intVol;
		return ss.str();
	}

	string getNetHeightArea(){
		stringstream ss;
		ss << static_cast<int>(netHeightArea);
		return ss.str();
	}
};
