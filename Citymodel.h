#include "globals.h"
#include "polybuilder.h"

/* This header file provides basic functions to read and write the CityGML 
file and to pop buildings one by one */

class CityModel {
private:
	string buildingID, bagID, buildingYear, streetName, houseNR, postalCode;
	ifstream R3D_input, bagInput;
	vector<string> cityHeader;
	bagMap bagData;
	map<string,int> bagArea;
	int nrOfBuildings, curBuildingNr;


public:
	//Forward declarations
	//Parsers
	polyTypeMap getNextBuilding();
	bool buildingsLeft();
	void readBagData();
	string isFeature (string line);

	//Getters
	string getCurBagID ();
	string getCurHouseNumber ();
	string getCurPostalCode ();
	string getCurStreetName ();
	string getCurBuildingID ();
	string getCurBuiltYear ();
	string getCurBuildingNr ();
	int getCurStoreys ();
	int CityModel::getHighestFloor ();
	int CityModel::getLowestFloor ();
	string getCurBagUseArea();
	string getCurBuildingType();

	//Readers
	void readHeader();

	//Writers
	void writeCityHeader(ofstream& output);
	void closeCityGML(ofstream& output);

	CityModel(string R3Dfile, string bagFile) {
		try {
			curBuildingNr = 1;
			ifstream countBuildings;
			countBuildings.open(R3Dfile);
			string line;
			if (countBuildings.is_open()) {
				while (countBuildings.good()) {
					getline(countBuildings,line);
					if (line.find("<bldg:Building")!=string::npos) {
						++nrOfBuildings;
					}
				}
			}
			countBuildings.close();
			R3D_input.open(R3Dfile);
			readHeader();
		} catch (...) {
			cout << "CityGML file does not exist. \n";
		}
		try {
			bagInput.open(bagFile);
			readBagData();
		} catch (...) {
			cout << "CityGML file does not exist. \n";
		}
	}
};


