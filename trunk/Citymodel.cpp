#include "Citymodel.h"
#include "globals.h"

void CityModel::closeCityGML(ofstream& output) {
	output << "</CityModel>";
	output.close();

}

void CityModel::readHeader() {
	string line;
	if (R3D_input.is_open()) {
		while (R3D_input.good()) {
			getline(R3D_input,line);
			cityHeader.push_back(line);
			if (line.find("</gml:boundedBy>")!=string::npos) {
				break;
			}
		}
	}
}

int CityModel::getCurStoreys () {
	int storeyNr=0;
	map<string,string> bagBuilding = bagData[buildingID];
	if ( bagBuilding.find("Aantal_bouwlagen_Pand") != bagBuilding.end() ) {
		istringstream (bagData[buildingID]["Aantal_bouwlagen_Pand"]  ) >> storeyNr;
		return storeyNr;
	}
	else {
		return 1;
	}
}

int CityModel::getHighestFloor () {
	int storeyNr;
	map<string,string> bagBuilding = bagData[buildingID];
	if ( bagBuilding.find("HOOGSTE_BOUWLAAG") != bagBuilding.end() ) {
		istringstream (bagData[buildingID]["HOOGSTE_BOUWLAAG"]  ) >> storeyNr;
	} else storeyNr = 2;
	return storeyNr;
}

int CityModel::getLowestFloor () {
	int storeyNr;
	map<string,string> bagBuilding = bagData[buildingID];
	if ( bagBuilding.find("LAAGSTE_BOUWLAAG") != bagBuilding.end() ) {
		istringstream (bagData[buildingID]["LAAGSTE_BOUWLAAG"]  ) >> storeyNr;
	} else storeyNr = 0;
	return storeyNr;
}

string CityModel::getCurBuildingID() {
	return buildingID;
}

string CityModel::getCurStreetName () {
	map<string,string> bagBuilding = bagData[buildingID];
	if ( bagBuilding.find("Openbare ruimte") != bagBuilding.end() ) {
		return bagData[buildingID]["Openbare ruimte"];
	}
	else {
		return "";
	}
}

string CityModel::getCurHouseNumber () {
	map<string,string> bagBuilding = bagData[buildingID];
	if (bagBuilding.find("Hsnr") != bagBuilding.end() ) {
		return bagData[buildingID]["Hsnr"];
	}
	else {
		return "";
	}
}

string CityModel::getCurPostalCode () {
	map<string,string> bagBuilding = bagData[buildingID];
	if ( bagBuilding.find("Postcd") != bagBuilding.end() ) {
		return bagData[buildingID]["Postcd"];
	}
	else {
		return "";
	}
}

string CityModel::getCurBagID () {
	return bagID;
}

string CityModel::getCurBagUseArea() {
	int area = bagArea[bagID];
	stringstream ss;
	string bagUseArea;
	ss << area;
	return ss.str();
}

string CityModel::getCurBuiltYear () {
	string builtYear;
	map<string,string> bagBuilding = bagData[buildingID];
	if ( bagBuilding.find("BOUWJAAR") != bagBuilding.end() ) {
		builtYear = bagData[buildingID]["BOUWJAAR"];
	}
	return builtYear;
}

string CityModel::getCurBuildingType () {
	string builtYear;
	map<string,string> bagBuilding = bagData[buildingID];
	if ( bagBuilding.find("Bestemming") != bagBuilding.end() ) {
		builtYear = bagData[buildingID]["Bestemming"];
	}
	return builtYear;
}

string CityModel::getCurBuildingNr () {
	stringstream ss;
	ss << curBuildingNr << "/" << nrOfBuildings;
	return ss.str();
}

string CityModel::isFeature (string line) {
	string building="<gml:name>", 
		roof="<bldg:RoofSurface", 
		ground="<bldg:GroundSurface", 
		wall="<bldg:WallSurface", 
		position= "<gml:posList", 
		lowerCorner = "<gml:lowerCorner>",
		buildingEnd = "</cityObjectMember>",
		type="none";

	if(line.find(building)!=string::npos) 
		type = "building";
	else if(line.find(ground)!=string::npos) 
		type = "GroundSurface";
	else if(line.find(roof)!=string::npos) 
		type = "RoofSurface";
	else if(line.find(wall)!=string::npos) 
		type = "WallSurface";
	else if(line.find(position)!=string::npos) 
		type = "polygon";
	else if(line.find(lowerCorner)!=string::npos) 
		type = "lowerCorner";
	else if(line.find(buildingEnd)!=string::npos) 
		type = "buildingEnd";
	return type;
}

void CityModel::readBagData() {
	string line;
	getline ( bagInput, line);	
	vector<string> attributes;
	vector<string> tempData;
	map<string,string> tempBagData;

	boost::split(attributes, line, boost::is_any_of(";"));	

	if (bagInput.is_open())
	{
		while ( bagInput.good() )
		{
			getline(bagInput,line);
			boost::split(tempData, line, boost::is_any_of(";"));	
			for (int v = 0; v!=tempData.size()-1;++v) {
				tempBagData[attributes[v]]=tempData[v];
			}
			string bagIDPand = tempBagData["BAG_id_Pand"];
			string tempArea = tempBagData["Oppervlakte"];
			stringstream ss(tempArea);
			int area;
			ss >> area;
			if (bagArea.find(bagIDPand) == bagArea.end()) {
				bagArea[bagIDPand] = area;
			} else {
				bagArea[bagIDPand] += area;
			}

			bagData[tempData[tempData.size()-1]]= tempBagData;
		}
	}
	bagInput.close();
}

bool CityModel::buildingsLeft() {
	string line;
	if (R3D_input.is_open())
	{
		getline(R3D_input,line);
		if (line.find("</CityModel>")==string::npos) return true;
		else {
			R3D_input.close();
			return false;
		}
	}
	else return false;
}

polyTypeMap CityModel::getNextBuilding(){
	string line, polType;
	polyTypeMap polygons;
	bool goodBuilding = true;
	int polNr=0; 
	if (R3D_input.is_open()) {
		while (R3D_input.good()) {
			getline(R3D_input,line);
			string type = isFeature(line);
			if (type != "none") {
				if (type == "building") {
					polyTypeMap polygons;
					goodBuilding = true;
					buildingID = extractInfo(line,">","<"); //store building id
					bagID = bagData[buildingID]["BAG_id_Pand"];
					if (bagID == "") {
						goodBuilding = false;
						continue;
					}
					polNr = 0;
				}
				else if (type == "GroundSurface" || type == "RoofSurface" || type == "WallSurface"){
					if (polType != type) {
						polNr = 0;
						polType = type;
					}
				}
				else if (type== "polygon") {
					string poly = extractInfo(line,">","<"); //coordinates of polygon in string
					vector<Point_3> polyVector = posToVector(poly, true); //string to vector of 3d points
					polyVector.erase(polyVector.end()-1);
					polygons[polType][polNr] = polyVector; //store in map
					polNr+=1;
				}
				if (type == "buildingEnd" && goodBuilding) {
					++curBuildingNr;
					return polygons;
				}
				else if (type == "buildingEnd" && !goodBuilding) {
					++curBuildingNr;
					polygons.clear();
				}
			}
		}
	}
	return polygons;
}

void CityModel::writeCityHeader(ofstream& output) {
	for (int i = 0; i != cityHeader.size(); ++i) {
		output << cityHeader[i] << endl;
	}
}
