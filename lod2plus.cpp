#include "lod2plus.h"
#include "globals.h"
#include "commonfunctions.h"
#include "Building.h"
#include "Citymodel.h"

string askNewFile (string R3D="", bool in = true) {
	/* Ask user for a filename, input or output */
	string file;
	if (in) {
		cout << "Enter input filename without .xml: "; 
		getline(cin,file);
		if (file.size()>=2) {
			return "D:\\Dropbox\\Data\\" + file + ".xml";
		} else return "D:\\Dropbox\\Data\\newHealed.xml";
	} else {
		cout << "Enter output filename without .xml: "; 
		getline(cin,file);
		if (file.size()>=2) {
			return "D:\\Dropbox\\Data\\" + file + ".xml";
		} else {
			string outFile = R3D.substr(0,R3D.size()-4) + "_output.xml";
			return outFile;
		}
	}
}

map<string,map<string,double>> readThickness (string fileName) {
	string line;
	map<string,map<string,double>> thicknesses;
	ifstream thicknessInput(fileName);
	if (thicknessInput.is_open()) {
		while (thicknessInput.good()) {
			getline(thicknessInput,line);
			vector<string> splits;
			boost::split(splits, line, boost::is_any_of(";"));
			string type = splits[0];
			double wall = strToDbl(splits[1]);
			double inner = strToDbl(splits[2]);
			thicknesses[type]["inner"]=inner;
			thicknesses[type]["outer"]=wall;			
		}
	}
	thicknessInput.close();
	return thicknesses;
}

void determineThickness (double &wall, double &innerWall, string builtYear, int storeys, string buildingType, map<string,map<string,double>> thicknesses) {
	string cat;
	int yearOfConstruction = strToInt(builtYear);
	if (buildingType.find("niet gestapeld")!=string::npos){
		if (yearOfConstruction<1970){
			if (storeys <= 2) cat = "A11";
			else  cat = "A12";
		} else if (yearOfConstruction>=1970 && yearOfConstruction<=1985){
			if (storeys <= 2) cat = "A21";
			else if (storeys == 3) cat = "A22";
			else cat = "A23";
		} else {
			if (storeys <= 2) cat = "A31";
			else if (storeys == 3) cat = "A32";
			else cat = "A33";
		}
	} else if (buildingType.find("gestapeld")!=string::npos){
		if (yearOfConstruction<1970){
			if (storeys <= 5) cat = "B11";
			else if (storeys <= 10) cat = "B12";
			else cat = "B13";
		} else if (yearOfConstruction>=1970 && yearOfConstruction<=1985){
			if (storeys <= 5) cat = "B21";
			else if (storeys <= 10) cat = "B22";
			else cat = "B23";
		} else {
			if (storeys <= 5) cat = "B31";
			else if (storeys <= 10) cat = "B32";
			else cat = "B33";
		}
	} else {
		if (yearOfConstruction<1970){
			if (storeys == 1) cat = "C11";
			else cat = "C12";
		} else if (yearOfConstruction>=1970 && yearOfConstruction<=1985){
			if (storeys == 1) cat = "C21";
			else cat = "C22";
		} else {
			if (storeys == 1) cat = "C31";
			else cat = "C32";
		}
	}
	wall = thicknesses[cat]["outer"];
	innerWall = thicknesses[cat]["inner"];
}

void updateConfigFile(string R3D, string BAG, string out, string& statsFile, string sharedWalls, string thickness, string latestBagID){
	ofstream config("D:\\Dropbox\\Data\\config.cfg");
	config << R3D << endl << BAG << endl << out << endl << statsFile << endl << sharedWalls << endl << thickness << endl << latestBagID << endl;
	config.close();
}

void readConfigFile(string& R3D, string& BAG, string& out, string& statsFile, string& sharedWalls, string& thickness, string& latestBagID){
	ifstream config("D:\\Dropbox\\Data\\config.cfg");
	getline(config, R3D);
	getline(config, BAG);
	getline(config, out);
	getline(config, statsFile);
	getline(config, sharedWalls);
	getline(config, thickness);
	getline(config, latestBagID);
	config.close();
}

int main (int argc, char* argv[] ) {
	VLDDisable(); // Disable memory leak reporting
	bool cont = false, start;
	string R3DFile, bagFile, outFile, statsFile, sharedWalls, thickness, latestBagID;
	string directory = "D:\\Dropbox\\Data\\Individual_buildings\\";
	ofstream stats, CityOut;

	if (argc==6) {
		R3DFile = argv[1]; bagFile = argv[2]; outFile = argv[3]; sharedWalls = argv[4]; thickness = argv[5];
	} else {
		cout << "Continue previous process? [y/n]: ";
		string contin;
		getline(cin,contin);
		if (contin == "y") {
			readConfigFile(R3DFile,bagFile,outFile,statsFile,sharedWalls,thickness,latestBagID);
			CityOut.open(outFile, ios_base::app);
			stats.open(statsFile, ios_base::app);

			cont = true;
			start = false;
		} else{	
			R3DFile = askNewFile(),
				bagFile = "D:\\Dropbox\\Data\\bag.csv",
				outFile = askNewFile(R3DFile, false),
				statsFile = outFile.substr(0,outFile.size()-4) + "_stats.csv",
				sharedWalls = "D:\\Dropbox\\Data\\sharedWalls.csv",
				thickness = "D:\\Dropbox\\Data\\thickness.csv";
		}
	}
	
	if (!cont) {
		try {
			boost::filesystem::path dir(directory);
			boost::filesystem::create_directories(dir);
			removeFilesInDir(directory,".xml");
		} catch(...){}
	}

	map<string,vector<Line_2>> sharedWallsMap = readSharedWalls(sharedWalls);
	map<string,map<string,double>> thicknessMap = readThickness(thickness);

	// CityModel read from CityGML + BAG info file
	CityModel *R3D = new CityModel(R3DFile,bagFile);
	
	if (!cont){
		CityOut.open(outFile);
		R3D->writeCityHeader(CityOut); // Write the first lines of the CityGML file
		stats.open(statsFile);
		stats << "BAG ID;BAG Use Area;R3D Area;Net Height Area;Built year;Building type;Storeys\n"; 	// file to save the stats (use area, built year, type, net height area etc)
	}

	// Storing the time needed to convert each building to lod2plus and calculate net internal area
	vector<double> buildingTimes;
	CGAL::Timer T;
	try {
		while(R3D->buildingsLeft()) {
			T.reset(); T.start(); //reset and start timer

			polyTypeMap building = R3D->getNextBuilding(); //Store polygons of new building

			string bagID = R3D->getCurBagID(),			//Get all the info of the building
				buildingID = R3D -> getCurBuildingID(),
				houseNR = R3D->getCurHouseNumber(), 
				streetName = R3D->getCurStreetName(), 
				postalCode = R3D->getCurPostalCode(),
				bagUseArea = R3D->getCurBagUseArea(),
				builtYear = R3D->getCurBuiltYear(),
				buildingType = R3D->getCurBuildingType(),
				buildingNr = R3D->getCurBuildingNr();

			if (cont && !start) {
				if (latestBagID == bagID) {
					start = true;
					continue;
				} else continue;
			}
			if (bagID == "0599100000661081" || // Skip these, the program goes into infinite loop otherwise
				bagID == "0599100000652777" || 
				bagID == "0599100000677533" ||
				bagID == "0599100000329367" ||
				bagID == "0599100000334617" ||
				bagID == "0599100000206904" ||
				bagID == "0599100000229841" ||
				bagID == "0599100000115118" ||
				bagID == "0599100000646766" ||
				bagID == "0599100000091030" ||
				bagID == "0599100000131408") 
				continue;

			updateConfigFile(R3DFile,bagFile,outFile,statsFile,sharedWalls,thickness,bagID);

			int storeys = R3D->getCurStoreys(),
				lowestFloor = R3D->getLowestFloor(),
				highestFloor = R3D->getHighestFloor();

			double wallThickness, innerWallThickness;
			determineThickness(wallThickness,innerWallThickness,builtYear,storeys,buildingType,thicknessMap);


			cout << "Processing building " << buildingNr << ": " << bagID << " (" << streetName << " " << houseNR << ")" << endl;

			Building *B = new Building(building, bagID, storeys); //create instance of building with polygons, bag id and nr of storeys

			if(sharedWallsMap[bagID].size()>0) B->setSharedWalls(sharedWallsMap[bagID]);

			B->setBuildingInfo(houseNR,streetName,"Rotterdam",postalCode,bagUseArea,lowestFloor,highestFloor);
			B->createBuilding();
			
			//B->writeOff(buildingID); //write current building in off format

			if (B->curValid()) {
				try {
					B->setThickness(wallThickness,innerWallThickness);
					cout << "\t ## Number of storeys: " << storeys << endl;
					cout << "\t ## Wall Thickness: " << wallThickness << " cm | Shared Wall Thickness: " << innerWallThickness << " cm" << endl;
					cout << "\t -> Creating LoD2+";
					try {
						B->createLoD2Plus();
					} catch(...) {
						cerr << "\t !!!! Could not create LoD2+\n";
						continue;
					}
					cout << endl;
					cout << "\t -> Calculating Floor Area\n";
					B->calcArea();
					string calcArea = B->getArea();
					double diff = static_cast<int>(((strToDbl(calcArea)-strToDbl(bagUseArea))/strToDbl(calcArea)*100)+0.5);
					cout << "\t\t BAG:" << bagUseArea << "m^2  |  R3D:" << calcArea << "m^2 | Diff: " << diff << "%\n";
					cout << "\t -> Writing building to CityGML\n";
					bool success = B->writeToCityGML(CityOut, false, true);

					/* Write individual building to citygml for viewing purposes*/
					string buildingFile = directory + bagID + "_" + calcArea + "_" + bagUseArea + ".xml";
					ofstream buildingOut(buildingFile);
					R3D->writeCityHeader(buildingOut);
					B->writeToCityGML(buildingOut,false,true,true);
					R3D->closeCityGML(buildingOut);
					buildingOut.close();
					/*------------------------------------------------------*/

					string netHeightArea = B->getNetHeightArea();
					if (success) stats << bagID << ";" << bagUseArea << ";" << calcArea << ";" << netHeightArea << ";" << builtYear << ";" << buildingType << ";" << storeys << endl;

				} catch (...) {
					cerr << "\n\n\n###########\nCould not split building.\nMaybe non-planar faces.\nSkipping building\n#########\n\n\n";
				}
			}
			else cout << "\t !!! Building probably no valid geometry\n"; 

			B->clearAll();
			delete B;
			T.stop();
			if (T.time() > 1.5) {
				buildingTimes.push_back(T.time());
				double avgTime = accumulate(buildingTimes.begin(),buildingTimes.end(),0) / buildingTimes.size();
				cout << "\t -> Finished in " << T.time() << " sec (average: " << avgTime << " sec)" << endl;
			}
			cout << endl;

		}
	} catch(...) {
		cerr << "Something went wrong, cause undetermined\n";
	}


	R3D->closeCityGML(CityOut);
	delete R3D;

	stats.close();
	cout << "\n\nFinished LoD2+ generation, press enter to exit\n";
	cin.clear();
	cin.get();
	return 0;
}


