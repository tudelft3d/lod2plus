#include "globals.h"
#include "commonfunctions.h"

string intToString(int number)
{
	stringstream ss;//create a stringstream
	ss << number;//add number to the stream
	return ss.str();//return a string with the contents of the stream
}


vector<int> makeUniqueVector (vector<int> v) {
	set<int> s;
	vector<int> newV;
	for (vector<int>::iterator vit = v.begin(); vit != v.end(); ++vit) {
		if (s.find(*vit)==s.end()) {
			s.insert(*vit);
			newV.push_back(*vit);
		}
	}
	return newV;
}

string stringToRatio (string coord, int dec = 3) {
	vector<string> digits;
	boost::split(digits, coord, boost::is_any_of("."));
	string integers = digits[0];
	string decimals = digits[1];
	string finalDecimals = decimals.substr(0,dec);
	integers += finalDecimals + "/1";
	for (int i = 0; i != finalDecimals.size(); ++i) {
		integers += "0";
	}
	return integers;
}

string extractInfo (string toExtract, string first, string second){
	/* Determines the info that is between 'first' and 'second' string.
	For example in xml to determine the data between quotes in line s
	call extractInfo (s, "\"", "\"") */
	size_t pos = toExtract.find(first)+1, pos2 = toExtract.find(second,pos+1);
	int length = pos2-pos;
	return toExtract.substr(pos,length);
}

vector<Point_3> posToVector (string posLine, bool epec = false) {
	vector<string> coords;
	boost::split(coords, posLine, boost::is_any_of(" "));	
	vector<Point_3> poly;

	istringstream iss(posLine);
	vector<double> tempDoubles;
	double coord;
	for (int i = 0; i != coords.size(); ++i) {
		iss >> coord;
		tempDoubles.push_back(coord);
		if (tempDoubles.size() == 3) {
			const double x = tempDoubles[0], y = tempDoubles[1], z = tempDoubles[2];
			poly.push_back(Point_3(x,y,z));
			tempDoubles.clear();
		}
	}
	return poly;
}

map<string,vector<Line_2>> readSharedWalls(string fileName){
	ifstream readFile(fileName);
	map<string,vector<Line_2>> sharedWalls;
	string line;
	if (readFile.is_open()) {
		while (readFile.good()) {
			getline(readFile,line);
			vector<string> attributes;
			boost::split(attributes, line, boost::is_any_of(";"));	
			string BAG1 = attributes[0],
				BAG2 = attributes[1];
			double x1 = atof (attributes[2].c_str()),
				y1 = atof (attributes[3].c_str()),
				x2 = atof (attributes[4].c_str()),
				y2 = atof (attributes[5].c_str());
			Line_2 lineSegment(Point_2(x1,y1),Point_2(x2,y2));
			sharedWalls[BAG1].push_back(lineSegment);
			sharedWalls[BAG2].push_back(lineSegment);
		}
	}
	readFile.close();
	return sharedWalls;
}

double strToDbl(string s) {
     double d;
     stringstream ss(s); //turn the string into a stream
     ss >> d; //convert
     return d;
}

int strToInt(string s) {
	 int i;
     stringstream ss(s); //turn the string into a stream
     ss >> i; //convert
     return i;
}

string dblToStr(double d, int ndec) {
	int i = 0;
	stringstream ss;
	ss.precision(50);
	bool pointFound = false;
	ss << d;
	string ds = ss.str(), ns;
	string::iterator si = ds.begin();
	while (i<ndec) {
		ns.push_back(*si);
		if (*si == '.' && !pointFound) {
			pointFound = true;
		} else if (pointFound) ++i;
		++si;
		if (si==ds.end()) {
			break;
		}
	}
	return ns;
}

bool isDegen(vector<Point_3> pointVector, double tol) {
	Point_3 pointA = pointVector[0], pointB = pointVector[1], pointC = pointVector[2];
	Line_3 lineA(pointA,pointB), lineB(pointB,pointC);
	Triangle_3 testTri(pointA,pointB,pointC);

	if (testTri.is_degenerate()) return true;
	else if (pointVector.size()<3) return true;
	else if(CGAL::compare (squared_distance(lineA, pointC), tol) == CGAL::SMALLER ||
		CGAL::compare (squared_distance(lineB, pointA), tol) == CGAL::SMALLER) return true;
	else return false;
}

void removeFilesInDir(string directory, string extension) {
	boost::filesystem::path p(directory);
	if(boost::filesystem::exists(p) && boost::filesystem::is_directory(p))
	{
		boost::filesystem::directory_iterator end;
		for(boost::filesystem::directory_iterator it(p); it != end; ++it)
		{
			try
			{
				if(boost::filesystem::is_regular_file(it->status()) && (it->path().extension().compare(extension) == 0))
				{
					boost::filesystem::remove(it->path());
				}
			}
			catch(const std::exception &ex)
			{
				ex;
			}
		}
	}
}

vector<Nef_polyhedron> splitNefs (Nef_polyhedron N) {
	vector<Nef_polyhedron> nefVector;
	//cout << "Number of volumes: " << N.number_of_volumes() << endl;
	if (N.number_of_volumes()>2){
		Nef_polyhedron::Volume_const_iterator vi = ++N.volumes_begin();
		Nef_polyhedron::Shell_entry_const_iterator si;
		CGAL_forall_volumes(vi,N) {	// Split intersection may produce separated volumes, handle all volumes individually
			if(vi->mark()) {
				Nef_polyhedron::Shell_entry_const_iterator si;
				CGAL_forall_shells_of(si, vi) {
					Nef_polyhedron::SFace_const_handle sfch(si);
					Nef_polyhedron temp(N,sfch);
					nefVector.push_back(temp);
				}
			}
		}
	} else nefVector.push_back(N);
	return nefVector;
}

vector<vector<Point_3>> getFacets (Nef_polyhedron N) {
	vector<vector<Point_3>> facets; vector<Point_3> tempPoints;
	Nef_polyhedron::Halffacet_const_iterator nhcit, nhcet;
	for(nhcit = N.halffacets_begin(), nhcet = N.halffacets_end(); nhcit!=nhcet; ++nhcit) { 
		if(nhcit->is_twin()) continue; 
		for(Nef_polyhedron::Halffacet_cycle_const_iterator fc = nhcit->facet_cycles_begin(), 
			end = nhcit->facet_cycles_end(); fc != end; ++fc) { 
			if ( fc.is_shalfedge() ) 
			{ 
				Nef_polyhedron::SHalfedge_const_handle h = fc; 
				Nef_polyhedron::SHalfedge_around_facet_const_circulator hc(h), he(hc);
				CGAL_For_all(hc,he){ // all vertex coordinates in facet cycle 
					Nef_polyhedron::SVertex_const_handle v = hc->source(); 
					Point_3 p = v->source()->point();
					tempPoints.push_back(p);
				} 
			} 
			facets.push_back(tempPoints); tempPoints.clear();
		} 
	} 
	return facets;
}

Nef_polyhedron createConvexHullNef (Nef_polyhedron& N) {
	Polyhedron P, Phull;
	N.convert_to_polyhedron(P);
	Polyhedron::Vertex_iterator vit;
	vector<Point_3> tempPoints;
	for (vit = P.vertices_begin(); vit != P.vertices_end(); ++ vit) {
		tempPoints.push_back(vit->point());
	}
	CGAL::convex_hull_3(tempPoints.begin(),tempPoints.end(),Phull);
	return Nef_polyhedron(Phull);
}

Transformation getRotationMatrix(Vector_3 sourceVec, Vector_3 targetVec) {
	Kernel::FT  spFT = sourceVec*targetVec;										// Scalar product
	double spDouble  = CGAL::to_double(spFT);			

	if (spDouble <= -1.0 || spDouble >= 1.0)	return Transformation(1,0,0,0,1,0,0,0,1);
	double angle = acos(spDouble);												// Calculate angle

	Vector_3 cross = CGAL::cross_product(sourceVec, targetVec);					// Cross product
	double cLen = sqrt(CGAL::to_double(cross.squared_length()));
	if (angle==0 || cLen==0)					return Transformation(1,0,0,0,1,0,0,0,1);

	cross = cross / cLen;														// Normalize rotation vector
	Kernel::FT x = cross.x(); Kernel::FT y = cross.y(); Kernel::FT z = cross.z();	
	Kernel::FT co = (1-spFT);
	Kernel::FT si = sin(angle);

	return Transformation	(1+co*(x*x-1),	-z*si+co*x*y,	y*si+co*x*z,		// Rodrigues formula for the rotation matrix
							 z*si+co*x*y,	1+co*(y*y-1),	-x*si+co*y*z,
							 -y*si+co*x*z,	x*si+co*y*z,	1+co*(z*z-1));
}


void writeOffFile (Nef_polyhedron N, string file) {
	ofstream f(file);
	Polyhedron p;
	N.convert_to_polyhedron(p);
	f << setprecision(30) << p;
	f.close();
}

void fix_degenerate(Polyhedron& polyhe) {
	if (polyhe.is_pure_triangle()) {

		int		maxItr		= 300;
		int		itr			= 0;
		for (;itr<maxItr;itr++) {
			bool degenFound = false;
			for (Polyhedron::Facet_iterator fit = polyhe.facets_begin(); fit != polyhe.facets_end(); ++fit) {	// Iterate over faces

				pointVector facePoints;
				Polyhedron::Facet::Halfedge_around_facet_circulator itHDS = fit->facet_begin();
				do facePoints.push_back(itHDS->vertex()->point());												// Get vertexes of face
				while (++itHDS != fit->facet_begin());

				if ((Triangle_3(facePoints[0],facePoints[1],facePoints[2])).is_degenerate()) {					// Check if degenerate
					degenFound = true;
					std::cerr << "WARNING: degenerate triangle found and will be fixed (fix_degenerate)" << std::endl;
					
					Kernel::FT length1 = 999999999;
					Kernel::FT length2 = 999999999;
					Polyhedron::Halfedge_handle smallestHE1;
					Polyhedron::Halfedge_handle smallestHE2;
					for (int i=0;i<3;i++,itHDS++){										// Get distance of each edge
						Kernel::FT tempLen = CGAL::squared_distance(facePoints[i],facePoints[(i+1)%3]);
						if (tempLen < length2) {										// Store 2 shortest edges
							if (tempLen < length1) {
								length2 = length1;
								length1 = tempLen;
								smallestHE2 = smallestHE1;
								smallestHE1 = itHDS->next();					
							} else {
								length2 = tempLen;
								smallestHE2 = itHDS->next();
					}	}	}

					bool succes = false;
								 succes = do_edgeCollapse(polyhe,smallestHE1);		// Try to collapse the shortest edge
					if (!succes) succes = do_edgeCollapse(polyhe,smallestHE2);		// If failed try to collapse second shortest edge
					if (!succes) std::cerr << "WARNING: Could not fix degenerate Triangle" << std::endl;
					break;
				}
			}
			if (!degenFound)break;
		}
		if (itr==maxItr) std::cerr << "ERROR: More than " << maxItr << " degenerate cases found. Fixing stopped" <<std::endl;
	} else std::cerr << "ERROR: Not pure triangle (fix_degenerate)" << std::endl;
}

bool do_edgeCollapse(Polyhedron& poly, Polyhedron::Halfedge_handle h) {
	if (CGAL::circulator_size(h->vertex_begin()) == 3) {										
		poly.erase_center_vertex(h);															// Probably crashes on border edges
	} else if(CGAL::circulator_size(h->vertex_begin()) > 3			&&							// Should not fail if closed
			  CGAL::circulator_size(h->next()->vertex_begin())>=3	&&
			  CGAL::circulator_size(h->opposite()->prev()->vertex_begin())>=3 ) {
				  poly.join_facet(h->next());													// Remove left edge
				  poly.join_facet(h->opposite()->prev());										// Remove right edge
				  poly.join_vertex(h);//->facet()->semanticBLA = "asasd";						// Collapse edge
	} else {
		std::cerr<<"WARNING: Could not collapse edge."<<std::endl;
		return false;
	}
	return true;
}
