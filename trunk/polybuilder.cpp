#include "polybuilder.h"
#include "commonfunctions.h"

template<class HDS>
class polyhedron_builder : public CGAL::Modifier_base<HDS> {
public:
	std::vector<Point_3> &coords;
	std::vector<vector<int>>    &faces;
	polyhedron_builder( vector<Point_3> &_coords, vector<vector<int>> &_faces) : coords(_coords), faces(_faces) {}
	void operator()( HDS& hds) {
		typedef typename HDS::Vertex   Vertex;
		typedef typename Vertex::Point Point;

		int Nfaces = faces.size();
		int Ncoords = coords.size();

		// create a cgal incremental builder
		CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
		B.begin_surface( Ncoords, Nfaces);

		// add the polyhedron vertices
		for( int i=0; i!=coords.size(); i++ ){
			B.add_vertex(coords[i]);
		}

		// add the polyhedron faces
		for( int i=0; i!=faces.size(); i++){
			if(B.test_facet(faces[i].begin(),faces[i].end()))
			{
				B.begin_facet();
				for ( int j=0; j!=faces[i].size(); j++) {
					B.add_vertex_to_facet(faces[i][j]);
				}
				B.end_facet();
			}
		}

		// finish up the surface
		B.end_surface();
	}
};

Polyhedron buildSurface(polyTypeMap polygons, double extrudeDown=0.0) {
	vector<Point_3> pointsVector;
	vector<vector<int>> faceVector;
	int pointFound = 0;

	itPolyType itPT;
	itPolyNr itPNr;

	for(itPT = polygons.begin(); itPT != polygons.end(); itPT++) {
		polyNrMap polyNr = itPT->second;
		vector<int> tempFaceVector;
		for(itPNr = polyNr.begin(); itPNr != polyNr.end(); itPNr++) {
			pointVector v = itPNr -> second;
			for (int iPoints = 0; iPoints != v.size(); iPoints++){

				pointFound=0;
				Point_3 tempPnt = v[iPoints];
				if (abs(tempPnt.z()) < 0.001 && extrudeDown != 0.0) {
					tempPnt = Point_3 (tempPnt.x(),tempPnt.y(),extrudeDown);
				}
				for (int j = 0; j!=pointsVector.size();j++){
					if (CGAL::squared_distance(tempPnt,pointsVector[j]) < 0.00001){
						tempFaceVector.push_back(j);
						pointFound=1;
					}
				}
				if (pointFound==0) {
					pointsVector.push_back(tempPnt);
					tempFaceVector.push_back(pointsVector.size()-1);
				}
			}
			vector<int> newTempFaceVector = makeUniqueVector(tempFaceVector);
			if (newTempFaceVector.size() > 2) {
				faceVector.push_back(newTempFaceVector);
			}
			tempFaceVector.clear();
			newTempFaceVector.clear();
		}
	}

	Polyhedron P;
	polyhedron_builder<HalfedgeDS> builder( pointsVector, faceVector );
	P.delegate( builder );
	P.normalize_border();
	return P;
}

