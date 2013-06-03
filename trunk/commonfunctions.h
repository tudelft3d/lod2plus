#include "globals.h"

string intToString(int number);
vector<int> makeUniqueVector (vector<int> v);
string stringToRatio (string coord, int dec);
string extractInfo (string toExtract, string first, string second);
vector<Point_3> posToVector (string posLine, bool epec);
map<string,vector<Line_2>> readSharedWalls(string fileName);
double strToDbl(string s);
int strToInt(string s);
string dblToStr(double d, int ndec);
bool isDegen(vector<Point_3> pointVector, double tol);
void removeFilesInDir(string directory, string extension);
vector<Nef_polyhedron> splitNefs (Nef_polyhedron N);
vector<vector<Point_3>> getFacets (Nef_polyhedron N);
Transformation getRotationMatrix(Vector_3 sourceVec, Vector_3 targetVec);
Nef_polyhedron createConvexHullNef (Nef_polyhedron& N);
void writeOffFile (Nef_polyhedron N, string file);
void fix_degenerate(Polyhedron& polyhe);
bool do_edgeCollapse(Polyhedron& poly, Polyhedron::Halfedge_handle h);
