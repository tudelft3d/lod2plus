#ifndef _INCL_GUARD
#define _INCL_GUARD

//Default includes needed
#include <vld.h> 
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <math.h>
#include <sstream>  
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <windows.h>
#include "boost/filesystem.hpp" 
#include <boost/algorithm/string.hpp>
#include <boost/chrono.hpp>
#include <boost/shared_ptr.hpp>

/*******************************************************/
/*Includes that are using the CGAL library*/
// Kernels
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/centroid.h>
#include <CGAL/Polyhedron_items_with_id_3.h>

// 2D polygons
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/convex_hull_2.h>

// Polyhedra & NEF
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Projection_traits_yz_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/IO/Nef_polyhedron_iostream_3.h>
#include <CGAL/OFF_to_nef_3.h>
#include <CGAL/Vector_3.h>
#include <CGAL/minkowski_sum_3.h>
#include <CGAL/Direction_3.h>
#include<CGAL/create_offset_polygons_2.h>
#include <CGAL/convex_decomposition_3.h> 
#include <CGAL/bounding_box.h>
#include <CGAL/Bbox_3.h>
#include <CGAL/Nef_nary_union_3.h> 

#include <CGAL/squared_distance_3.h>

//Simplification
#include <CGAL/Surface_mesh_simplification/HalfedgeGraph_Polyhedron_3.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_length_cost.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Midpoint_and_length.h>

using namespace std;

namespace SMS = CGAL::Surface_mesh_simplification ;

//CGAL TypeDefs
typedef CGAL::Exact_predicates_exact_constructions_kernel	Kernel; //Exact geometry constructions and exact 				
typedef CGAL::Projection_traits_xy_3<Kernel>				K;
typedef CGAL::Simple_cartesian<double>						Kernel_2;
typedef CGAL::Polyhedron_3<Kernel_2,CGAL::Polyhedron_items_with_id_3>						Surface;
typedef Kernel::Point_2										Point_2;
typedef Kernel::Point_3										Point_3;
typedef Kernel::Plane_3										Plane_3;
typedef CGAL::Triangle_3<Kernel>							Triangle_3;
typedef Kernel::Vector_3									Vector_3;
typedef Kernel::Direction_3									Direction_3;
typedef CGAL::Polyhedron_3<Kernel>							Polyhedron;
typedef Polyhedron::HalfedgeDS								HalfedgeDS;
typedef CGAL::Polyhedron_incremental_builder_3<HalfedgeDS>	polyIncr;
typedef CGAL::Nef_polyhedron_3<Kernel>						Nef_polyhedron;
typedef CGAL::Polygon_2<Kernel>								Polygon_2;
typedef CGAL::Aff_transformation_3<Kernel>					Transformation;
typedef Polyhedron::Halfedge_handle							Halfedge_handle;
typedef CGAL::Line_2<Kernel>								Line_2;
typedef CGAL::Line_3<Kernel>								Line_3;
typedef Kernel::Triangle_2									Triangle_2;
typedef Kernel::Iso_cuboid_3								bounding_box;


//CityModel type
typedef map<string,map<int,vector<Point_3>>>				polyTypeMap;
typedef map<int,vector<Point_3>>							polyNrMap;
typedef vector<Point_3>										pointVector;
typedef map<string,map<string,Polyhedron>>					buildingPolys;
typedef map<string,map<string,string>>						bagMap;

//Iterator typedefs
typedef polyTypeMap::iterator								itPolyType;
typedef polyNrMap::iterator									itPolyNr;
typedef buildingPolys::iterator								itBuildingPolys;
#endif