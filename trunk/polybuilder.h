#ifndef _INCL_POLYBUILDER
#define _INCL_POLYBUILDER

#include "globals.h"
#include "commonfunctions.h"

struct Plane_equation {
	template <class Facet>
	typename Facet::Plane_3 operator()( Facet& f) {
		typename Facet::Halfedge_handle h = f.halfedge();
		typedef typename Facet::Plane_3  Plane;
		return Plane( h->vertex()->point(),
			h->next()->vertex()->point(),
			h->next()->next()->vertex()->point());
	}
};

template<class HDS>
class polyhedron_builder;

Polyhedron buildSurface(polyTypeMap polygons, double extrudeDown);

#endif