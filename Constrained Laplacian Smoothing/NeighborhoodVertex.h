#ifndef NEIGHBORHOOD_VERTEX
#define NEIGHBORHOOD_VERTEX

#include "Vertex.h"

class NeighborhoodVertex {
public:

	unsigned int id;
	Vertex coordinates;

	NeighborhoodVertex(unsigned int id, Vertex coordinates);

};

#endif
