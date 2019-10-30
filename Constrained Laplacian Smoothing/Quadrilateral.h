#ifndef QUADRILATERAL
#define QUADRILATERAL

#include "Primitive.h"

class Quadrilateral : public Primitive {
public:

	const static int POINT_COUNT = 4;

	NeighborhoodVertex* a;
	NeighborhoodVertex* b;
	NeighborhoodVertex* c;
	NeighborhoodVertex* d;

	Quadrilateral(NeighborhoodVertex* a, NeighborhoodVertex* b, NeighborhoodVertex* c, NeighborhoodVertex* d);
	Quadrilateral(Vertex a, Vertex b, Vertex c, Vertex d);

	void output(std::string filename) const;

};

#endif
