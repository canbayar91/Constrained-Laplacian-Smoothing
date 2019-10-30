#ifndef EDGE
#define EDGE

#include "Vertex.h"

class Vector {
public:

	Vertex start;
	Vertex end;

	Vector(const Vertex &start, const Vertex &end);
	Vector(const Vector& edge);

	const double getLength() const;

	const double getProductX() const;
	const double getProductY() const;
	const double getProductZ() const;

	const double getLengthX() const;
	const double getLengthY() const;
	const double getLengthZ() const;
	
};

#endif
