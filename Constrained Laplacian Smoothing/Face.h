#ifndef FACE
#define FACE

#include "Quadrilateral.h"

class Face {
public:

	unsigned int id;

	Face(unsigned int id, Quadrilateral* quadrilateral);

	Quadrilateral* getQuadrilateral();

private:
	
	Quadrilateral* quadrilateral;

};

#endif
