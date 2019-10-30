#include "Face.h"

Face::Face(unsigned int id, Quadrilateral* quadrilateral) {
	this->id = id;
	this->quadrilateral = quadrilateral;
}

Quadrilateral* Face::getQuadrilateral() {
	return quadrilateral;
}

