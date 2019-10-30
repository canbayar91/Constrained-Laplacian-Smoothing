#ifndef PROJECTION_FUNCTIONS
#define PROJECTION_FUNCTIONS

#include "GeometricFunctions.h"
#include "Quadrilateral.h"
#include "Triangle.h"

class ProjectionFunctions {
public:
	
	// Projects a triangle to the z=0 plane
	const static void projectTriangle(Triangle* &triangle);

	// Projects a non-planar quadrilateral onto an average plane
	const static Quadrilateral projectQuadrilateral(const Quadrilateral* quadrilateral);

private:

	// Calculates the average of the corner points in a quadrilateral
	const static Vertex calculateCornerAverage(const Quadrilateral* quadrilateral);

	// Calculates the average of the normals on corner points in a quadrilateral
	const static Normal calculateNormalAverage(const Quadrilateral* quadrilateral);

};

#endif
