#ifndef GEOMETRIC_FUNCTIONS
#define GEOMETRIC_FUNCTIONS

#include "Vector.h"

typedef Vertex Point;
typedef Vector Edge;
typedef Vector Normal;
typedef double Angle;

#define PI 3.14159265

class GeometricFunctions {
public:
	
	// Calculates the dot product value of two vectors
	static double dotProduct(const Vector &a, const Vector &b);

	// Calculates the cross product value of two vectors
	static double crossProduct(const Vector &a, const Vector &b);

	// Checks whether the two line segments intersect or not
	static bool checkLineIntersection(const Edge &a, const Edge &b);

	// Finds the intersection point of two line segments
	const static Vertex findLineIntersection(const Edge &a, const Edge &b);

	// Calculates the normal vector of two vectors using the cross product
	const static Normal findNormal(const Vector &a, const Vector &b);

	// Calculates the angle between two vectors
	static Angle calculateAngle(const Vector &a, const Vector &b);

	// Converts degrees to radians
	static Angle degreesToRadians(const Angle angle);

	// Converts radians to degrees
	static Angle radiansToDegrees(const Angle angle);

	// Normalizes the given vector to a unit vector
	const static Vector normalizeVector(const Vector &vector);

};

#endif
