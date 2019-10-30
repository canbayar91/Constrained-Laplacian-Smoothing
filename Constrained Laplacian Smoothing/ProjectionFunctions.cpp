#include "ProjectionFunctions.h"

const Vertex ProjectionFunctions::calculateCornerAverage(const Quadrilateral* quadrilateral) {

	// Get the coordinates
	Vertex a = quadrilateral->a->coordinates;
	Vertex b = quadrilateral->b->coordinates;
	Vertex c = quadrilateral->c->coordinates;
	Vertex d = quadrilateral->d->coordinates;

	// Create and return the average vertex
	return (a + b + c + d) / Quadrilateral::POINT_COUNT;
}

const Normal ProjectionFunctions::calculateNormalAverage(const Quadrilateral* quadrilateral) {

	// Calculate the normals on the first vertex
	const Vector AB(quadrilateral->a->coordinates, quadrilateral->b->coordinates);
	const Vector AD(quadrilateral->a->coordinates, quadrilateral->d->coordinates);
	const Normal normalA = GeometricFunctions::findNormal(AB, AD);

	// Calculate the normals on the second vertex
	const Vector BC(quadrilateral->b->coordinates, quadrilateral->c->coordinates);
	const Vector BA(quadrilateral->b->coordinates, quadrilateral->a->coordinates);
	const Normal normalB = GeometricFunctions::findNormal(BC, BA);

	// Calculate the normals on the third vertex
	const Vector CD(quadrilateral->c->coordinates, quadrilateral->d->coordinates);
	const Vector CB(quadrilateral->c->coordinates, quadrilateral->b->coordinates);
	const Normal normalC = GeometricFunctions::findNormal(CD, CB);

	// Calculate the normals on the fourth vertex
	const Vector DA(quadrilateral->d->coordinates, quadrilateral->a->coordinates);
	const Vector DC(quadrilateral->d->coordinates, quadrilateral->c->coordinates);
	const Normal normalD = GeometricFunctions::findNormal(DA, DC);

	// Create start and end vertices for the normal
	const Vertex startVertex = (normalA.start + normalB.start + normalC.start + normalD.start) / Quadrilateral::POINT_COUNT;
	const Vertex endVertex = (normalA.end + normalB.end + normalC.end + normalD.end) / Quadrilateral::POINT_COUNT;

	// Create the average normal
	const Normal average(startVertex, endVertex);

	// Normalize the average normal into a unit vector
	const Normal normal = GeometricFunctions::normalizeVector(average);

	// Return the unit normal
	return normal;
}

const void ProjectionFunctions::projectTriangle(Triangle* &triangle) {

	// Create a temporary edge to complete the triangle
	const Vector leftover(triangle->c->coordinates, triangle->b->coordinates);

	// Get the vectors on triangle adjacent to vertex A
	Vector left(triangle->a->coordinates, triangle->b->coordinates);
	Vector right(triangle->a->coordinates, triangle->c->coordinates);

	// Get the edge lengths
	double leftEdgeLength = left.getLength();
	double rightEdgeLength = right.getLength();
	double leftoverLength = leftover.getLength();

	// Calculate the x-y values for the third vertex
	double x = (pow(leftEdgeLength, 2) - pow(leftoverLength, 2) + pow(rightEdgeLength, 2)) / (2 * rightEdgeLength);
	double y = sqrt(abs(pow(leftEdgeLength, 2) - pow(x, 2)));

	// Create vertices that are aligned with z=0 plane
	const Vertex middleVertex(0, 0);
	const Vertex rightVertex(rightEdgeLength, 0);
	const Vertex leftVertex(x, y);

	// Update the edge coordinates
	triangle->a->coordinates = middleVertex;
	triangle->b->coordinates = leftVertex;
	triangle->c->coordinates = rightVertex;
}

const Quadrilateral ProjectionFunctions::projectQuadrilateral(const Quadrilateral* quadrilateral) {

	// Calculate the average of the corner points
	const Vertex origin = calculateCornerAverage(quadrilateral);

	// Calculate the average of the normals of the corner points
	const Normal normal = calculateNormalAverage(quadrilateral);

	// Create vectors from the origin to the each vertex of the quadrilateral
	const Vector vectorA(origin, quadrilateral->a->coordinates);
	const Vector vectorB(origin, quadrilateral->b->coordinates);
	const Vector vectorC(origin, quadrilateral->c->coordinates);
	const Vector vectorD(origin, quadrilateral->d->coordinates);

	// Calculate the distance to the plane by taking dot product of each vertex with the normal (|normal| = 1)
	double distanceA = GeometricFunctions::dotProduct(normal, vectorA);
	double distanceB = GeometricFunctions::dotProduct(normal, vectorB);
	double distanceC = GeometricFunctions::dotProduct(normal, vectorC);
	double distanceD = GeometricFunctions::dotProduct(normal, vectorD);

	// Calculate the projection point for the first vertex
	double x = quadrilateral->a->coordinates.x - distanceA * normal.getLengthX();
	double y = quadrilateral->a->coordinates.y - distanceA * normal.getLengthY();
	double z = quadrilateral->a->coordinates.z - distanceA * normal.getLengthZ();
	const Vertex projectedA(x, y, z);

	// Calculate the projection point for the second vertex
	x = quadrilateral->b->coordinates.x - distanceB * normal.getLengthX();
	y = quadrilateral->b->coordinates.y - distanceB * normal.getLengthY();
	z = quadrilateral->b->coordinates.z - distanceB * normal.getLengthZ();
	const Vertex projectedB(x, y, z);

	// Calculate the projection point for the third vertex
	x = quadrilateral->c->coordinates.x - distanceC * normal.getLengthX();
	y = quadrilateral->c->coordinates.y - distanceC * normal.getLengthY();
	z = quadrilateral->c->coordinates.z - distanceC * normal.getLengthZ();
	const Vertex projectedC(x, y, z);

	// Calculate the projection point for the fourth vertex
	x = quadrilateral->d->coordinates.x - distanceD * normal.getLengthX();
	y = quadrilateral->d->coordinates.y - distanceD * normal.getLengthY();
	z = quadrilateral->d->coordinates.z - distanceD * normal.getLengthZ();
	const Vertex projectedD(x, y, z);

	// Create the quadrilateral with projected vertices
	const Quadrilateral projection(projectedA, projectedB, projectedC, projectedD);

	// Return the projected quadrilateral
	return projection;
}
