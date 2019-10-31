#include "Mesh.h"
#include "ProjectionFunctions.h"
#include <iostream>

Mesh::Mesh() {
	// Default constructor
}

Mesh::~Mesh() {
	for (size_t i = 0; i < vertexList.size(); i++) {
		delete vertexList[i];
	}
}

void Mesh::addVertex(unsigned int id, Vertex coordinates) {
	NeighborhoodVertex* vertex = new NeighborhoodVertex(id, coordinates);
	vertexList.push_back(vertex);
}

void Mesh::addFace(unsigned int id, Quadrilateral* quadrilateral) {

	NeighborhoodVertex* a = quadrilateral->a;
	NeighborhoodVertex* b = quadrilateral->b;
	NeighborhoodVertex* c = quadrilateral->c;
	NeighborhoodVertex* d = quadrilateral->d;

	vertexMapping[a->id].push_back(b->id);
	vertexMapping[b->id].push_back(c->id);
	vertexMapping[c->id].push_back(d->id);
	vertexMapping[d->id].push_back(a->id);

	vertexMapping[a->id].push_back(d->id);
	vertexMapping[b->id].push_back(a->id);
	vertexMapping[c->id].push_back(b->id);
	vertexMapping[d->id].push_back(c->id);

	Face* face = new Face(id, quadrilateral);
	faceList.push_back(face);

	faceMapping[a->id].push_back(id);
	faceMapping[b->id].push_back(id);
	faceMapping[c->id].push_back(id);
	faceMapping[d->id].push_back(id);
}

size_t Mesh::getVertexCount() {
	return vertexList.size();
}

size_t Mesh::getFaceCount() {
	return faceList.size();
}

NeighborhoodVertex* Mesh::getVertex(size_t index) {
	return vertexList[index];
}

Face* Mesh::getFace(size_t index) {
	return faceList[index];
}

void Mesh::setEdgeCount(unsigned int edgeCount) {
	this->edgeCount = edgeCount;
}

unsigned int Mesh::getEdgeCount() {
	return edgeCount;
}

void Mesh::smooth() {

	/* DEBUG CODE - double maxLength = 0;
	for (size_t i = 0; i < faceList.size(); i++) {
		Quadrilateral* current = faceList[i]->getQuadrilateral();

		Vector AB(current->a->coordinates, current->b->coordinates);
		Vector BC(current->b->coordinates, current->c->coordinates);
		Vector CD(current->c->coordinates, current->d->coordinates);
		Vector DA(current->d->coordinates, current->a->coordinates);

		maxLength = (AB.getLength() > maxLength) ? AB.getLength() : maxLength;
		maxLength = (BC.getLength() > maxLength) ? BC.getLength() : maxLength;
		maxLength = (CD.getLength() > maxLength) ? CD.getLength() : maxLength;
		maxLength = (DA.getLength() > maxLength) ? DA.getLength() : maxLength;
	}

	std::cout << "Maximum Edge Length: " << maxLength << std::endl; */

	for (size_t i = 0; i < vertexList.size(); i++) {
		updateCoordinates(i);
	}
}

void Mesh::updateCoordinates_old(unsigned int index) {

	// Calculate the average coordinates for the vertex
	const Vertex average = calculateAverageCoordinates(index);

	// We want to iterate through each face and project the point onto each plane defined by the face
	// When we find the face that the projected vertex lies, the position of the projected point is the new position of the vertex
	std::vector<unsigned int> neighborFaceList = faceMapping[index];
	for (unsigned int faceId : neighborFaceList) {

		// Project the quadrilateral onto a plane
		Quadrilateral* quad = faceList[faceId]->getQuadrilateral();
		Quadrilateral projectedQuad = ProjectionFunctions::projectQuadrilateral(quad);

		// Vector test(quad->a->coordinates, projectedQuad.a->coordinates);
		// std::cout << test.getLength() << std::endl;

		// Find the normal of the quadrilateral using two of its edges
		const Vector AB(projectedQuad.a->coordinates, projectedQuad.b->coordinates);
		const Vector AD(projectedQuad.a->coordinates, projectedQuad.d->coordinates);
		const Normal normal = GeometricFunctions::findNormal(AB, AD);

		// Create a vector from the start of normal to our point
		const Vector vector(normal.start, average);

		// Project the point onto the normal vector
		const Angle theta = GeometricFunctions::calculateAngle(normal, vector);
		const Angle radians = GeometricFunctions::degreesToRadians(theta);
		const Vertex dotProduct = Vertex(normal.getProductX(), normal.getProductY(), normal.getProductZ()) * cos(radians);

		// Subtract the dot product from the original point to find the projected location
		const Vertex projectedVertex = average - dotProduct;

		// If the projected vertex is inside the quadrilateral, update the original vertex coordinates
		// Note: I decided not to use this approach because the check requires projection to z = 0
		// if (projectedQuad.checkInside(projectedVertex)) {
		NeighborhoodVertex* current = vertexList[index];
		current->coordinates.x = projectedVertex.x;
		current->coordinates.y = projectedVertex.y;
		current->coordinates.z = projectedVertex.z;
		// break;
		// }
	}
}

void Mesh::updateCoordinates(unsigned int index) {

	// Calculate the average coordinates for the vertex
	const Vertex average = calculateAverageCoordinates(index);

	// Get the vertex with the given index
	NeighborhoodVertex* current = vertexList[index];

	// We want to iterate through each face and project the point onto each plane defined by the face
	// When we find the face that the projected vertex lies, the position of the projected point is the new position of the vertex
	std::vector<unsigned int> neighborFaceList = faceMapping[index];
	for (unsigned int faceId : neighborFaceList) {

		// For each face, get the quadrilateral data
		Quadrilateral* face = faceList[faceId]->getQuadrilateral();

		// Find the node that stores the given vertex 
		Node* node = face->findNode(current);
		if (node != NULL) {

			// Find its next and previous vertices
			NeighborhoodVertex* next = node->next->data;
			NeighborhoodVertex* previous = node->prev->data;

			// Find the normal of the quadrilateral using two of its edges
			const Vector AB(current->coordinates, next->coordinates);
			const Vector AC(current->coordinates, previous->coordinates);
			const Normal normal = GeometricFunctions::findNormal(AB, AC);

			// Create a vector from the start of normal to our point
			const Vector vector(normal.start, average);

			// Calculate the angle between the normal and the vector
			const Angle theta = GeometricFunctions::calculateAngle(normal, vector);
			const Angle radians = GeometricFunctions::degreesToRadians(theta);

			// Get the signed length of the vector on each coordinate plane
			double productX = vector.getProductX();
			double productY = vector.getProductY();
			double productZ = vector.getProductZ();

			// Project the point onto the normal vector using dot product formula
			const Vertex dotProduct = Vertex(productX, productY, productZ) * cos(radians);

			// Subtract the dot product from the original point to find the projected location
			const Vertex projectedVertex = average - dotProduct;

			// Create a vector from the original vertex location to projected vertex location
			const Vector projectionVector(current->coordinates, projectedVertex);

			// Check if the vector lies in the current triangle by using angles 
			// If the projected vertex is inside the triangle, update the original vertex coordinates
			const Angle triangleAngle = GeometricFunctions::calculateAngle(AB, AC);
			const Angle projectionAngle1 = GeometricFunctions::calculateAngle(AB, projectionVector);
			const Angle projectionAngle2 = GeometricFunctions::calculateAngle(AC, projectionVector);
			if (projectionAngle1 < triangleAngle && projectionAngle2 < triangleAngle) {
				current->coordinates.x = projectedVertex.x;
				current->coordinates.y = projectedVertex.y;
				current->coordinates.z = projectedVertex.z;
				break;
			}
		}
	}
}

const Vertex  Mesh::calculateAverageCoordinates(unsigned int index) {

	double totalX = 0;
	double totalY = 0;
	double totalZ = 0;

	std::vector<unsigned int> neighbors = vertexMapping[index];
	for (unsigned int vertexId : neighbors) {
		NeighborhoodVertex* neighbor = vertexList[vertexId];
		totalX += neighbor->coordinates.x;
		totalY += neighbor->coordinates.y;
		totalZ += neighbor->coordinates.z;
	}

	unsigned int neighborCount = neighbors.size();

	double x = totalX / neighborCount;
	double y = totalY / neighborCount;
	double z = totalZ / neighborCount;

	return Vertex(x, y, z);
}
