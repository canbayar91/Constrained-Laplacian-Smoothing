#include "Vector.h"
#include <cmath>

Vector::Vector(const Vertex &start, const Vertex &end) {
	this->start = start;
	this->end = end;
}

Vector::Vector(const Vector& edge) {
	this->start = edge.start;
	this->end = edge.end;
}

const double Vector::getLength() const {
	double componentX = pow(getLengthX(), 2);
	double componentY = pow(getLengthY(), 2);
	double componentZ = pow(getLengthZ(), 2);
	return sqrt(componentX + componentY + componentZ);
}

const double Vector::getProductX() const {
	return end.x - start.x;
}

const double Vector::getProductY() const {
	return end.y - start.y;
}

const double Vector::getProductZ() const {
	return end.z - start.z;
}

const double Vector::getLengthX() const {
	return abs(getProductX());
}

const double Vector::getLengthY() const {
	return abs(getProductY());
}

const double Vector::getLengthZ() const {
	return abs(getProductZ());
}
