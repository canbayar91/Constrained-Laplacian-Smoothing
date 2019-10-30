#include "Triangle.h"
#include <fstream>

Triangle::Triangle(NeighborhoodVertex* a, NeighborhoodVertex* b, NeighborhoodVertex* c) {
	this->a = a;
	this->b = b;
	this->c = c;

	searcher.insert(this->a);
	searcher.insert(this->b);
	searcher.insert(this->c);
}

Triangle::Triangle(Vertex a, Vertex b, Vertex c) {
	this->a = new NeighborhoodVertex(0, a);
	this->b = new NeighborhoodVertex(0, b);
	this->c = new NeighborhoodVertex(0, c);

	searcher.insert(this->a);
	searcher.insert(this->b);
	searcher.insert(this->c);
}

void Triangle::output(std::string filename) const {

	// Open the file in given location
	std::ofstream outfile(filename);

	// Write the file format
	outfile << "OFF" << std::endl;
	outfile << "3 1 3" << std::endl;

	// Write the vertex coordinates to file
	outfile << a->coordinates.x << " " << a->coordinates.y << " " << a->coordinates.z << std::endl;
	outfile << b->coordinates.x << " " << b->coordinates.y << " " << b->coordinates.z << std::endl;
	outfile << c->coordinates.x << " " << c->coordinates.y << " " << c->coordinates.z << std::endl;

	// Write the vertex order
	outfile << "3 0 1 2" << std::endl;

	// Close the input file
	outfile.close();
}
