#include "Quadrilateral.h"
#include <fstream>

Quadrilateral::Quadrilateral(NeighborhoodVertex* a, NeighborhoodVertex* b, NeighborhoodVertex* c, NeighborhoodVertex* d) {
	this->a = a;
	this->b = b;
	this->c = c;
	this->d = d;

	searcher.insert(this->a);
	searcher.insert(this->b);
	searcher.insert(this->c);
	searcher.insert(this->d);
}

Quadrilateral::Quadrilateral(Vertex a, Vertex b, Vertex c, Vertex d) {
	this->a = new NeighborhoodVertex(0, a);
	this->b = new NeighborhoodVertex(0, b);
	this->c = new NeighborhoodVertex(0, c);
	this->d = new NeighborhoodVertex(0, d);

	searcher.insert(this->a);
	searcher.insert(this->b);
	searcher.insert(this->c);
	searcher.insert(this->d);
}

void Quadrilateral::output(std::string filename) const {

	// Open the file in given location
	std::ofstream outfile(filename);

	// Write the file format
	outfile << "OFF" << std::endl;
	outfile << "4 1 4" << std::endl;	

	// Write the vertex coordinates to file
	outfile << a->coordinates.x << " " << a->coordinates.y << " " << a->coordinates.z << std::endl;
	outfile << b->coordinates.x << " " << b->coordinates.y << " " << b->coordinates.z << std::endl;
	outfile << c->coordinates.x << " " << c->coordinates.y << " " << c->coordinates.z << std::endl;
	outfile << d->coordinates.x << " " << d->coordinates.y << " " << d->coordinates.z << std::endl;

	// Write the vertex order
	outfile << "4 0 1 2 3" << std::endl;

	// Close the input file
	outfile.close();
}
