#ifndef MESH
#define MESH

#include "Face.h"
#include <vector>
#include <map>

class Mesh {
public:

	Mesh();
	~Mesh();

	void addVertex(unsigned int id, Vertex coordinates);
	void addFace(unsigned int id, Quadrilateral* quadrilateral);

	size_t getVertexCount();
	size_t getFaceCount();

	NeighborhoodVertex* getVertex(size_t index);
	Face* getFace(size_t index);

	void smooth();

	void setEdgeCount(unsigned int edgeCount);
	unsigned int getEdgeCount();

private:

	std::vector<NeighborhoodVertex*> vertexList;
	std::vector<Face*> faceList;

	std::map<unsigned int, std::vector<unsigned int>> vertexMapping;
	std::map<unsigned int, std::vector<unsigned int>> faceMapping;

	unsigned int edgeCount;

	void updateCoordinates_old(unsigned int index);
	void updateCoordinates(unsigned int index);

	const Vertex calculateAverageCoordinates(unsigned int index);

};

#endif
