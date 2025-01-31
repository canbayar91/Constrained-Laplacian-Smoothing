#include "MeshReader.h"
#include <stdio.h>
#include <iostream>
#include <ctime>

int main(int argc, char *argv[]) {

	// Read the file name from command line
	std::string filename = argv[1];

	// Read how many times smoothing operation will be run
	int iterationCount = std::atoi(argv[2]);

	// Create an empty quadrilateral mesh
	Mesh* quadrilateralMesh = new Mesh();

	// Read the mesh from input file
	MeshReader::getInstance()->readQuadrilateralMesh(filename, quadrilateralMesh);

	// Start time of smoothing
	const clock_t beginTime = clock();

	// Update each vertex coordinate to the average of its neighbors
	for (int i = 0; i < iterationCount; i++) {
		quadrilateralMesh->smooth();
	}

	// End time of smoothing
	const clock_t endTime = clock();

	// Output the time difference
	float timeDifference = float(endTime - beginTime);
	std::cout << "Running time: " << timeDifference / CLOCKS_PER_SEC << std::endl;

	// Write the smoothed mesh to the output file
	MeshReader::getInstance()->writeQuadrilateralMesh("output.off", quadrilateralMesh);

	// Wait for user input
	std::cout << "Smoothing completed." << std::endl;
	getchar();

	// Delete the mesh to release memory
	delete quadrilateralMesh;

    return 0;
}

