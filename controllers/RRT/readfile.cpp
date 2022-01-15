#include <iostream>
#include <fstream>
#include "readfile.hpp"

#define COLUMNS 2 // Number of coordinates (xi, zi) 
#define MAX_NODES 2000 // Maximum number allowed of nodes to visit

int count_nodes = 0;

float **coordinates_to_follow(void)
{
	float auxiliar[2000][2];

	/* Reding .txt file */
	std::ifstream input_file;
	input_file.open("/home/angelo/Desktop/test.txt");

	if (!input_file)
	{
		std::cerr << "Error finding input file";
	}

	while (!input_file.eof())
	{		
		/* Creates nx2 array */
		for (int i = 0; i < COLUMNS; ++i)
		{
			input_file >> auxiliar[count_nodes][i];
		}

		count_nodes++;
	}

	/* Dynamically allocating the coordinates */
	float  **coordinates = new float*[count_nodes];

	for (int i = 0; i < count_nodes; ++i)
	{
		coordinates[i] = new float[COLUMNS];

		for (int j = 0; j < COLUMNS; ++j)
		{
			coordinates[i][j] = auxiliar[i][j]; 
		}
	}

	return coordinates;
}

int nodes_to_follow(void)
{
	return count_nodes; // Number of nodes to follow
}