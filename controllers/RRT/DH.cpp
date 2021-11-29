#include <iostream>
#include <cmath>
#include "DH.hpp"

#define COLUMNS 4
#define ROWS 4

float **dh(float theta, float d, float a, float alpha)
{	
	/* Create a 4x4 matrix full of zeros */
	float **T = new float*[COLUMNS];

	for (int i = 0; i < ROWS; ++i)
	{
		T[i] = new float[ROWS];
		
		for (int j = 0; j < ROWS; ++j)
		{
			T[i][j] = 0;
		}
	}

	/* Fill the matrix according to a D-H homogeneous matrix */
	T[0][0] = cos(theta);
	T[0][1] = -sin(theta) * cos(alpha);
	T[0][2] = sin(theta) * sin(alpha);
	T[0][3] = a * cos(theta);
	
	T[1][0] = sin(theta);
	T[1][1] = cos(theta) * cos(alpha);
	T[1][2] = -cos(theta) * sin(alpha);
	T[1][3] = a * sin(theta);

	T[2][0] = 0;
	T[2][1] = sin(alpha);
	T[2][2] = cos(alpha);
	T[2][3] = d;

	T[3][0] = 0;
	T[3][1] = 0;
	T[3][2] = 0;
	T[3][3] = 1;

	return T;
}

float **multiply_matrices(float **matrix_1, float **matrix_2)
{
	// Only multiplies 4x4 matrices

	/* Create a 4x4 matrix full of zeros */
	float **result = new float*[COLUMNS];

	for (int i = 0; i < ROWS; ++i)
	{
		result[i] = new float[ROWS];

		for (int j = 0; j < ROWS; ++j)
		{
			result[i][j] = 0;
		}
	}

	for (int i = 0; i < COLUMNS; ++i)
	{
		for (int j = 0; j < ROWS; ++j)
		{
			for (int k = 0; k < COLUMNS; ++k)
			{				
				result[i][j] += matrix_1[i][k] * matrix_2[k][j];
			}
		}
	}

	return result;
}