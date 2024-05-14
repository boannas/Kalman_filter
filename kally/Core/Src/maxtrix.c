/*
 * matrix.c
 *
 *  Created on: May 5, 2024
 *      Author: Talae
 */

#include "matrix.h"
#include <math.h>


//matrix matrix_init(int row, int col,float* data_in);
//matrix matrix_plus(matrix* in1, matrix* in2);
//matrix matrix_minus(matrix* in1, matrix* in2);
//matrix matrix_multiply(matrix* in1, matrix* in2);
//matrix matrix_gain(matrix* in,float gain);
//matrix matrix_transpose(matrix* in);
//float matrix_determinant(matrix* in);
//matrix matrix_inverse(matrix* in);
//matrix matrix_update(matrix* in,float* _update);

matrix matrix_init(int row, int col, float *_data_in){
	matrix result;
	result.d_row = row;
	result.d_col = col;

	for (int i = 0; i < row; i++){
		for (int j = 0; j < col; ++j){
			result.data[i][j] = _data_in[i * col + j];
		}
	}

	return result;
}

matrix matrix_update(matrix* in,float* _update){
	for (int i = 0; i < in->d_row; i++){
		for (int j = 0; j < in->d_col; j++){
			in->data[i][j] = _update[i * in->d_col + j];
		}
	}
	return *in;
}

matrix matrix_plus(matrix* in1, matrix* in2){
	matrix result;
	result.d_row = in1->d_row;
	result.d_col = in1->d_col;

	for (int row = 0; row < in1->d_row; row++){
		for (int col = 0; col < in1->d_col; col++){
			result.data[row][col] = in1->data[row][col] + in2->data[row][col];
		}
	}

	return result;
}

matrix matrix_minus(matrix* in1, matrix* in2){
	matrix result;
	result.d_row = in1->d_row;
	result.d_col = in1->d_col;

	for (int row = 0; row < in1->d_row; row++){
		for (int col = 0; col < in1->d_col; col++){
			result.data[row][col] = in1->data[row][col] - in2->data[row][col];
		}
	}

	return result;
}

matrix matrix_multiply(matrix* in1, matrix* in2){
	matrix result;
	result.d_row = in1->d_row;
	result.d_col = in2->d_col;

	for (int m = 0; m < in1->d_row; m++) {
		for (int n = 0; n < in2->d_col; n++) {
			float total = 0;
			for (int k = 0; k < in1->d_col; k++) {
				total = total + (in1->data[m][k] * in2->data[k][n]);
			}
			result.data[m][n] = total;
		}
	}

	return result;
}

matrix matrix_gain(matrix* in,float gain){
	matrix result;
	result.d_row = in->d_row;
	result.d_col = in->d_col;

	for (int m = 0; m < in->d_row; m++) {
		for (int n = 0; n < in->d_col; n++) {
			result.data[m][n] = in->data[m][n]*gain;
		}
	}

	return result;
}

matrix matrix_transpose(matrix* in){
	matrix result;
	result.d_row = in->d_col;
	result.d_col = in->d_row;

	for (int m = 0; m < in->d_row; m++) {
		for (int n = 0; n < in->d_col; n++) {
			result.data[n][m] = in->data[m][n];
		}
	}

	return result;
}

float matrix_determinant(matrix* in){
	float det_val = 0;
	if (in->d_row == 3 && in->d_col == 3){
		for (int i = 0; i < 3; i++){
			det_val = det_val + (in->data[0][i] * (in->data[1][(i + 1) % 3] * in->data[2][(i + 2) % 3]
								- in->data[1][(i + 2) % 3] * in->data[2][(i + 1) % 3]));
		}
	}
	else if (in->d_row == 2 && in->d_col == 2){
		det_val = (in->data[0][0] * in->data[1][1]) - (in->data[0][1] * in->data[1][0]);
	}
	else
		det_val = 10;

	return det_val;
}

matrix matrix_inverse(matrix* in2){
	float det = 0;
	det = matrix_determinant(in2);
	float num[9] = {0,0,0,0,0,0,0,0,0,0};
	matrix result = matrix_init(3, 3, &num);
	if (in2->d_row == 3 && in2->d_col == 3){
		for (int i = 0; i < 3; i++){
			for (int j = 0; j < 3; j++){
				result.data[j][i] = ((in2->data[(i + 1) % 3][(j + 1) % 3] * in2->data[(i + 2) % 3][(j + 2) % 3])
						- (in2->data[(i + 1) % 3][(j + 2) % 3] * in2->data[(i + 2) % 3][(j + 1) % 3])) * pow(-1,(i+j)) / det;
			}
		}
	}
	else if (in2->d_row == 2 && in2->d_col == 2){
		result.data[0][0] = in2->data[1][1] * 1.0 / det;
		result.data[0][1] = -1 * in2->data[0][1] * 1.0 / det;
		result.data[1][0] = -1 * in2->data[1][0] * 1.0 / det;
		result.data[1][1] = in2->data[0][0] * 1.0 / det;

	}
	else{
		result.data[0][0] = 1/in2->data[0][0];
	}
	result.d_col = det;
	result.d_row = in2->d_row;
	return result;
}
