/*
 * matrix.h
 *
 *  Created on: May 5, 2024
 *      Author: Talae
 */

#ifndef INC_MATRIX_H_
#define INC_MATRIX_H_

#include "stm32g4xx_hal.h"
#include "string.h"
#include <stdlib.h>
#include "main.h"

typedef struct matrixStruct{
int d_row;
int d_col;
float data[4][4];

}matrix;

matrix matrix_init(int row, int col, float *_data_in);
matrix matrix_plus(matrix* in1, matrix* in2);
matrix matrix_minus(matrix* in1, matrix* in2);
matrix matrix_multiply(matrix* in1, matrix* in2);
matrix matrix_gain(matrix* in,float gain);
matrix matrix_transpose(matrix* in);
float matrix_determinant(matrix* in);
matrix matrix_inverse(matrix* in2);
matrix matrix_update(matrix* in,float* _update);

#endif /* INC_MATRIX_H_ */
