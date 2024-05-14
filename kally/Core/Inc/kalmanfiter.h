/*
 * KalmanFilter.h
 *
 *  Created on: May 5, 2024
 *      Author: Talae
 */

#ifndef INC_KALMANFILTER_H_
#define INC_KALMANFILTER_H_

#include "matrix.h"
#include "stm32g4xx_hal.h"

typedef struct{

matrix A, B, C, D, G, Q, R;
matrix S;
matrix P_old, P_new;
matrix K_gain;
matrix Y, Y_Error, X_correct, P_correct;
matrix predictX_new, predictX_old;
matrix I44;
matrix U;
matrix result_X, result_Y;
float U_In[1];
float Y_measure[1];
float estimate_State[4];

}KalmanFilter;

void KalmanFilter_Init(KalmanFilter* KF, float* A_data, float* _B_data, float* _C_data, float* _G_data, float* _Q_data, float* _R_data);
void Compute(KalmanFilter* KF, double _position); //, float _Vin

#endif /* INC_KALMANFILTER_H_ */
