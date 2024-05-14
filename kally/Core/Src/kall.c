/*
 * KalmanFilter.c
 *
 *  Created on: May 5, 2024
 *      Author: Talae
 */

#include "kalmanfiter.h"

void Do_Innovation_Covariance(KalmanFilter* KF);
void Do_Optimal_Kalman_Gain(KalmanFilter* KF);
void Do_Error_Y(KalmanFilter* KF);
void Do_Correct_X(KalmanFilter* KF);
void Do_Correct_P(KalmanFilter* KF);
void Do_Predict_X(KalmanFilter* KF);
void Do_Predict_P(KalmanFilter* KF);
void Do_Result(KalmanFilter* KF);
void Run_KalmanFilter(KalmanFilter* KF);

void KalmanFilter_Init(KalmanFilter* KF, float* A_data, float *_B_data, float *_C_data, float *_G_data, float *_Q_data, float *_R_data){

//	*KF = &_KF;

	KF->A = matrix_init(4, 4, A_data);

	KF->B = matrix_init(4, 1, _B_data);

	KF->C = matrix_init(1, 4, _C_data);

	float _D_Data[4] = {0, 0, 0, 0};
	KF->D = matrix_init(4, 1, &_D_Data);

	KF->G = matrix_init(4, 1, _G_data);

	KF->Q = matrix_init(1, 1, _Q_data);

	KF->R = matrix_init(1, 1, _R_data);

	float _S_Data[1] = {0};
	KF->S = matrix_init(1, 1, &_S_Data);

	float _P_old_Data[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	KF->P_old = matrix_init(4, 4, &_P_old_Data);

	float _P_new_Data[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	KF->P_new = matrix_init(4, 4, &_P_new_Data);

	float _K_Gain_Data[4] = {0, 0, 0, 0};
	KF->K_gain = matrix_init(4, 1, &_K_Gain_Data);

	float Y_Data[1] = {0};
	KF->Y = matrix_init(1, 1, &Y_Data);

	float Y_error_Data[1] = {0};
	KF->Y_Error = matrix_init(1, 1, &Y_error_Data);

	float predictX_old_Data[4] = {0,0,0,0};
	KF->predictX_old = matrix_init(4, 1, &predictX_old_Data);

	float X_correct_Data[4] = {0, 0, 0, 0};
	KF->X_correct = matrix_init(4, 1, &X_correct_Data);

	float _I44_Data[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
	KF->I44 = matrix_init(4, 4, &_I44_Data);

	float _U_Data[1] = {0};
	KF->U = matrix_init(1, 1, &_U_Data);

}

void Do_Innovation_Covariance(KalmanFilter* KF){
	float __S_Data[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	matrix _S = matrix_init(4, 4, &__S_Data);

	_S = matrix_multiply(&KF->C, &KF->P_old);
	matrix C_transpose = matrix_transpose(&KF->C);
	_S = matrix_multiply(&_S, &C_transpose);
	KF->S = matrix_plus(&_S, &KF->R);
}

void Do_Optimal_Kalman_Gain(KalmanFilter* KF){
	matrix S_inverse = matrix_inverse(&KF->S);
	matrix C_transpose = matrix_transpose(&KF->C);
	matrix K_gain_1 = matrix_multiply(&KF->P_old, &C_transpose);
	KF->K_gain = matrix_gain(&K_gain_1, S_inverse.data[0][0]);
}

void Do_Error_Y(KalmanFilter* KF){
	matrix Y_Error1 = matrix_multiply(&KF->C, &KF->predictX_old);
	KF->Y_Error = matrix_minus(&KF->Y, &Y_Error1);
}

void Do_Correct_X(KalmanFilter* KF){
	matrix X_correct1 = matrix_multiply(&KF->K_gain, &KF->Y_Error);
	KF->X_correct = matrix_plus(&KF->predictX_old, &X_correct1);
}

void Do_Correct_P(KalmanFilter* KF){
	float _correctP_Data[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	matrix _correctP = matrix_init(4, 4, &_correctP_Data);
	matrix _correctP2 = matrix_multiply(&KF->K_gain, &KF->C);

	_correctP = matrix_minus(&KF->I44, &_correctP2);
	KF->P_correct = matrix_multiply(&_correctP, &KF->P_old);
}

void Do_Predict_X(KalmanFilter* KF){
	float predictX_new_1_data[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	float predictX_new_2_data[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	matrix _predictX_new_1 = matrix_init(4, 4, &predictX_new_1_data);
	matrix _predictX_new_2 = matrix_init(4, 4, &predictX_new_2_data);

	_predictX_new_1 = matrix_multiply(&KF->A, &KF->X_correct); //edit
	_predictX_new_2 = matrix_multiply(&KF->B, &KF->U);
	KF->predictX_new = matrix_plus(&_predictX_new_1, &_predictX_new_2);
}

void Do_Predict_P(KalmanFilter* KF){
	float predictP_new_1_data[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	float predictP_new_2_data[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	matrix _predictP_new_1 = matrix_init(4, 4, &predictP_new_1_data);
	matrix _predictP_new_2 = matrix_init(4, 4, &predictP_new_2_data);

	matrix A_transpose = matrix_transpose(&KF->A);
	matrix G_transpose = matrix_transpose(&KF->G);

	matrix _predictP_new_11 = matrix_multiply(&KF->A, &KF->P_correct); //edit
	matrix _predictP_new_21 = matrix_multiply(&KF->G, &KF->Q);

	_predictP_new_1 = matrix_multiply(&_predictP_new_11, &A_transpose);
	_predictP_new_2 = matrix_multiply(&_predictP_new_21, &G_transpose);
	KF->P_new = matrix_plus(&_predictP_new_1, &_predictP_new_2);

}

void Do_Result(KalmanFilter* KF){
	KF->result_X = KF->predictX_new;
	matrix result_Y1 = matrix_multiply(&KF->C, &KF->predictX_new);
	matrix result_Y2 = matrix_multiply(&KF->D, &KF->U);
	KF->result_Y = matrix_plus(&result_Y1, &result_Y2);
}

void Run_KalmanFilter(KalmanFilter* KF){

	// Correct
	Do_Innovation_Covariance(KF);
	Do_Optimal_Kalman_Gain(KF);
	Do_Error_Y(KF);
	Do_Correct_X(KF);
	Do_Correct_P(KF);

	// Predict
	Do_Predict_X(KF);
	Do_Predict_P(KF);
	Do_Result(KF);

	//update
	KF->predictX_old = KF->predictX_new;
	KF->P_old = KF->P_new;

}

void Compute(KalmanFilter* KF, double _position){ //, float _Vin

	//Update Observer and Command
//	KF->U_In[0] = _Vin;
	KF->Y_measure[0] = _position;

	float ResultU_data[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	float ResultY_data[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	matrix ResultU = matrix_init(4, 4, &ResultU_data);
	matrix ResultY = matrix_init(4, 4, &ResultY_data);

//	KF->U = matrix_update(&KF->U, &KF->U_In);
	KF->Y = matrix_update(&KF->Y, &KF->Y_measure);

	//Kalmanfilter
	Run_KalmanFilter(KF);

	//Estimate State
	KF->estimate_State[0] = KF->result_X.data[0][0];
	KF->estimate_State[1] = KF->result_X.data[1][0];
	KF->estimate_State[2] = KF->result_X.data[2][0];
	KF->estimate_State[3] = KF->result_X.data[3][0];

//  return KF->estimate_State;
}
