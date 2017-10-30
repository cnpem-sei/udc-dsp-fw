/*
 * 		FILE: 		ELP_DCL.h
 * 		PROJECT: 	DRS v2.0
 * 		CREATION:	09/28/2015
 * 		MODIFIED:	10/01/2015
 *
 * 		AUTHOR: 	Gabriel O. Brunheira  (LNLS/ELP)
 *
 * 		DESCRIPTION:
 *		Definition of constants, structs, enumarates and prototypes statements
 *		for variables and functions	from source code of ELP Digital Control Library
 *
 *		TODO:
 */

#ifndef ELP_DCL_H
#define ELP_DCL_H

#include <math.h>
#include "DSP28x_Project.h"

#define	PID_DEFAULTS { 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, -1.0f }

#define SATURATE(var, max, min)			\
		if(var > max) var =  max;    	\
		if(var < min) var = min;

#define	USE_MODULE		0
#define	BYPASS_MODULE	1

#define NUM_MAX_MATRIX_SIZE 16

//typedef enum {Sine, Square, Triangle, FreqSweep} eSigGenType;

typedef volatile struct
{
	volatile float *pos;
	volatile float *neg;
	volatile float *error;
} tELP_Error;

typedef volatile struct
{
	Uint16 bypass;
	float dI_max;
	volatile float *in;
	volatile float *out;
} tELP_SRLim;

typedef volatile struct
{
	float K;
	float a;
	float in_old;
	volatile float *in;
	volatile float *out;
} tELP_LPF;

typedef volatile struct
{
	float Kp;
	float Ki;
	float FreqSample;
	float Umax;
	float Umin;
    float up;
    float ui;
	volatile float *in;
	volatile float *out;
} tELP_PI_dawu;

typedef volatile struct
{
	float Umax;
	float Umin;
	float b0;
	float b1;
	float b2;
	float a1;
	float a2;
	float w1;
	float w2;
	volatile float *in;
	volatile float *out;
} tELP_IIR_2P2Z;

typedef volatile struct
{
	float Umax;
	float Umin;
	float b0;
	float b1;
	float b2;
	float b3;
	float a1;
	float a2;
	float a3;
	float w1;
	float w2;
	float w3;
	volatile float *in;
	volatile float *out;
} tELP_IIR_3P3Z;

typedef volatile struct
{
	float vdc_nom;
	float vdc_min;
	volatile float *vdc_meas;
	volatile float *in;
	volatile float *out;
} tELP_DCLink_FF;

typedef volatile struct
{
    Uint16  num_rows;
    Uint16  num_cols;
    float   data[NUM_MAX_MATRIX_SIZE][NUM_MAX_MATRIX_SIZE];
} matrix_t;

typedef volatile struct
{
    matrix_t        matrix;
    volatile float  *in;
    volatile float  *out;
} tELP_MultiplyMatrix;

extern float b_coeff_NF_Vdc[3];
extern float a_coeff_NF_Vdc[2];
extern float b_coeff_LPF_Iin[3];
extern float a_coeff_LPF_Iin[2];
extern float b_coeff_IIR_Vdc_ctrl[3];
extern float a_coeff_IIR_Vdc_ctrl[2];

extern void Init_ELP_Error(tELP_Error *ptr_Err, volatile float *pos, volatile float *neg, volatile float *error);
extern void Reset_ELP_Error(tELP_Error *ptr_Err);
extern void Run_ELP_Error(tELP_Error *ptr_Err);

extern void Init_ELP_SRLim(tELP_SRLim *ptr_SR, float max_slewrate, float freqSample, volatile float *in, volatile float *out);
extern void Bypass_ELP_SRLim(tELP_SRLim *ptr_SR, Uint16 bypass);
extern void Reset_ELP_SRLim(tELP_SRLim *ptr_SR);
extern void Run_ELP_SRLim(tELP_SRLim *ptr_SR, Uint16 bypass);

extern void Init_ELP_LPF(tELP_LPF *ptr_LPF, float freqCut, float freqSample, volatile float *in, volatile float *out);
extern void Reset_ELP_LPF(tELP_LPF *ptr_LPF);
extern void Run_ELP_LPF(tELP_LPF *ptr_LPF);

extern void Init_ELP_PI_dawu(tELP_PI_dawu *ptr_PI, float Kp, float Ki, float freqSample, float uMax, float uMin, volatile float *in, volatile float *out);
extern void Reset_ELP_PI_dawu(tELP_PI_dawu *ptr_PI);
extern void Run_ELP_PI_dawu(tELP_PI_dawu *ptr_PI);

extern void Init_ELP_IIR_2P2Z(tELP_IIR_2P2Z *ptr_iir, float b0, float b1, float b2, float a1, float a2, float uMax, float uMin, volatile float *in, volatile float *out);
extern void Init_ELP_NF_IIR(tELP_IIR_2P2Z *ptr_iir, float alpha, float freqCut, float freqSample, float uMax, float uMin, volatile float *in, volatile float *out);
extern void Reset_ELP_IIR_2P2Z(tELP_IIR_2P2Z *ptr_iir);
extern void Run_ELP_IIR_2P2Z(tELP_IIR_2P2Z *ptr_iir);

extern void Init_ELP_IIR_3P3Z(tELP_IIR_3P3Z *ptr_iir, float b0, float b1, float b2, float b3, float a1, float a2, float a3, float uMax, float uMin, volatile float *in, volatile float *out);
extern void Reset_ELP_IIR_3P3Z(tELP_IIR_3P3Z *ptr_iir);
extern void Run_ELP_IIR_3P3Z(tELP_IIR_3P3Z *ptr_iir);

extern void Init_ELP_DCLink_FF(tELP_DCLink_FF *ptr_ff, float vdc_nom, float vdc_min, volatile float *vdc_meas, volatile float *in, volatile float *out);
extern void Reset_ELP_DCLink_FF(tELP_DCLink_FF *ptr_ff);
extern void Run_ELP_DCLink_FF(tELP_DCLink_FF *ptr_ff);

extern void Init_ELP_MultiplyMatrix(tELP_MultiplyMatrix *ptr_mtx, Uint16 num_rows, Uint16 num_cols, volatile float matrix[num_rows][num_cols], volatile float *in, volatile float *out);
extern void Reset_ELP_MultiplyMatrix(tELP_MultiplyMatrix *ptr_mtx);
extern void Run_ELP_MatrixMultiplication(tELP_MultiplyMatrix *ptr_mtx);

#endif	/* ELP_DCL_H */

