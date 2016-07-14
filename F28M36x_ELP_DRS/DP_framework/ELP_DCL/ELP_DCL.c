/*
 * 		FILE: 		ELP_DCL.c
 * 		PROJECT: 	DRS v2.0
 * 		CREATION:	09/28/2015
 * 		MODIFIED:	10/01/2015
 *
 * 		AUTHOR: 	Gabriel O. Brunheira  (LNLS/ELP)
 *
 * 		DESCRIPTION:
 *		Source code of ELP Digital Control Library.
 *
 *		This library created by LNLS/ELP group implements generic DSP modules normally
 *		used in digital control strategies for switched-mode power supplies, like PI
 *		controllers, filters and slew-rate limiters.
 *
 *		These modules are intended to be used within a Digital Power Framework.
 *
 *		TODO:
 */

#include "ELP_DCL.h"
#define _USE_MATH_DEFINES

#pragma CODE_SECTION(Run_ELP_SRLim, "ramfuncs");
#pragma CODE_SECTION(Run_ELP_LPF, "ramfuncs");
#pragma CODE_SECTION(Run_ELP_PI_dawu, "ramfuncs");
#pragma CODE_SECTION(Run_ELP_IIR_2P2Z, "ramfuncs");

float b_coeff_NF_Vdc[3] = {1.0, 0.0, 0.0};
float a_coeff_NF_Vdc[2] = {0.0, 0.0};
float b_coeff_LPF_Iin[3] = {1.0, 0.0, 0.0};
float a_coeff_LPF_Iin[2] = {0.0, 0.0};

float b_coeff_IIR_Vdc_ctrl[3] = {0.006955170312344, -0.013788820282703, 0.006861687945192};
float a_coeff_IIR_Vdc_ctrl[2] = {-0.789271966686379, -0.210728033313621};

/*
 * Prototype statements for functions found within this file
 */

void Init_ELP_Error(tELP_Error *ptr_Err, volatile float *pos, volatile float *neg, volatile float *error);
void Reset_ELP_Error(tELP_Error *ptr_Err);
void Run_ELP_Error(tELP_Error *ptr_Err);

void Init_ELP_SRLim(tELP_SRLim *ptr_SR, float max_slewrate, float freqSample, volatile float *in, volatile float *out);
void Bypass_ELP_SRLim(tELP_SRLim *ptr_SR, Uint16 bypass);
void Reset_ELP_SRLim(tELP_SRLim *ptr_SR);
void Run_ELP_SRLim(tELP_SRLim *ptr_SR, Uint16 bypass);

void Init_ELP_LPF(tELP_LPF *ptr_LPF, float freqCut, float freqSample, volatile float *in, volatile float *out);
void Reset_ELP_LPF(tELP_LPF *ptr_LPF);
void Run_ELP_LPF(tELP_LPF *ptr_LPF);

void Init_ELP_PI_dawu(tELP_PI_dawu *ptr_PI, float Kp, float Ki, float freqSample, float uMax, float uMin, volatile float *in, volatile float *out);
void Reset_ELP_PI_dawu(tELP_PI_dawu *ptr_PI);
void Run_ELP_PI_dawu(tELP_PI_dawu *ptr_PI);

void Init_ELP_IIR_2P2Z(tELP_IIR_2P2Z *ptr_iir, float b0, float b1, float b2, float a1, float a2, float uMax, float uMin, volatile float *in, volatile float *out);
void Init_ELP_NF_IIR(tELP_IIR_2P2Z *ptr_iir, float alpha, float freqCut, float freqSample, float uMax, float uMin, volatile float *in, volatile float *out);
void Reset_ELP_IIR_2P2Z(tELP_IIR_2P2Z *ptr_iir);
void Run_ELP_IIR_2P2Z(tELP_IIR_2P2Z *ptr_iir);

void Init_ELP_IIR_3P3Z(tELP_IIR_3P3Z *ptr_iir, float b0, float b1, float b2, float b3, float a1, float a2, float a3, float uMax, float uMin, volatile float *in, volatile float *out);
void Reset_ELP_IIR_3P3Z(tELP_IIR_3P3Z *ptr_iir);
void Run_ELP_IIR_3P3Z(tELP_IIR_3P3Z *ptr_iir);

/****************/

/*
 * Initialization of error signal entity
 */

void Init_ELP_Error(tELP_Error *ptr_Err, volatile float *pos, volatile float *neg, volatile float *error)
{
	ptr_Err->pos = pos;
	ptr_Err->neg = neg;
	ptr_Err->error = error;
	*(ptr_Err->error) = 0.0;
}

/*
 * Reset error signal
 */

void Reset_ELP_Error(tELP_Error *ptr_Err)
{
	*(ptr_Err->error) = 0.0;
}

/*
 * Calculate error signal
 *
 * 		- For improved speed, use the following macro:
 * 			 CALC_ERROR(ELP_Error)
 */

void Run_ELP_Error(tELP_Error *ptr_Err)
{
	*ptr_Err->error = (*ptr_Err->pos - *ptr_Err->neg);
}

/*
 * Initialization of slew-rate limiter
 *
 * 		Ref.: http://www.embeddedrelated.com/showarticle/646.php
 *
 * 		- max_slewrate 	[Units/sec]
 * 		- freqSample 	[Hz]
 */

void Init_ELP_SRLim(tELP_SRLim *ptr_SR, float max_slewrate, float freqSample, volatile float *in, volatile float *out)
{
	ptr_SR->bypass = USE_MODULE;
	ptr_SR->dI_max = max_slewrate / freqSample;
	ptr_SR->in = in;
	ptr_SR->out = out;
	*(ptr_SR->out) = 0.0;
}

/*
 * Bypass or not the specified slew-rate limiter
 */

void Bypass_ELP_SRLim(tELP_SRLim *ptr_SR, Uint16 bypass)
{
	ptr_SR->bypass = bypass;
}

/*
 * Reset slew-rate limiter
 */

void Reset_ELP_SRLim(tELP_SRLim *ptr_SR)
{
	*(ptr_SR->out) = 0.0;
}

/*
 * Run slew-rate limiter
 */

void Run_ELP_SRLim(tELP_SRLim *ptr_SR, Uint16 bypass)
{
	float delta;

	if(bypass)
	{
		*(ptr_SR->out) = *(ptr_SR->in);
	}
	else
	{
		delta = *(ptr_SR->in) - *(ptr_SR->out);
		SATURATE(delta, ptr_SR->dI_max, -ptr_SR->dI_max);
		*(ptr_SR->out) = *(ptr_SR->out) + delta;
	}
}

/*
 * Initialization of 1st-order digital low pass filter
 *
 * 		This is a Tustin discretization of the following
 * 		continuous 1st-order low-pass filter:
 * 					H(s) = 1 / (tau*s + 1)
 *
 * 		- freqCut		[Hz]
 * 		- freqSample	[Hz]
 */

void Init_ELP_LPF(tELP_LPF *ptr_LPF, float freqCut, float freqSample, volatile float *in, volatile float *out)
{
	float wT, K, a;

	wT = (2.0 * 3.141592653589793 * freqCut) / freqSample;
	K = wT / (2.0 + wT);
	a = (2 - wT)/(2 + wT);

	ptr_LPF->K = K;
	ptr_LPF->a = a;
	ptr_LPF->in_old = 0.0;
	ptr_LPF->in = in;
	ptr_LPF->out = out;
	*(ptr_LPF->out) = 0.0;
}

/*
 * Reset 1st-order digital low-pass filter
 */

void Reset_ELP_LPF(tELP_LPF *ptr_LPF)
{
	ptr_LPF->in_old = 0.0;
	*(ptr_LPF->out) = 0.0;
}

/*
 * Run 1st-order digital low-pass-filter
 */

void Run_ELP_LPF(tELP_LPF *ptr_LPF)
{
	float yacc;

	yacc = *(ptr_LPF->out) * ptr_LPF->a;
	yacc += ptr_LPF->K * (ptr_LPF->in_old + *(ptr_LPF->in));
	ptr_LPF->in_old = *(ptr_LPF->in);
	*(ptr_LPF->out) = yacc;
}

/*
 * Initialization of PI controller with dynamic anti-windup scheme
 *
 * 		Ref.: Buso, S.; Mattavelli, P.
 * 			  "Digital Control in Power Electronics", pag. 59
 *
 * 		- freqSample	[Hz]
 */

void Init_ELP_PI_dawu(tELP_PI_dawu *ptr_PI, float Kp, float Ki, float freqSample, float uMax, float uMin, volatile float *in, volatile float *out)
{
	ptr_PI->Kp = Kp;
	ptr_PI->Ki = Ki / freqSample;
	ptr_PI->FreqSample = freqSample;
	ptr_PI->Umax = uMax;
	ptr_PI->Umin = uMin;
	ptr_PI->up = 0.0;
	ptr_PI->ui = 0.0;
	ptr_PI->in = in;
	ptr_PI->out = out;
	*(ptr_PI->out) = 0.0;
}

/*
 * Reset PI controller
 */
void Reset_ELP_PI_dawu(tELP_PI_dawu *ptr_PI)
{
	ptr_PI->up = 0.0;
	ptr_PI->ui = 0.0;
	*(ptr_PI->out) = 0.0;
}

/*
 * Run PI controller
 */
void Run_ELP_PI_dawu(tELP_PI_dawu *ptr_PI)
{
    float dyn_max;
    float dyn_min;
    float temp;

    temp = *(ptr_PI->in) * ptr_PI->Kp;
    SATURATE(temp, ptr_PI->Umax, ptr_PI->Umin);
    ptr_PI->up = temp;

    dyn_max = (ptr_PI->Umax - temp);
    dyn_min = (ptr_PI->Umin - temp);

    temp = ptr_PI->ui + *(ptr_PI->in) * ptr_PI->Ki;
    SATURATE(temp, dyn_max, dyn_min);
    ptr_PI->ui = temp;

    *(ptr_PI->out) = ptr_PI->ui + ptr_PI->up;
}

/*
 * Initialization of 2nd-order digital IIR filter
 *
 * 		Implemented with Transposed Direct-Form II
 */

void Init_ELP_IIR_2P2Z(tELP_IIR_2P2Z *ptr_iir, float b0, float b1, float b2, float a1, float a2, float uMax, float uMin, volatile float *in, volatile float *out)
{
	ptr_iir->Umax = uMax;
	ptr_iir->Umin = uMin;
	ptr_iir->b0 = b0;
	ptr_iir->b1 = b1;
	ptr_iir->b2 = b2;
	ptr_iir->a1 = a1;
	ptr_iir->a2 = a2;
	ptr_iir->w1 = 0.0;
	ptr_iir->w2 = 0.0;
	ptr_iir->in = in;
	ptr_iir->out = out;
	*(ptr_iir->out) = 0.0;
}

/*
 * Initialization of 2nd-order digital IIR filter as
 * a notch-filter
 *
 *		Ref.: Welch, T. B.; Wright, C. H. G.; Morrow, M. G.
 *			  "Real-Time Digital Signal Processing from MATLAB
 *			   to C with the TMS320C6x DSPs", 2nd Edition
 *
 * 		- freqCut		[Hz]
 * 		- freqSample	[Hz]
 *
 */
void Init_ELP_NF_IIR(tELP_IIR_2P2Z *ptr_iir, float alpha, float freqCut, float freqSample, float uMax, float uMin, volatile float *in, volatile float *out)
{
	float beta = cos(2.0 * 3.141592653589793 * (freqCut/freqSample));

	SATURATE(alpha, 0.99999, 0.0);

	ptr_iir->Umax = uMax;
	ptr_iir->Umin = uMin;
	ptr_iir->b0 = (1.0 + alpha)/2.0;
	ptr_iir->b1 = -beta*(1.0 + alpha);
	ptr_iir->b2 = ptr_iir->b0;
	ptr_iir->a1 = ptr_iir->b1;
	ptr_iir->a2 = alpha;
	ptr_iir->w1 = 0.0;
	ptr_iir->w2 = 0.0;
	ptr_iir->in = in;
	ptr_iir->out = out;
	*(ptr_iir->out) = 0.0;
}

/*
 * Reset 2nd-order digital IIR filter
 */

void Reset_ELP_IIR_2P2Z(tELP_IIR_2P2Z *ptr_iir)
{
	ptr_iir->w1 = 0.0;
	ptr_iir->w2 = 0.0;
	*(ptr_iir->out) = 0.0;
}

/*
 * Run 2nd-order digital IIR filter
 */

void Run_ELP_IIR_2P2Z(tELP_IIR_2P2Z *ptr_iir)
{
	float w0, yacc;

	yacc = *(ptr_iir->in) * ptr_iir->b0;
	yacc += ptr_iir->w1;

	SATURATE(yacc, ptr_iir->Umax, ptr_iir->Umin);

	w0 = *(ptr_iir->in) * ptr_iir->b1;
	w0 += ptr_iir->w2;
	w0 -= yacc * ptr_iir->a1;
	ptr_iir->w1 = w0;

	w0 = *(ptr_iir->in) * ptr_iir->b2;
	w0 -= yacc * ptr_iir->a2;
	ptr_iir->w2 = w0;

	*(ptr_iir->out) = yacc;
}

/*
 * Initialization of 3rd-order digital IIR filter
 *
 * 		Implemented with Transposed Direct-Form II
 */

void Init_ELP_IIR_3P3Z(tELP_IIR_3P3Z *ptr_iir, float b0, float b1, float b2, float b3, float a1, float a2, float a3, float uMax, float uMin, volatile float *in, volatile float *out)
{
	ptr_iir->Umax = uMax;
	ptr_iir->Umin = uMin;
	ptr_iir->b0 = b0;
	ptr_iir->b1 = b1;
	ptr_iir->b2 = b2;
	ptr_iir->b3 = b3;
	ptr_iir->a1 = a1;
	ptr_iir->a2 = a2;
	ptr_iir->a3 = a3;
	ptr_iir->w1 = 0.0;
	ptr_iir->w2 = 0.0;
	ptr_iir->w3 = 0.0;
	ptr_iir->in = in;
	ptr_iir->out = out;
	*(ptr_iir->out) = 0.0;
}

/*
 * Reset 3rd-order digital IIR filter
 */

void Reset_ELP_IIR_3P3Z(tELP_IIR_3P3Z *ptr_iir)
{
	ptr_iir->w1 = 0.0;
	ptr_iir->w2 = 0.0;
	ptr_iir->w3 = 0.0;
	*(ptr_iir->out) = 0.0;
}

/*
 * Run 3rd-order digital IIR filter
 */

void Run_ELP_IIR_3P3Z(tELP_IIR_3P3Z *ptr_iir)
{
	float w0, yacc;

	yacc = *(ptr_iir->in) * ptr_iir->b0;
	yacc += ptr_iir->w1;

	SATURATE(yacc, ptr_iir->Umax, ptr_iir->Umin);

	w0 = *(ptr_iir->in) * ptr_iir->b1;
	w0 += ptr_iir->w2;
	w0 -= yacc * ptr_iir->a1;
	ptr_iir->w1 = w0;

	w0 = *(ptr_iir->in) * ptr_iir->b2;
	w0 += ptr_iir->w3;
	w0 -= yacc * ptr_iir->a2;
	ptr_iir->w2 = w0;

	w0 = *(ptr_iir->in) * ptr_iir->b3;
	w0 -= yacc * ptr_iir->a3;
	ptr_iir->w3 = w0;

	*(ptr_iir->out) = yacc;
}
