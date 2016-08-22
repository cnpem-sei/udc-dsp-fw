/*
 * 		FILE: 		DP_framework.h
 * 		PROJECT: 	DRS v2.0
 * 		CREATION:	09/28/2015
 * 		MODIFIED:	10/01/2015
 *
 * 		AUTHOR: 	Gabriel O. Brunheira  (LNLS/ELP)
 *
 * 		DESCRIPTION:
 *		Definition of constants, structs, enumarates and prototypes statements
 *		for variables and functions	from source code of Digital Power Framework.
 *
 *		TODO:
 */

#ifndef DP_FRAMEWORK_H
#define DP_FRAMEWORK_H

/*
 * Enumerates classes of DP modules
 *
 * This list must contain a entry naming each module class included in the module libraries
 * used by the DP Framework.
 */

//typedef enum {Sine, Square, Triangle, FreqSweep} eSigGenType;

typedef enum
{
	ELP_Error,
	ELP_SRLim,
	ELP_LPF,
	ELP_PI_dawu,
	ELP_IIR_2P2Z,
	ELP_IIR_3P3Z,
	DCL_PID,
	DCL_PI,
	DCL_DF13,
	DCL_DF22,
	DCL_23
} eDPclass;

#include "F28M36x_ELP_DRS.h"

/* Library-wide limits */

#define N_MAX_NET_SIGNALS	32
#define N_MAX_TS_COUNTERS	4

#define N_MAX_ELP_ERROR		4
#define N_MAX_ELP_SRLim		3
#define N_MAX_ELP_PI_DAWU	6
#define	N_MAX_ELP_IIR_2P2Z	6
#define N_MAX_ELP_IIR_3P3Z  3
#define N_MAX_ELP_LPF		4
#define	N_MAX_ELP_DCLink_FF 2
#define N_MAX_TI_PID		3
#define N_MAX_TI_PI			3
#define N_MAX_TI_DF13		3
#define N_MAX_TI_DF22		3
#define N_MAX_TI_DF23		3

/* Collection of Digital Control Library (DCL) modules used by Framework */

typedef volatile struct
{
	tELP_Error		ELP_Error[N_MAX_ELP_ERROR];
	tELP_SRLim		ELP_SRLim[N_MAX_ELP_SRLim];
	tELP_LPF		ELP_LPF[N_MAX_ELP_LPF];
	tELP_PI_dawu 	ELP_PI_dawu[N_MAX_ELP_PI_DAWU];
	tELP_IIR_2P2Z	ELP_IIR_2P2Z[N_MAX_ELP_IIR_2P2Z];
	tELP_IIR_3P3Z	ELP_IIR_3P3Z[N_MAX_ELP_IIR_3P3Z];
	tELP_DCLink_FF	ELP_DCLink_FF[N_MAX_ELP_DCLink_FF];
	PID				TI_PID[N_MAX_TI_PID];
	PI				TI_PI[N_MAX_TI_PI];
	DF13			TI_DF13[N_MAX_TI_DF13];
	DF22			TI_DF22[N_MAX_TI_DF22];
	DF23			TI_DF23[N_MAX_TI_DF23];
} tDP_Library;

typedef struct
{
	float Iout1;
	float Iout2;
	float Vin;
	float Vout;
	Uint16 TempChv1;
	Uint16 TempChv2;
	Uint16 TempL1;
	Uint16 TempL2;
	Uint16 DvrVolt;
	Uint16 DvrCurr;
	Uint16 RH;
	Uint16 ErrAl;
} Q1Module_t;

typedef struct
{
	float Iout;
	float Vin;
	float Vout;
	Uint16 TempChv1;
	Uint16 TempChv2;
	Uint16 TempL1;
	Uint16 TempL2;
	Uint16 DvrVolt;
	Uint16 DvrCurr;
	Uint16 RH;
} Q4Module_t;

/*
 * DP Framework entity. This struct groups information regardind a
 * particular DP Framework implementation, including:
 *
 * 		- Pointer to reference signal
 * 		- Set of netlists for modules interconnection
 * 		- Set of DCL modules
 * 		- Time-Slice Manager module
 */

typedef volatile struct
{
	volatile float	*Ref;
	float 			NetSignals[N_MAX_NET_SIGNALS];
	float			DutySignals[2*N_MAX_PWM_MODULES];
	tDP_Library		DPlibrary;
} tDP_Framework;


extern volatile tDP_Framework DP_Framework;
extern volatile tDP_Framework DP_Framework_MtoC;

extern void InitDP_Framework(volatile tDP_Framework *ptr_DP, volatile float *ref);
extern Uint16 ConfigDP_Module(volatile tDP_Framework *ptr_frame, Uint16 id, eDPclass dp_class, volatile float *ptr_coeff);
extern Uint16 ConfigDP_Module_AllParam(volatile tDP_Framework *ptr_frame, Uint16 id, eDPclass dp_class, volatile float *ptr_coeff);

#endif	/* DP_FRAMEWORK_H */
