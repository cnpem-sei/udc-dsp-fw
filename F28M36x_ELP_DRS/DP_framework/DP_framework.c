/*
 * 		FILE: 		DP_framework.c
 * 		PROJECT: 	DRS v2.0
 * 		CREATION:	09/28/2015
 * 		MODIFIED:	13/01/2015
 *
 * 		AUTHOR: 	Gabriel O. Brunheira  (LNLS/ELP)
 *
 * 		DESCRIPTION:
 *		Source code of Digital Power Framework, which implements a framework
 *		for the creation of digital controllers architectures, dedicated to
 *		switched-mode power supplies.
 *
 *		A DP Framework consists of:
 *
 *			- Set of modules for digital implementation of signal processing used
 *			  in control strategies (as control laws, filters, limiters, etc);
 *
 *			- Set of hardware drivers for peripherals, like A/D converters and
 *			  PWM modulators;
 *
 *			- Set of netlists (global variables) used for modules interconnections
 *
 *			- Time-Slice Manager, which controls rate of execution of modules for
 *			  multi-rate strategies
 *
 *		All modules and drivers are implemented in separete libraries and must
 *		be manually incorporated to the DP Framework according to the descriptions
 *		included in the comments from this code.
 *
 *		A main ISR is used to control the periodic and deterministic execution
 *		of the framework.
 *
 *		TODO:
 */

#include "DP_framework.h"

#pragma DATA_SECTION(DP_Framework,"SHARERAMS1");
volatile tDP_Framework DP_Framework;

#pragma DATA_SECTION(DP_Framework_MtoC,"SHARERAMS0");
volatile tDP_Framework DP_Framework_MtoC;

void InitDP_Framework(volatile tDP_Framework *ptr_DP, volatile float *ref);
Uint16 ConfigDP_Module(volatile tDP_Framework *ptr_frame, Uint16 id, eDPclass dp_class, volatile float *ptr_coeff);
Uint16 ConfigDP_Module_AllParam(volatile tDP_Framework *ptr_frame, Uint16 id, eDPclass dp_class, volatile float *ptr_coeff);

void InitDP_Framework(volatile tDP_Framework *ptr_DP, volatile float *ref)
{
	Uint16 i;

	ptr_DP->Ref = ref;
	*(ptr_DP->Ref) = 0.0;


	for(i = 0; i < N_MAX_NET_SIGNALS; i++)
	{
		ptr_DP->NetSignals[i] = 0.0;
	}

	for(i = 0; i < 2*N_MAX_PWM_MODULES; i++)
	{
		ptr_DP->DutySignals[i] = 0.0;
	}
}

Uint16 ConfigDP_Module(volatile tDP_Framework *ptr_frame, Uint16 id, eDPclass dp_class, volatile float *ptr_coeff)
{
	Uint16 result;

	switch(dp_class)
	{
		case ELP_SRLim:
			Init_ELP_SRLim(&ptr_frame->DPlibrary.ELP_SRLim[id],
							*(ptr_coeff),
							*(ptr_coeff+1),
							ptr_frame->DPlibrary.ELP_SRLim[id].in,
							ptr_frame->DPlibrary.ELP_SRLim[id].out);
			result = 1;
			break;

		case ELP_LPF:
			Init_ELP_LPF(&ptr_frame->DPlibrary.ELP_LPF[id],
						  *(ptr_coeff),
						  *(ptr_coeff+1),
						  ptr_frame->DPlibrary.ELP_LPF[id].in,
						  ptr_frame->DPlibrary.ELP_LPF[id].out);
			result = 1;
			break;

		case ELP_PI_dawu:
			Init_ELP_PI_dawu(&ptr_frame->DPlibrary.ELP_PI_dawu[id],
					          *(ptr_coeff),
							  *(ptr_coeff+1),
							  ptr_frame->DPlibrary.ELP_PI_dawu[id].FreqSample,
							  ptr_frame->DPlibrary.ELP_PI_dawu[id].Umax,
							  ptr_frame->DPlibrary.ELP_PI_dawu[id].Umin,
							  ptr_frame->DPlibrary.ELP_PI_dawu[id].in,
							  ptr_frame->DPlibrary.ELP_PI_dawu[id].out);
			result = 1;
			break;

		case ELP_IIR_2P2Z:
			Init_ELP_IIR_2P2Z(&ptr_frame->DPlibrary.ELP_IIR_2P2Z[id],
							   *(ptr_coeff),
							   *(ptr_coeff+1),
							   *(ptr_coeff+2),
							   *(ptr_coeff+3),
							   *(ptr_coeff+4),
							   ptr_frame->DPlibrary.ELP_IIR_2P2Z[id].Umax,
							   ptr_frame->DPlibrary.ELP_IIR_2P2Z[id].Umin,
							   ptr_frame->DPlibrary.ELP_IIR_2P2Z[id].in,
							   ptr_frame->DPlibrary.ELP_IIR_2P2Z[id].out);
			result = 1;
			break;

		case ELP_IIR_3P3Z:
			Init_ELP_IIR_3P3Z(&ptr_frame->DPlibrary.ELP_IIR_3P3Z[id],
							   *(ptr_coeff),
							   *(ptr_coeff+1),
							   *(ptr_coeff+2),
							   *(ptr_coeff+3),
							   *(ptr_coeff+4),
							   *(ptr_coeff+5),
							   *(ptr_coeff+6),
							   ptr_frame->DPlibrary.ELP_IIR_3P3Z[id].Umax,
							   ptr_frame->DPlibrary.ELP_IIR_3P3Z[id].Umin,
							   ptr_frame->DPlibrary.ELP_IIR_3P3Z[id].in,
							   ptr_frame->DPlibrary.ELP_IIR_3P3Z[id].out);
			result = 1;
			break;

		default:
			result = 0;
	}

	return result;
}

Uint16 ConfigDP_Module_AllParam(volatile tDP_Framework *ptr_frame, Uint16 id, eDPclass dp_class, volatile float *ptr_coeff)
{
	Uint16 result;

	switch(dp_class)
	{
		case ELP_SRLim:
			Init_ELP_SRLim(&ptr_frame->DPlibrary.ELP_SRLim[id],
							*(ptr_coeff),
							*(ptr_coeff+1),
							ptr_frame->DPlibrary.ELP_SRLim[id].in,
							ptr_frame->DPlibrary.ELP_SRLim[id].out);
			result = 1;
			break;

		case ELP_LPF:
			Init_ELP_LPF(&ptr_frame->DPlibrary.ELP_LPF[id],
						  *(ptr_coeff),
						  *(ptr_coeff+1),
						  ptr_frame->DPlibrary.ELP_LPF[id].in,
						  ptr_frame->DPlibrary.ELP_LPF[id].out);
			result = 1;
			break;

		case ELP_PI_dawu:
			Init_ELP_PI_dawu(&ptr_frame->DPlibrary.ELP_PI_dawu[id],
					          *(ptr_coeff),
							  *(ptr_coeff+1),
							  *(ptr_coeff+2),
							  *(ptr_coeff+3),
							  *(ptr_coeff+4),
							  ptr_frame->DPlibrary.ELP_PI_dawu[id].in,
							  ptr_frame->DPlibrary.ELP_PI_dawu[id].out);
			result = 1;
			break;

		case ELP_IIR_2P2Z:
			Init_ELP_IIR_2P2Z(&ptr_frame->DPlibrary.ELP_IIR_2P2Z[id],
							   *(ptr_coeff),
							   *(ptr_coeff+1),
							   *(ptr_coeff+2),
							   *(ptr_coeff+3),
							   *(ptr_coeff+4),
							   *(ptr_coeff+5),
							   *(ptr_coeff+6),
							   ptr_frame->DPlibrary.ELP_IIR_2P2Z[id].in,
							   ptr_frame->DPlibrary.ELP_IIR_2P2Z[id].out);
			result = 1;
			break;

		case ELP_IIR_3P3Z:
			Init_ELP_IIR_3P3Z(&ptr_frame->DPlibrary.ELP_IIR_3P3Z[id],
							   *(ptr_coeff),
							   *(ptr_coeff+1),
							   *(ptr_coeff+2),
							   *(ptr_coeff+3),
							   *(ptr_coeff+4),
							   *(ptr_coeff+5),
							   *(ptr_coeff+6),
							   *(ptr_coeff+7),
							   *(ptr_coeff+8),
							   ptr_frame->DPlibrary.ELP_IIR_3P3Z[id].in,
							   ptr_frame->DPlibrary.ELP_IIR_3P3Z[id].out);
			result = 1;
			break;

		default:
			result = 0;
	}

	return result;
}
