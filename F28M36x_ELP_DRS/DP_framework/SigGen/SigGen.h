/*
 * 		FILE: 		SigGen.h
 * 		PROJECT: 	DRS v2.0
 * 		CREATION:	03/04/2016
 * 		MODIFIED:	03/04/2016
 *
 * 		AUTHOR: 	Gabriel O. Brunheira  (LNLS/ELP)
 *
 * 		DESCRIPTION:
 *		Definition of constants, structs, enumarates and prototypes statements
 *		for variables and functions	from source code of Signal Generator module
 *
 *		TODO:
 *
 */

#ifndef SIGGEN_H
#define SIGGEN_H

typedef enum {Sine, Square, Triangle, FreqSweep, DampedSine} eSigGenType;

#include <math.h>
#include "DSP28x_Project.h"

typedef volatile struct tELP_SigGen tELP_SigGen;

struct tELP_SigGen {
	Uint16 			Enable;
	eSigGenType		Type;
	Uint16			nCycles;
	float			PhaseStart;
	float			PhaseEnd;
	float 			FreqSample;
	float			Aux;
	float 			w;
	float 			n;
	float 			nSamples;
	volatile float 	*ptr_FreqSignal;
	volatile float 	*ptr_Amp;
	volatile float 	*ptr_Offset;
	volatile float	*ptr_Aux;
	volatile float 	*out;
	void			(*Run_ELP_SigGen)(tELP_SigGen *ptr_sg);
};

extern tELP_SigGen SignalGenerator;

Uint16 Init_ELP_SigGen(tELP_SigGen *ptr_sg, eSigGenType sigType, float phase_start, float phase_end, Uint16 ncycles, float freqSample,
					  volatile float *ptr_freqSig, volatile float *ptr_amp, volatile float *ptr_offset, volatile float *ptr_aux, volatile float *out);
extern void Enable_ELP_SigGen(tELP_SigGen *ptr_sg);
extern void Disable_ELP_SigGen(tELP_SigGen *ptr_sg);
extern void Reset_ELP_SigGen(tELP_SigGen *ptr_sg);
extern void Update_ELP_SigGen(tELP_SigGen *ptr_sg);
extern void Run_ELP_SigGen_Sine(tELP_SigGen *ptr_sg);
extern void Run_ELP_SigGen_Square(tELP_SigGen *ptr_sg);
extern void Run_ELP_SigGen_Triangle(tELP_SigGen *ptr_sg);
extern void Run_ELP_SigGen_FreqSweep(tELP_SigGen *ptr_sg);
extern void Run_ELP_SigGen_DampedSine(tELP_SigGen *ptr_sg);

#endif
