/*
 * 		FILE: 		SigGen.c
 * 		PROJECT: 	DRS v2.0
 * 		CREATION:	03/04/2016
 * 		MODIFIED:	03/04/2016
 *
 * 		AUTHOR: 	Gabriel O. Brunheira  (LNLS/ELP)
 *
 * 		DESCRIPTION:
 *		Source code for Signal Generator module.
 *
 *		This library created by LNLS/ELP group implements a Digital Signal Generator,
 *		which is able to calculate iteratively (using a function call) digital samples
 *		of a specified periodic waveform.
 *
 *		These modules are intended to be used within a Digital Power Framework.
 *
 *		TODO:	- Atualizar Aux no Update;
 *				- Implementar Run para Square, Triangle, FreqSweep
 */

#include "SigGen.h"

#define _USE_MATH_DEFINES
#define	PI	3.141592653589793

#pragma CODE_SECTION(Enable_ELP_SigGen, "ramfuncs");
#pragma CODE_SECTION(Disable_ELP_SigGen, "ramfuncs");
#pragma CODE_SECTION(Update_ELP_SigGen, "ramfuncs");
#pragma CODE_SECTION(Run_ELP_SigGen_Sine, "ramfuncs");
#pragma CODE_SECTION(Run_ELP_SigGen_Square, "ramfuncs");
#pragma CODE_SECTION(Run_ELP_SigGen_Triangle, "ramfuncs");
#pragma CODE_SECTION(Run_ELP_SigGen_FreqSweep, "ramfuncs");
#pragma CODE_SECTION(Run_ELP_SigGen_DampedSine, "ramfuncs");

/*
 * Prototype statements for functions found within this file
 */

Uint16 Init_ELP_SigGen(tELP_SigGen *ptr_sg, eSigGenType sigType, float phase_start, float phase_end, Uint16 ncycles, float freqSample,
					   volatile float *ptr_freqSig, volatile float *ptr_amp, volatile float *ptr_offset, volatile float *ptr_aux, volatile float *out);
void Enable_ELP_SigGen(tELP_SigGen *ptr_sg);
void Disable_ELP_SigGen(tELP_SigGen *ptr_sg);
void Reset_ELP_SigGen(tELP_SigGen *ptr_sg);
void Update_ELP_SigGen(tELP_SigGen *ptr_sg);
void Run_ELP_SigGen_Sine(tELP_SigGen *ptr_sg);
void Run_ELP_SigGen_Square(tELP_SigGen *ptr_sg);
void Run_ELP_SigGen_Triangle(tELP_SigGen *ptr_sg);
void Run_ELP_SigGen_FreqSweep(tELP_SigGen *ptr_sg);
void Run_ELP_SigGen_DampedSine(tELP_SigGen *ptr_sg);
void Run_ELP_SigGen_Trapezoidal(tELP_SigGen *ptr_sg);

tELP_SigGen SignalGenerator;

/*
 * Initialization of Signal Generator module
 */

Uint16 Init_ELP_SigGen(tELP_SigGen *ptr_sg, eSigGenType sigType, float phase_start, float phase_end, Uint16 ncycles, float freqSample,
					   volatile float *ptr_freqSig, volatile float *ptr_amp, volatile float *ptr_offset, volatile float *ptr_aux, volatile float *out)
{
	DINT;
	if(ptr_sg->Enable)
	{
		EINT;
		return 0;
	}
	else
	{
		ptr_sg->Type = sigType;
		ptr_sg->FreqSample = freqSample;
		ptr_sg->ptr_FreqSignal = ptr_freqSig;
		ptr_sg->ptr_Amp = ptr_amp;
		ptr_sg->ptr_Offset = ptr_offset;
		ptr_sg->ptr_Aux = ptr_aux;
		ptr_sg->n = 0.0;
		ptr_sg->w = 2.0 * PI * (*ptr_freqSig) / freqSample;
		ptr_sg->PhaseStart = PI * phase_start / 180.0;
		ptr_sg->PhaseEnd = PI * phase_end / 180.0;
		ptr_sg->nCycles = ncycles;
		ptr_sg->out = out;

		ptr_sg->nSamples = ncycles + (ptr_sg->PhaseEnd - ptr_sg->PhaseStart)/(2.0 * PI);
		if(ptr_sg->PhaseStart > ptr_sg->PhaseEnd)
		{
			ptr_sg->nSamples++;
		}
		ptr_sg->nSamples = ptr_sg->nSamples * freqSample / (*ptr_freqSig);

		switch(sigType)
		{
			case Sine:
				ptr_sg->Run_ELP_SigGen = Run_ELP_SigGen_Sine;
				break;

			case Square:
				ptr_sg->Run_ELP_SigGen = Run_ELP_SigGen_Square;
				break;

			case Triangle:
				ptr_sg->Run_ELP_SigGen = Run_ELP_SigGen_Triangle;
				break;

			case FreqSweep:
				ptr_sg->Run_ELP_SigGen = Run_ELP_SigGen_FreqSweep;
				break;

			case DampedSine:
				ptr_sg->Aux = -(1.0/(*ptr_aux)) / freqSample;
				ptr_sg->Run_ELP_SigGen = Run_ELP_SigGen_DampedSine;
				break;

			case Trapezoidal:
				ptr_sg->Run_ELP_SigGen = Run_ELP_SigGen_Trapezoidal;
				break;
		}

		EINT;
		return 1;
	}
}

void Enable_ELP_SigGen(tELP_SigGen *ptr_sg)
{
	Reset_ELP_SigGen(ptr_sg);
	if(ptr_sg->Type != DampedSine)
	{
		Update_ELP_SigGen(ptr_sg);
	}
	ptr_sg->Enable = 1;
}

void Disable_ELP_SigGen(tELP_SigGen *ptr_sg)
{
	ptr_sg->Enable = 0;
}

/*
 * Reset Signal Generator module
 */

void Reset_ELP_SigGen(tELP_SigGen *ptr_sg)
{
	ptr_sg->n = 0.0;
	*(ptr_sg->out) = 0.0;
}

/*
 * Run Signal Generator module
 */

void Run_ELP_SigGen_Sine(tELP_SigGen *ptr_sg)
{
	if(ptr_sg->Enable)
	{
		*(ptr_sg->out) = (*ptr_sg->ptr_Amp) * sin( ptr_sg->w * ptr_sg->n++ + ptr_sg->PhaseStart) + (*ptr_sg->ptr_Offset);

		if(ptr_sg->nSamples > 0)
		{
			if(ptr_sg->n >= ptr_sg->nSamples)
			{
				Disable_ELP_SigGen(ptr_sg);
			}

		}
		else if(ptr_sg->n >= ptr_sg->FreqSample)
		{
			/*
			 *  Compara��o � feita com fs, para que n seja incrementado at� completar 1 segundo.
			 *  Isso foi feito, pois ao comparar n com n_samp, distorcemos o sinal gerado, j� que
			 *  n_samp pode ser float, e n n�o.
			 *
			 *  Assim que o per�odo se completa e os par�metros do gerador s�o atualizados, de
			 *  forma a garantir uma transi��o suave entre a sen�ide gerada anteriormente e a nova
			 */
			Update_ELP_SigGen(ptr_sg);
			ptr_sg->n = 0.0;
		}
	}
}

void Run_ELP_SigGen_Square(tELP_SigGen *ptr_sg)
{

	*(ptr_sg->out) = 0.0;
}

void Run_ELP_SigGen_Triangle(tELP_SigGen *ptr_sg)
{
	*(ptr_sg->out) = 0.0;
}

void Run_ELP_SigGen_FreqSweep(tELP_SigGen *ptr_sg)
{
	*(ptr_sg->out) = 0.0;
}

void Run_ELP_SigGen_DampedSine(tELP_SigGen *ptr_sg)
{
	if(ptr_sg->Enable)
	{
		if(ptr_sg->n < ptr_sg->nSamples)
		{
			*(ptr_sg->out) = (*ptr_sg->ptr_Amp) * exp(ptr_sg->Aux * ptr_sg->n) * sin( ptr_sg->w * ptr_sg->n + ptr_sg->PhaseStart) + (*ptr_sg->ptr_Offset);
			ptr_sg->n++;
		}
		else
		{
			Disable_ELP_SigGen(ptr_sg);
		}
	}
}

void Run_ELP_SigGen_Trapezoidal(tELP_SigGen *ptr_sg)
{
	if(ptr_sg->Enable)
	{
		if(ptr_sg->n < ptr_sg->PhaseStart)
		{
			*(ptr_sg->out) = ptr_sg->n * ptr_sg->w + (*ptr_sg->ptr_Offset);
		}
		else if(ptr_sg->n < ptr_sg->PhaseEnd)
		{
			*(ptr_sg->out) = (*ptr_sg->ptr_Amp) + (*ptr_sg->ptr_Offset);
		}
		else if(ptr_sg->n < ptr_sg->nSamples)
		{
			*(ptr_sg->out) = ptr_sg->Aux * (ptr_sg->n - ptr_sg->PhaseEnd) + (*ptr_sg->ptr_Amp) + (*ptr_sg->ptr_Offset);
		}
		else
		{
			Disable_ELP_SigGen(ptr_sg);
		}

		ptr_sg->n++;
	}
}

void Update_ELP_SigGen(tELP_SigGen *ptr_sg)
{
	//ptr_sg->nSamples = ptr_sg->FreqSample / (*ptr_sg->ptr_FreqSignal);
	//ptr_sg->w = 2.0 * PI / ptr_sg->nSamples;
	ptr_sg->w = 2.0 * PI * (*ptr_sg->ptr_FreqSignal) / ptr_sg->FreqSample;
}
