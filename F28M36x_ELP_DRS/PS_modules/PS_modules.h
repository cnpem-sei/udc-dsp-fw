#ifndef PS_MODULES_H
#define PS_MODULES_H

typedef enum
{
	SlowRef,
	FastRef,
	WfmRef,
	SigGen
} ePSOpMode;

typedef enum
{
	CLOSED_LOOP,
	OPEN_LOOP
} ePSLoopState;

typedef enum
{
	FBP_100kHz,
	FBP_Parallel_100kHz,
	FAC_ACDC_10kHz,
	FAC_DCDC_20kHz,
	FAC_Full_ACDC_10kHz,
	FAC_Full_DCDC_20kHz,
	FAP_ACDC,
	FAP_DCDC_20kHz,
	TEST_HRPWM,
	TEST_HRADC
} ePSModel;

#define PS_MODEL	FBP_100kHz

#include "F28M36x_ELP_DRS.h"

#endif
