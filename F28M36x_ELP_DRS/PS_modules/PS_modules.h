#ifndef PS_MODULES_H
#define PS_MODULES_H

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

#define PS_MODEL	FAP_ACDC

//#include "FBP_100kHz.h"
//#include "FBP_Parallel_100kHz.h"
//#include "FAC_ACDC_10kHz.h"
//#include "FAC_DCDC_20kHz.h"
//#include "FAC_Full_ACDC_10kHz.h"
//#include "FAC_Full_DCDC_20kHz.h"
#include "FAP_ACDC.h"
//#include "FAP_DCDC_20kHz.h"
//#include "Test_HRPWM.h"
//#include "Test_HRADC.h"

#include "F28M36x_ELP_DRS.h"
#endif
