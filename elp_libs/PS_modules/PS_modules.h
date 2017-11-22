#ifndef PS_MODULES_H
#define PS_MODULES_H

#define CHECK_SOFTINTERLOCK(itlk)   !(IPC_CtoM_Msg.PSModule.SoftInterlocks & itlk)
#define CHECK_INTERLOCK(itlk)       !(IPC_CtoM_Msg.PSModule.HardInterlocks & itlk)
#define CHECK_INTERLOCKS            !(IPC_CtoM_Msg.PSModule.HardInterlocks)

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
	TEST_HRADC,
	JIGA_HRADC,
	FAP_DCDC_15kHz_225A,
	FBPx4_100kHz,
	FAP_6U_DCDC_20kHz,
	JIGA_BASTIDOR
} ePSModel;

#include "F28M36x_ELP_DRS.h"

#endif
