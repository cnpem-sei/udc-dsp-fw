/*
 * 		FILE: 		HRADC_Boards.c
 * 		PROJECT: 	DRS v2.0
 * 		CREATION:	07/23/2015
 * 		MODIFIED:	07/23/2015
 *
 * 		AUTHOR: 	Gabriel O. B.  (LNLS/ELP)
 *
 * 		DESCRIPTION:
 *		Source code for access and control of HRADC boards
 *
 *		TODO: Finish CheckStatus_HRADC()
 *		TODO: Create functions to access flash memory (UFM)
 */


#include "HRADC_Boards.h"

/**********************************************************************************************/
//
// 	Prototype statements for functions found within this file
//
void Init_HRADC_Info(volatile HRADC_struct *hradcPtr, Uint16 ID, Uint16 buffer_size, volatile Uint32 *buffer, float transducer_gain);
void Init_HRADC_Info2(volatile HRADC_struct *hradcPtr, Uint16 ID, Uint16 buffer_size, volatile Uint32 *buffer, float transducer_gain, float rburden);
void Config_HRADC_board(volatile HRADC_struct *hradcPtr, enum_AN_INPUT AnalogInput, Uint16 enHeater, Uint16 enRails);
void SendCommand_HRADC(volatile HRADC_struct *hradcPtr, Uint16 command);
void Config_HRADC_SoC(float freq);
void Init_HRADCs_Info(void);
void enable_HRADC_Sampling(volatile HRADCs_struct HRADCs_Info);
void disable_HRADC_Sampling(volatile HRADCs_struct HRADCs_Info);
Uint16 CheckStatus_HRADC(volatile HRADC_struct *hradcPtr);

/**********************************************************************************************/
//
// 	Global variables instantiation
// 	Structs gather all information regarding used HRADC boards
//	Place them in shared memory RAMS1
//
#pragma DATA_SECTION(HRADCs_Info, "SHARERAMS2")
#pragma DATA_SECTION(HRADC0_board, "SHARERAMS2")
#pragma DATA_SECTION(HRADC1_board, "SHARERAMS2")
#pragma DATA_SECTION(HRADC2_board, "SHARERAMS2")
#pragma DATA_SECTION(HRADC3_board, "SHARERAMS2")

volatile HRADCs_struct HRADCs_Info;
volatile HRADC_struct  HRADC0_board;
volatile HRADC_struct  HRADC1_board;
volatile HRADC_struct  HRADC2_board;
volatile HRADC_struct  HRADC3_board;

volatile Uint32 counterErrorSendCommand;
volatile float AverageFilter;
/**********************************************************************************************/
//
//	Initialize information of selected HRADC board
//
void Init_HRADC_Info(volatile HRADC_struct *hradcPtr, Uint16 ID, Uint16 buffer_size, volatile Uint32 *buffer, float transducer_gain)
{
	hradcPtr->ID = ID;
	hradcPtr->index_SamplesBuffer = 0;
	hradcPtr->size_SamplesBuffer = buffer_size;
	hradcPtr->SamplesBuffer = buffer;

	hradcPtr->gain = &hradcPtr->CalibrationInfo.gain_Vin_bipolar;
	hradcPtr->offset = &hradcPtr->CalibrationInfo.offset_Vin_bipolar;

	hradcPtr->CalibrationInfo.gain_Vin_bipolar = 		transducer_gain * HRADC_VIN_BI_P_GAIN;
	hradcPtr->CalibrationInfo.offset_Vin_bipolar = 		HRADC_BI_OFFSET;

	hradcPtr->CalibrationInfo.gain_Vin_unipolar_p = 	0.0;
	hradcPtr->CalibrationInfo.offset_Vin_unipolar_p =	0.0;

	hradcPtr->CalibrationInfo.gain_Vin_unipolar_n =		0.0;
	hradcPtr->CalibrationInfo.offset_Vin_unipolar_n =	0.0;

	hradcPtr->CalibrationInfo.gain_Iin_bipolar = 		transducer_gain * HRADC_IIN_BI_P_GAIN;
	hradcPtr->CalibrationInfo.offset_Iin_bipolar =		HRADC_BI_OFFSET;

	hradcPtr->CalibrationInfo.gain_Iin_unipolar_p = 	0.0;
	hradcPtr->CalibrationInfo.offset_Iin_unipolar_p =	0.0;

	hradcPtr->CalibrationInfo.gain_Iin_unipolar_n = 	0.0;
	hradcPtr->CalibrationInfo.offset_Iin_unipolar_n =	0.0;

	while(hradcPtr->SamplesBuffer < &buffer[buffer_size])
	{
		*(hradcPtr->SamplesBuffer++) = 0.0;
	}

	hradcPtr->SamplesBuffer = buffer;
}

void Init_HRADC_Info2(volatile HRADC_struct *hradcPtr, Uint16 ID, Uint16 buffer_size, volatile Uint32 *buffer, float transducer_gain, float rburden)
{
	hradcPtr->ID = ID;
	hradcPtr->index_SamplesBuffer = 0;
	hradcPtr->size_SamplesBuffer = buffer_size;
	hradcPtr->SamplesBuffer = buffer;

	hradcPtr->gain = &hradcPtr->CalibrationInfo.gain_Vin_bipolar;
	hradcPtr->offset = &hradcPtr->CalibrationInfo.offset_Vin_bipolar;

	hradcPtr->CalibrationInfo.gain_Vin_bipolar = 		transducer_gain * HRADC_VIN_BI_P_GAIN;
	hradcPtr->CalibrationInfo.offset_Vin_bipolar = 		HRADC_BI_OFFSET;

	hradcPtr->CalibrationInfo.gain_Vin_unipolar_p = 	0.0;
	hradcPtr->CalibrationInfo.offset_Vin_unipolar_p =	0.0;

	hradcPtr->CalibrationInfo.gain_Vin_unipolar_n =		0.0;
	hradcPtr->CalibrationInfo.offset_Vin_unipolar_n =	0.0;

	hradcPtr->CalibrationInfo.gain_Iin_bipolar = 		transducer_gain * (1.0/(rburden * 131072.0));
	hradcPtr->CalibrationInfo.offset_Iin_bipolar =		HRADC_BI_OFFSET;

	hradcPtr->CalibrationInfo.gain_Iin_unipolar_p = 	0.0;
	hradcPtr->CalibrationInfo.offset_Iin_unipolar_p =	0.0;

	hradcPtr->CalibrationInfo.gain_Iin_unipolar_n = 	0.0;
	hradcPtr->CalibrationInfo.offset_Iin_unipolar_n =	0.0;

	while(hradcPtr->SamplesBuffer < &buffer[buffer_size])
	{
		*(hradcPtr->SamplesBuffer++) = 0.0;
	}

	hradcPtr->SamplesBuffer = buffer;
}

/**********************************************************************************************/
//
//	Configure HRADC board with selected parameters and check status
//
void Config_HRADC_board(volatile HRADC_struct *hradcPtr, enum_AN_INPUT AnalogInput, Uint16 enHeater, Uint16 enRails)
{
	Uint16 command;

	switch(AnalogInput)
	{
		case Vin_bipolar:
		{
			hradcPtr->gain = &hradcPtr->CalibrationInfo.gain_Vin_bipolar;
			hradcPtr->offset = &hradcPtr->CalibrationInfo.offset_Vin_bipolar;
			break;
		}
		case Vin_unipolar_p:
		{
			hradcPtr->gain = &hradcPtr->CalibrationInfo.gain_Vin_unipolar_p;
			hradcPtr->offset = &hradcPtr->CalibrationInfo.offset_Vin_unipolar_p;
			break;
		}
		case Vin_unipolar_n:
		{
			hradcPtr->gain = &hradcPtr->CalibrationInfo.gain_Vin_unipolar_n;
			hradcPtr->offset = &hradcPtr->CalibrationInfo.offset_Vin_unipolar_n;
			break;
		}
		case Iin_bipolar:
		{
			hradcPtr->gain = &hradcPtr->CalibrationInfo.gain_Iin_bipolar;
			hradcPtr->offset = &hradcPtr->CalibrationInfo.offset_Iin_bipolar;
			break;
		}
		case Iin_unipolar_p:
		{
			hradcPtr->gain = &hradcPtr->CalibrationInfo.gain_Iin_unipolar_p;
			hradcPtr->offset = &hradcPtr->CalibrationInfo.offset_Iin_unipolar_p;
			break;
		}
		case Iin_unipolar_n:
		{
			hradcPtr->gain = &hradcPtr->CalibrationInfo.gain_Iin_unipolar_n;
			hradcPtr->offset = &hradcPtr->CalibrationInfo.offset_Iin_unipolar_n;
			break;
		}
		default:
		{
			hradcPtr->gain = &hradcPtr->CalibrationInfo.gain_Vin_bipolar;
			hradcPtr->offset = &hradcPtr->CalibrationInfo.offset_Vin_bipolar;
			break;
		}
	}

	// Store new configuration parameters
	hradcPtr->AnalogInput = AnalogInput;
	hradcPtr->enable_Heater = enHeater;
	hradcPtr->enable_RailsMonitor = enRails;

	// Create command according to the HRADC Command Protocol (HCP)
	command = ((enRails << 8) | (enHeater << 7) | (AnalogInput << 3)) & 0x01F8;

	SendCommand_HRADC(hradcPtr, command);
	//CheckStatus_HRADC(hradcPtr);

	while(CheckStatus_HRADC(hradcPtr))
	{
		SendCommand_HRADC(hradcPtr, command);
	}
}


/**********************************************************************************************/
//
//	Send command to selected HRADC board
//
void SendCommand_HRADC(volatile HRADC_struct *hradcPtr, Uint16 command)
{
	Uint32 auxH;
	Uint32 auxL;

	// Stop DMA controller to prevent DMA access to McBSP
	// during configuration
	stop_DMA();

	// Set appropriate Chip-Select and CONFIG signals
	GpioDataRegs.GPECLEAR.all = 3;
	GpioDataRegs.GPESET.all = hradcPtr->ID;
	GpioDataRegs.GPESET.bit.GPIO131 = 1; 	// Aciona CONFIG

	// Transmit command via SPI
	McbspaRegs.DXR2.all = 0x0000;
	McbspaRegs.DXR1.all = command;

	// Wait for end of transmission/reception
	while(!McbspaRegs.SPCR1.bit.RRDY){}

	// Store previous HRADC configuration and status
	auxH = (Uint32) McbspaRegs.DRR2.all << 16;
	auxL = (Uint32) McbspaRegs.DRR1.all;
	hradcPtr->Status = auxH + auxL;

	// Clear DMA events triggers flags
	EALLOW;
	DmaRegs.CH1.CONTROL.bit.PERINTCLR = 1;
	DmaRegs.CH2.CONTROL.bit.PERINTCLR = 1;
	EDIS;

	// Reset Chip-Select and CONFIG signals
	GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;
	GpioDataRegs.GPECLEAR.all = 3;
}

/**********************************************************************************************/
//
//	Check status flags of selected HRADC board and take appropriate action
//
Uint16 CheckStatus_HRADC(volatile HRADC_struct *hradcPtr)
{
	Uint16 statusAux;

	SendCommand_HRADC(hradcPtr,CHECK_STATUS);
	statusAux = (hradcPtr->Status & 0x00000078) >> 3;

	if(statusAux != hradcPtr->AnalogInput)
	{
		counterErrorSendCommand++;
		return 1;
	}
	else
	{
		return 0;
	}
}

/**********************************************************************************************/
//
//	Configure Start-of-Conversion generator
//
void Config_HRADC_SoC(float freq)
{
	HRADCs_Info.freq_Sampling = freq;

	/*
	 * Configure ePWM10 module as Start-of-Conversion generator
	 *
	 * 		ePWM10 is synchronized with CTR = ZERO event from ePWM1
	 */

	InitPWMModule(&EPwm10Regs, freq, 0, SlavePWM, 0, NO_COMPLEMENTARY, 0);
	//
	// Modify standard initialization:
	//	1. Disable One-Shot Trip
	//  2. Configure as low-active signal
	//  3. CMPA = 15 sysclk -> 100 ns CNVST pulse
	//  4. Disable simultaneous writing to modulation registers
	//  5. Disable interrupts
	EALLOW;
	GpioCtrlRegs.GPEMUX1.bit.GPIO130 = 1;
	EPwm10Regs.AQCTLA.bit.CAU = AQ_SET;
	EPwm10Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;
	EPwm10Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
	EPwm10Regs.CMPA.half.CMPA = 15;
	EPwm10Regs.EPWMXLINK.bit.TBPRDLINK = 0;
	EPwm10Regs.EPWMXLINK.bit.CMPALINK = 0;
	EPwm10Regs.EPWMXLINK.bit.CMPBLINK = 0;
	EPwm10Regs.ETSEL.bit.INTEN = 0;
	EPwm10Regs.TZSEL.bit.OSHT1 = 0;
    EPwm10Regs.TZCLR.bit.OST = 1;
	EDIS;
}
