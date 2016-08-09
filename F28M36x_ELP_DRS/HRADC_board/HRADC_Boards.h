/*
 * 		FILE: 		HRADC_Boards.h
 * 		PROJECT: 	DRS v2.0
 * 		CREATION:	07/23/2015
 * 		MODIFIED:	07/23/2015
 *
 * 		AUTHOR: 	Gabriel O. B.  (LNLS/ELP)
 *
 * 		DESCRIPTION:
 *		Source code for access and control of HRADC boards
 *
 *		TODO: Generalize HRADC_FREQ_SAMP calculation for interleaving
 */

#ifndef HRADC_BOARDS_H
#define HRADC_BOARDS_H

#define N_MAX_HRADC			4
#define CHECK_STATUS		0x01	// HCP Command for check HRADC board status
#define HEATER_DISABLE 		0
#define HEATER_ENABLE 		1
#define RAILS_DISABLE 		0
#define RAILS_ENABLE 		1

#define HRADC_BI_OFFSET		131072.0

#define UFM_OPCODE_WREN			0x06
#define UFM_OPCODE_WRDI			0x04
#define UFM_OPCODE_RDSR			0x05
#define UFM_OPCODE_WRSR			0x01
#define UFM_OPCODE_READ			0x03
#define UFM_OPCODE_WRITE		0x02
#define UFM_OPCODE_SECTOR_ERASE	0x20
#define UFM_OPCODE_UFM_ERASE	0x60

#define HRADC_CONFIG			GpioDataRegs.GPESET.bit.GPIO131 = 1;
#define HRADC_nCONFIG			GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

#define HRADC_CS_SET(id) 		GpioDataRegs.GPESET.all = id;
#define HRADC_CS_CLEAR			GpioDataRegs.GPECLEAR.all = 3;
#define HRADC_VIN_BI_P_GAIN		(20.0/262144.0)

#define HRADC_CS_RESET(id)		HRADC_CS_CLEAR; \
								HRADC_CS_SET((~id) & 3); \
								DELAY_US(1); \
								HRADC_CS_CLEAR; \
								HRADC_CS_SET(id);
/**********************************************************************************************/
//
// 	Enumerate definition for configurable analog inputs
//
typedef enum {
		Vin_bipolar,
		Vin_unipolar_p,
		Vin_unipolar_n,
		Iin_bipolar,
		Iin_unipolar_p,
		Iin_unipolar_n,
		Vref_bipolar_p,
		Vref_bipolar_n,
		GND,
		Vref_unipolar_p,
		Vref_unipolar_n,
		GND_unipolar,
		Temp,
		Reserved0,
		Reserved1,
		Reserved2
} enum_AN_INPUT;

//#include "DSP28x_Project.h"
//#include "../C28 Project/config.h"
//#include "../PWM_modules/PWM_modules.h"
#include "F28M36x_ELP_DRS.h"

/**********************************************************************************************/
//
// 	HRADC Board Calibration Database
//
typedef volatile struct
{
	unsigned char	date_hw[8];			// Date of last hardware modification [MMDDYYYY]
	unsigned char	name_hw[8];	  		// Responsible for last hardware modification
	unsigned char 	date_calib[8]; 		// Date of last calibration [MMDDYYYY]
	unsigned char	name_calib[8]; 		// Responsible for last calibration
	float 			temp_calib;	  		// Ambiente temperature during last calibration

	float	gain_Vin_bipolar;			// Calibration gain		Vin bipolar
	float	offset_Vin_bipolar;			// Calibration offset	Vin bipolar
	float	gain_Vin_unipolar_p;	 	// Calibration gain		Vin unipolar (+)
	float	offset_Vin_unipolar_p;	  	// Calibration offset	Vin unipolar (+)
	float	gain_Vin_unipolar_n;		// Calibration gain		Vin unipolar
	float	offset_Vin_unipolar_n;		// Calibration offset	Vin unipolar
	float	gain_Iin_bipolar;			// Calibration gain		Iin bipolar
	float	offset_Iin_bipolar;			// Calibration offset	Iin bipolar
	float	gain_Iin_unipolar_p;		// Calibration gain		Iin unipolar (+)
	float	offset_Iin_unipolar_p;		// Calibration offset	Iin unipolar (+)
	float	gain_Iin_unipolar_n;		// Calibration gain		Iin unipolar (-)
	float	offset_Iin_unipolar_n;		// Calibration offset	Iin unipolar (-)
	float	Vref_bipolar_p;				// + Voltage reference 	Bipolar
	float	Vref_bipolar_n;				// - Voltage reference 	Bipolar
	float	GND_bipolar;				// GND					Bipolar
	float	Vref_unipolar_p;			// + Voltage reference 	Unipolar
	float	Vref_unipolar_n;			// - Voltage reference 	Unipolar
	float	GND_unipolar;				// GND					Unipolar

} HRADC_CalibInfo;


/**********************************************************************************************/
//
// 	HRADC Board Database
//
typedef volatile struct
{

	Uint16 				ID;						// Backplane position
	Uint16 				SerialNumber;			// Unique identification
	Uint32				HW_version;				// Hardware information: Rburden / Fcut / Filter order / Heater_TempRef
	enum_AN_INPUT		AnalogInput;			// Analog input
	Uint16 				enable_Heater;			// Enable temperature controller
	Uint16 				enable_RailsMonitor;	// Enable rails monitor
	Uint32				Status;					// Configuration/Status Register
	Uint16				size_SamplesBuffer;		// Size of samples buffer
	Uint16				index_SamplesBuffer;	// Current buffer pointer position
	volatile Uint32		*SamplesBuffer;			// Pointer to samples buffer
	volatile float		*gain;					// Pointer to gain value of current analog input
	volatile float		*offset;				// Pointer to offset value of current analog input
	HRADC_CalibInfo 	CalibrationInfo;		// Calibration database

} HRADC_struct;

typedef volatile struct
{
	float					freq_Sampling;
	Uint16 					enable_Sampling;
	Uint16 					n_HRADC_boards;
	volatile HRADC_struct 	*HRADC_boards[4];
} HRADCs_struct;


/**********************************************************************************************/
//
// 	Prototype statements for variables and functions found in source code HRADC_Boards.c
//
extern volatile HRADCs_struct HRADCs_Info;
extern volatile HRADC_struct  HRADC0_board;
extern volatile HRADC_struct  HRADC1_board;
extern volatile HRADC_struct  HRADC2_board;
extern volatile HRADC_struct  HRADC3_board;

extern volatile Uint32 counterErrorSendCommand;
extern volatile float AverageFilter;

extern void Init_HRADC_Info(volatile HRADC_struct *hradcPtr, Uint16 ID, Uint16 buffer_size, volatile Uint32 *buffer, float transducer_gain, float rburden);
extern void Config_HRADC_board(volatile HRADC_struct *hradcPtr, enum_AN_INPUT AnalogInput, Uint16 enHeater, Uint16 enRails);

extern void SendCommand_HRADC(volatile HRADC_struct *hradcPtr, Uint16 command);
extern Uint16 CheckStatus_HRADC(volatile HRADC_struct *hradcPtr);

extern void Config_HRADC_SoC(float freq);
extern void Enable_HRADC_Sampling(void);
extern void Disable_HRADC_Sampling(void);

extern void Config_HRADC_Sampling_OpMode(Uint16 ID);
extern void Config_HRADC_UFM_OpMode(Uint16 ID);
extern void Erase_HRADC_UFM(Uint16 ID);
extern void Read_HRADC_UFM(Uint16 ID, Uint16 ufm_address, Uint16 n_words, volatile Uint16 *ufm_buffer);
extern void Write_HRADC_UFM(Uint16 ID, Uint16 ufm_address, Uint16 data);

extern void Start_HRADC_Sampling(void);
extern void Stop_HRADC_Sampling(void);

#endif
