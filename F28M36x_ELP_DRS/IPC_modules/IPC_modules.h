	/*
 * 		FILE: 		IPC_modules.h
 * 		PROJECT: 	DRS v2.0
 * 		CREATION:	05/11/2015
 * 		MODIFIED:	05/11/2015
 *
 * 		AUTHOR: 	Ricieri  (LNLS/ELP)
 *
 * 		DESCRIPTION:
 *		Source code for interprocessor communications (IPC)
 *
 *		TODO:
 */


#ifndef IPC_MODULES_H
#define IPC_MODULES_H

#include "F28M36x_ELP_DRS.h"

#define WFMREF_SAMPLING_FREQ	4096.0

#define DP_MODULE_MAX_COEFF		16

#define N_SWEEP_FREQS			37
#define N_MAX_REF				4
#define N_MAX_DIG				8
#define	N_MAX_AI				8

/*
 * MtoC Message Defines
 */
#define IPC_PS_ON_OFF			0x00000011 //IPC1+IPC5
#define OPERATING_MODE			0x00000021 //IPC1+IPC6
#define OPEN_CLOSE_LOOP			0x00000041 //IPC1+IPC7
#define SLOWREF_UPDATE			0x00000081 //IPC1+IPC8
#define SIGGEN_ENABLE			0x00000101 //IPC1+IPC9
#define SIGGEN_CONFIG			0x00000201 //IPC1+IPC10
#define DPMODULES_CONFIG		0x00000401 //IPC1+IPC11
#define SAMPLES_BUFFER_ON_OFF	0x00000801 //IPC1+IPC12
#define RESET_INTERLOCKS		0x00001001 //IPC1+IPC13
#define RESET_WFMREF			0x00002001 //IPC1+IPC14
//...//
#define HRADC_SELECT_BOARD		0x02000001 //IPC1+IPC26
#define HRADC_TEST_SOURCE		0x04000001 //IPC1+IPC27
#define HRADC_SAMPLING_DISABLE	0x08000001 //IPC1+IPC28
#define HRADC_SAMPLING_ENABLE	0x10000001 //IPC1+IPC29
#define HRADC_OPMODE			0x20000001 //IPC1+IPC30
#define HRADC_CONFIG			0x40000001 //IPC1+IPC31
#define CTOM_MESSAGE_ERROR		0x80000001 //IPC1+IPC32

#define WFMREF_SYNC				0x00000002 //IPC2
#define SOFT_INTERLOCK_MTOC		0x00000004 //IPC3
#define HARD_INTERLOCK_MTOC		0x00000008 //IPC4

/*
 * CtoM Message Defines
 */
#define IPC5				0x00000011 //IPC1+IPC5
#define IPC6				0x00000021 //IPC1+IPC6
#define IPC7				0x00000041 //IPC1+IPC7
#define IPC8				0x00000081 //IPC1+IPC8
#define IPC9				0x00000101 //IPC1+IPC9
#define IPC10				0x00000201 //IPC1+IPC10
#define IPC11				0x00000401 //IPC1+IPC11
#define MTOC_MESSAGE_ERROR	0x80000001 //IPC1+IPC32
#define SOFT_INTERLOCK_CTOM	0x00000002 //IPC2
#define HARD_INTERLOCK_CTOM	0x00000004 //IPC3
#define IPC4				0x00000008 //IPC4

/*
 * CtoM Hard Interlocks Defines
 */
#define	LOAD_OVERCURRENT		0x00000001
#define	EXTERNAL_INTERLOCK		0x00000002
#define	AC_FAULT				0x00000004
#define	ACDC_FAULT				0x00000008
#define	DCDC_FAULT				0x00000010
#define	LOAD_OVERVOLTAGE		0x00000020
#define PRECHARGERS_FAULT		0x00000040
#define OUTPUT_CAP_CHARGE_FAULT	0x00000080
#define	EMERGENCY_BUTTON		0x00000100
#define OUT_OVERVOLTAGE			0x00000200
#define IN_OVERVOLTAGE			0x00000400
#define ARM1_OVERCURRENT		0x00000800
#define ARM2_OVERCURRENT		0x00001000
#define IN_OVERCURRENT			0x00002000
#define DRIVER1_FAULT			0x00004000
#define DRIVER2_FAULT			0x00008000
#define OUT1_OVERCURRENT		0x00010000
#define OUT2_OVERCURRENT		0x00020000
#define OUT1_OVERVOLTAGE		0x00040000
#define	OUT2_OVERVOLTAGE		0x00080000
#define LEAKAGE_OVERCURRENT		0x00100000
#define AC_OVERCURRENT			0x00200000

/*
 * CtoM Soft Interlocks Defines
 */
#define IGBT1_OVERTEMP			0x00000001
#define IGBT2_OVERTEMP			0x00000002
#define L1_OVERTEMP				0x00000004
#define L2_OVERTEMP				0x00000008
#define HEATSINK_OVERTEMP		0x00000010
#define WATER_OVERTEMP			0x00000020
#define RECTFIER1_OVERTEMP		0x00000040
#define RECTFIER2_OVERTEMP		0x00000080
#define AC_TRANSF_OVERTEMP		0x00000100
#define WATER_FLUX_FAULT		0x00000200
#define OVER_HUMIDITY_FAULT		0x00000400

#define SIZE_SAMPLES_BUFFER		4096
#define SIZE_WFMREF_BLOCK		2048

typedef enum {NO_ERROR_CTOM,

			  ERROR1,
	 	 	  ERROR2,
			  ERROR3,
			  ERROR4} eCTOMerror;

typedef enum {NO_ERROR_MTOC,
			  INVALID_OPMODE,
	 	 	  INVALID_DP_MODULE,
			  IPC_LOW_PRIORITY_MSG_FULL,
			  HRADC_CONFIG_ERROR} eMTOCerror;

typedef enum {OneShot,
			  SampleBySample,
			  SampleBySample_Continuous} eSyncMode;

typedef volatile struct
{							 // Bits	Description
	Uint16 bit0 : 1;         // 0
	Uint16 bit1 : 1;         // 1
	Uint16 bit2 : 1;         // 2
	Uint16 bit3 : 1;         // 3
	Uint16 bit4 : 1;         // 4
	Uint16 bit5 : 1;         // 5
	Uint16 bit6 : 1;         // 6
	Uint16 bit7 : 1;         // 7
	Uint16 bit8 : 1;         // 8
	Uint16 bit9 : 1;         // 9
	Uint16 bit10: 1;         // 10
	Uint16 bit11: 1;         // 11
	Uint16 bit12: 1;         // 12
	Uint16 bit13: 1;         // 13
	Uint16 bit14: 1;         // 14
	Uint16 bit15: 1;         // 15
	Uint16 bit16: 1;         // 16
	Uint16 bit17: 1;         // 17
	Uint16 bit18: 1;         // 18
	Uint16 bit19: 1;         // 19
	Uint16 bit20: 1;         // 20
	Uint16 bit21: 1;         // 21
	Uint16 bit22: 1;         // 22
	Uint16 bit23: 1;         // 23
	Uint16 bit24: 1;         // 24
	Uint16 bit25: 1;         // 25
	Uint16 bit26: 1;         // 26
	Uint16 bit27: 1;         // 27
	Uint16 bit28: 1;         // 28
	Uint16 bit29: 1;         // 29
	Uint16 bit30: 1;         // 30
	Uint16 bit31: 1;         // 31
} tSoftItlk;

typedef volatile struct
{							 		// Bits	Description
	Uint16 LoadOverCurrent : 1;		// 0
	Uint16 ExternalInterlock : 1;	// 1
	Uint16 AcFault : 1;         	// 2
	Uint16 ACDCFault : 1;         	// 3
	Uint16 DCDCFault : 1;			// 4
	Uint16 LoadOverVoltage : 1;     // 5
	Uint16 bit6 : 1;         		// 6
	Uint16 bit7 : 1;         // 7
	Uint16 bit8 : 1;         // 8
	Uint16 bit9 : 1;         // 9
	Uint16 bit10: 1;         // 10
	Uint16 bit11: 1;         // 11
	Uint16 bit12: 1;         // 12
	Uint16 bit13: 1;         // 13
	Uint16 bit14: 1;         // 14
	Uint16 bit15: 1;         // 15
	Uint16 bit16: 1;         // 16
	Uint16 bit17: 1;         // 17
	Uint16 bit18: 1;         // 18
	Uint16 bit19: 1;         // 19
	Uint16 bit20: 1;         // 20
	Uint16 bit21: 1;         // 21
	Uint16 bit22: 1;         // 22
	Uint16 bit23: 1;         // 23
	Uint16 bit24: 1;         // 24
	Uint16 bit25: 1;         // 25
	Uint16 bit26: 1;         // 26
	Uint16 bit27: 1;         // 27
	Uint16 bit28: 1;         // 28
	Uint16 bit29: 1;         // 29
	Uint16 bit30: 1;         // 30
	Uint16 bit31: 1;         // 31
} tHardItlk;

typedef volatile struct
{
	ePSModel 	Model;
	Uint16 		OnOff;
	ePSOpMode	OpMode;
	Uint16 		OpenLoop;
	Uint16		LocalRemote;
	Uint32		SoftInterlocks;
	Uint32		HardInterlocks;
	Uint16		BufferOnOff;
	float		ISlowRef;
	eCTOMerror	ErrorCtoM;
} tPSModuleMtoC;

typedef volatile struct
{
	Uint16 		OnOff;
	ePSOpMode	OpMode;
	Uint16 		OpenLoop;
	Uint32		SoftInterlocks;
	Uint32		HardInterlocks;
	Uint16		BufferOnOff;
	float		IRef;
	eMTOCerror	ErrorMtoC;
} tPSModuleCtoM;

typedef volatile struct
{
	tBuffer		BufferInfo;
	float		Gain;
	float		Offset;
	eSyncMode	SyncMode;
} tWfmRef;

typedef volatile struct
{
	volatile float A[SIZE_WFMREF_BLOCK];
	volatile float B[SIZE_WFMREF_BLOCK];
} tWfmBlock;

union uWfmRef
{
	tWfmBlock 		WfmRef_Block;
	volatile float 	WfmRef_Blocks[2*SIZE_WFMREF_BLOCK];
};

typedef volatile struct
{
       Uint16		Enable;
       eSigGenType	Type;
       Uint16		Ncycles;
       float		PhaseStart;
       float		PhaseEnd;
       float		Freq;
       float		Amplitude[N_SWEEP_FREQS];
       float		Offset;
       float		Aux;
} tSigGen;


typedef volatile struct
{
	Uint16		ID;
	eDPclass	DPclass;
	float		Coeffs[DP_MODULE_MAX_COEFF];
} tDPModule;

typedef volatile struct
{
	Uint16			ID;
	float			FreqSampling;
	eHRADCOpMode	OpMode;
	eInputType		InputType;
	Uint16			EnableHeater;
	Uint16			EnableMonitor;
} tHRADCConfig;

typedef volatile struct
{
	tPSModuleCtoM	PSModule;
	tWfmRef			WfmRef;
	tBuffer			SamplesBuffer;
} tIPC_CTOM_MSG_RAM;

typedef volatile struct
{
	tPSModuleMtoC	PSModule;
	tWfmRef 		WfmRef;
	tSigGen			SigGen;
	tDPModule		DPModule;
	tHRADCConfig	HRADCConfig;
} tIPC_MTOC_MSG_RAM;

typedef volatile struct
{
		float		Ref_max[N_MAX_REF];
		float		SR_ref_max[N_MAX_REF];

		Uint16			HRADC_K_decim;
		float			HRADC_Transducer_InputRated[N_MAX_HRADC];
		float			HRADC_Transducer_OutputRated[N_MAX_HRADC];
		eInputType		HRADC_Transducer_OutputType[N_MAX_HRADC];
		Uint16			HRADC_EnableHeater[N_MAX_HRADC];
		Uint16			HRADC_EnableRails[N_MAX_HRADC];

		ePWMConfig	PWM_Config;
		Uint16		N_PowerModules;
		double		Freq_PWM[N_MAX_PWM_MODULES];
		float		T_dead_min[N_MAX_PWM_MODULES];
		float		T_dead[N_MAX_PWM_MODULES];
		float		d_min_lim[2*N_MAX_PWM_MODULES];
		float		d_min[2*N_MAX_PWM_MODULES];
		float		d_max_lim[2*N_MAX_PWM_MODULES];
		float		d_max[2*N_MAX_PWM_MODULES];
		float		dD_max_lim[2*N_MAX_PWM_MODULES];
		float		dD_max[2*N_MAX_PWM_MODULES];

		Uint16		T_Off;
		Uint16		T_Stg;

		Uint16		C_Dig[N_MAX_DIG];
		Uint16		N_Dig[N_MAX_DIG];

		float		V_min_NC[N_MAX_AI];
		float		V_max_NC[N_MAX_AI];
		float		V_min_CR[N_MAX_AI];
		float		V_max_CR[N_MAX_AI];
		float		V_min_SC[N_MAX_AI];
		float		V_max_SC[N_MAX_AI];
		Uint16		N_NC[N_MAX_AI];
		Uint16		N_CR[N_MAX_AI];
		Uint16		N_SC[N_MAX_AI];
		Uint16		K_decim[N_MAX_AI];
} tIPC_MTOC_PARAM_RAM;

typedef volatile struct
{
	void (*PS_turnOn)(void);
	void (*PS_turnOff)(void);
} tIPC_PS_FUNCS;

/****************/
extern volatile Uint16			wfmSyncFlag;
extern volatile float 			samplesBuffer[SIZE_SAMPLES_BUFFER];
extern volatile union uWfmRef	wfmRef_Curve;

extern tIPC_CTOM_MSG_RAM	IPC_CtoM_Msg;
extern tIPC_MTOC_MSG_RAM	IPC_MtoC_Msg;
extern tIPC_MTOC_PARAM_RAM	IPC_MtoC_Param;
extern tIPC_PS_FUNCS		IPC_PS_funcs;

extern void InitIPC(void (*ps_turnOn)(void), void (*ps_turnOff)(void), void (*isr_SoftItlk)(void), void (*isr_HardItlk)(void));
extern void SendIpcFlag(Uint32 flag);

extern interrupt void isr_IPC_Channel_1(void);
extern interrupt void isr_IPC_Channel_2(void);
extern interrupt void isr_IPC_Channel_3(void);
extern interrupt void isr_IPC_Channel_4(void);

extern void ConfigPSOpMode(ePSOpMode opMode);

#endif
