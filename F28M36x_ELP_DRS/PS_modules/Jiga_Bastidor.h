#ifndef JIGA_BASTIDOR_H
#define JIGA_BASTIDOR_H

/*
 * Especificações da fonte
 */

#define USE_ITLK

#define PWM_FREQ	   			50000.0		// Frequencia do sinal PWM gerado [Hz]
#define PWM_DEAD_TIME			300			// Dead-time dos sinais PWM [ns]
#define PWM_MAX_DUTY    		0.9         // Maximo ciclo de trabalho para o PWM (em p.u.)
#define PWM_MIN_DUTY    		-0.9		// Minimo ciclo de trabalho para o PWM (em p.u.)
#define PWM_MAX_DUTY_OL			0.9			// Maximo ciclo de trabalho para o PWM em malha aberta (em p.u.)
#define PWM_MIN_DUTY_OL			-0.9		// Minimo ciclo de trabalho para o PWM em malha aberta (em p.u.)

#define MAX_REF					10.0		// Valor máximo da referência de corrente na carga [A]
#define MIN_REF					-10.0		// Valor mínimo da referência de corrente na carga [A]
#define MAX_ILOAD				10.5		// Limite de corrente na carga para interlock [A]
#define MAX_VLOAD				10.5			// Limite de tensão na carga para interlock [V]
#define MIN_DCLINK				3.0		// Limite mínimo de tensão no DC-Link para interlock [V]
#define MAX_DCLINK				17.0		// Limite máximo de tensão no DC-Link para interlock [V]
#define MAX_TEMP				80.0		// Limite de temperatura nos módulos de potência para interlock [ºC]

#define MAX_REF_SLEWRATE		1000000.0	// Slew-rate máximo (A/s)
#define MAX_SR_SIGGEN_OFFSET	50.0		// Slew-rate máximo do offset do gerador senoidal (A/s)
#define MAX_SR_SIGGEN_AMP		100.0		// Slew-rate máximo da amplitude do gerador senoidal (A/s)

#define KP						0.0         // <= CARGA RESISTIVA WEG           //0.0  <= CARGA RESISTIVA WEG              //0.08976		//1.9			//2.8
#define KI						680.0       // <= Ro ~ 1.3 Ohm, DC-Link ~ 6 V   //280  <= Ro ~ 1.3 Ohm, DC-Link ~ 14.5 V   //148.10		// 559.0		//470.0

#define CONTROL_FREQ			(2.0*PWM_FREQ)
#define CONTROL_PERIOD			(1.0/CONTROL_FREQ)
#define DECIMATION_FACTOR		1
#define TRANSFER_BUFFER_SIZE	DECIMATION_FACTOR
#define HRADC_FREQ_SAMP			(float) CONTROL_FREQ*DECIMATION_FACTOR
#define HRADC_SPI_CLK			SPI_15MHz

#define	BUFFER_DECIMATION		1

#define TRANSDUCER_0_INPUT_RATED		12.5			// 			LEM ITN 12-P:
#define TRANSDUCER_0_OUTPUT_RATED		0.05			//   In_rated 	= +/- 12.5 A
#define TRANSDUCER_0_OUTPUT_TYPE		Iin_bipolar		//   Out_rated 	= +/- 50 mA
#define TRANSDUCER_0_GAIN				(TRANSDUCER_0_INPUT_RATED/TRANSDUCER_0_OUTPUT_RATED)
#define HRADC_0_R_BURDEN				20.0			// Resistor Burden = 20 R

#define TRANSDUCER_1_INPUT_RATED		12.5			// 			LEM ITN 12-P:
#define TRANSDUCER_1_OUTPUT_RATED		0.05			//   In_rated 	= +/- 12.5 A
#define TRANSDUCER_1_OUTPUT_TYPE		Iin_bipolar		//   Out_rated 	= +/- 50 mA
#define TRANSDUCER_1_GAIN				(TRANSDUCER_1_INPUT_RATED/TRANSDUCER_1_OUTPUT_RATED)
#define HRADC_1_R_BURDEN				20.0			// Resistor Burden = 20 R

#define TRANSDUCER_2_INPUT_RATED		12.5			// 			LEM ITN 12-P:
#define TRANSDUCER_2_OUTPUT_RATED		0.05			//   In_rated 	= +/- 12.5 A
#define TRANSDUCER_2_OUTPUT_TYPE		Iin_bipolar		//   Out_rated 	= +/- 50 mA
#define TRANSDUCER_2_GAIN				(TRANSDUCER_2_INPUT_RATED/TRANSDUCER_2_OUTPUT_RATED)
#define HRADC_2_R_BURDEN				20.0				// Resistor Burden = 20 R

#define TRANSDUCER_3_INPUT_RATED		12.5			// 			LEM ITN 12-P:
#define TRANSDUCER_3_OUTPUT_RATED		0.05			//   In_rated 	= +/- 12.5 A
#define TRANSDUCER_3_OUTPUT_TYPE		Iin_bipolar		//   Out_rated 	= +/- 50 mA
#define TRANSDUCER_3_GAIN				(TRANSDUCER_3_INPUT_RATED/TRANSDUCER_3_OUTPUT_RATED)
#define HRADC_3_R_BURDEN				20.0			// Resistor Burden = 20 R

#define TRANSDUCER_GAIN                 TRANSDUCER_0_GAIN
/*
 * DP modules mnemonics
 */

#define ERROR_CALCULATOR_PS1			&DP_Framework.DPlibrary.ELP_Error[0]
#define	PI_DAWU_CONTROLLER_ILOAD_PS1	&DP_Framework.DPlibrary.ELP_PI_dawu[0]

#define ERROR_CALCULATOR_PS2			&DP_Framework.DPlibrary.ELP_Error[1]
#define	PI_DAWU_CONTROLLER_ILOAD_PS2	&DP_Framework.DPlibrary.ELP_PI_dawu[1]

#define ERROR_CALCULATOR_PS3			&DP_Framework.DPlibrary.ELP_Error[2]
#define	PI_DAWU_CONTROLLER_ILOAD_PS3	&DP_Framework.DPlibrary.ELP_PI_dawu[2]

#define ERROR_CALCULATOR_PS4			&DP_Framework.DPlibrary.ELP_Error[3]
#define	PI_DAWU_CONTROLLER_ILOAD_PS4	&DP_Framework.DPlibrary.ELP_PI_dawu[3]

/*
 * Digital IO's defines
 */

// Power supply 1

#define PS1_ID							0x0001

#define PIN_OPEN_PS1_DCLINK_RELAY		GpioDataRegs.GPCCLEAR.bit.GPIO64 = 1;		// GPDO4: PS1_OUTPUT_CTRL (FBP v4.0)
#define PIN_CLOSE_PS1_DCLINK_RELAY		GpioDataRegs.GPCSET.bit.GPIO64 = 1;

#define PIN_STATUS_PS1_DCLINK_RELAY		GpioDataRegs.GPDDAT.bit.GPIO125				// GPDI4
#define PIN_STATUS_PS1_DRIVER_ERROR		GpioG2DataRegs.GPGDAT.bit.GPIO195			// GPDI5
#define PIN_STATUS_PS1_FUSE				GpioG2DataRegs.GPGDAT.bit.GPIO196			// GPDI14

#define PS1_LOAD_CURRENT				DP_Framework.NetSignals[5]					// HRADC0
#define PS1_LOAD_VOLTAGE				DP_Framework_MtoC.NetSignals[9]				// ANI6
#define PS1_DCLINK_VOLTAGE				DP_Framework_MtoC.NetSignals[5]				// ANI2
#define PS1_TEMPERATURE					DP_Framework_MtoC.NetSignals[13]			// I2C Add 0x48

#define	PS1_LOAD_OVERCURRENT			0x00000001
#define PS1_LOAD_OVERVOLTAGE			0x00000002
#define PS1_DCLINK_OVERVOLTAGE			0x00000004
#define PS1_DCLINK_UNDERVOLTAGE			0x00000008
#define PS1_DCLINK_RELAY_FAIL			0x00000010
#define PS1_FUSE_FAIL					0x00000020
#define PS1_DRIVER_FAIL					0x00000040
#define PS1_OVERTEMP					0x00000080


// Power supply 2

#define PS2_ID							0x0002

#define PIN_OPEN_PS2_DCLINK_RELAY		GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;		// GPDO3: PS2_OUTPUT_CTRL (FBP v4.0)
#define PIN_CLOSE_PS2_DCLINK_RELAY		GpioDataRegs.GPCSET.bit.GPIO66 = 1;

#define PIN_STATUS_PS2_DCLINK_RELAY		GpioDataRegs.GPDDAT.bit.GPIO113				// GPDI11
#define PIN_STATUS_PS2_DRIVER_ERROR		GpioDataRegs.GPDDAT.bit.GPIO109				// GPDI9
#define PIN_STATUS_PS2_FUSE				GpioG2DataRegs.GPGDAT.bit.GPIO198			// GPDI15

#define PS2_LOAD_CURRENT				DP_Framework.NetSignals[7]					// HRADC1
#define PS2_LOAD_VOLTAGE				DP_Framework_MtoC.NetSignals[10]			// ANI7
#define PS2_DCLINK_VOLTAGE				DP_Framework_MtoC.NetSignals[6]				// ANI1
#define PS2_TEMPERATURE					DP_Framework_MtoC.NetSignals[14]			// I2C Add 0x49

#define	PS2_LOAD_OVERCURRENT			0x00000100
#define PS2_LOAD_OVERVOLTAGE			0x00000200
#define PS2_DCLINK_OVERVOLTAGE			0x00000400
#define PS2_DCLINK_UNDERVOLTAGE			0x00000800
#define PS2_DCLINK_RELAY_FAIL			0x00001000
#define PS2_FUSE_FAIL					0x00002000
#define PS2_DRIVER_FAIL					0x00004000
#define PS2_OVERTEMP					0x00008000


// Power supply 3

#define PS3_ID							0x0004

#define PIN_OPEN_PS3_DCLINK_RELAY		GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;		// GPDO1: PS3_OUTPUT_CTRL (FBP v4.0)
#define PIN_CLOSE_PS3_DCLINK_RELAY		GpioDataRegs.GPCSET.bit.GPIO67 = 1;

#define PIN_STATUS_PS3_DCLINK_RELAY		GpioG2DataRegs.GPGDAT.bit.GPIO192			// GPDI8
#define PIN_STATUS_PS3_DRIVER_ERROR		GpioDataRegs.GPDDAT.bit.GPIO126				// GPDI1
#define PIN_STATUS_PS3_FUSE				GpioG2DataRegs.GPGDAT.bit.GPIO197			// GPDI13

#define PS3_LOAD_CURRENT				DP_Framework.NetSignals[9]					// HRADC2
#define PS3_LOAD_VOLTAGE				DP_Framework_MtoC.NetSignals[11]			// ANI3
#define PS3_DCLINK_VOLTAGE				DP_Framework_MtoC.NetSignals[7]				// ANI4
#define PS3_TEMPERATURE					DP_Framework_MtoC.NetSignals[15]			// I2C Add 0x4A

#define	PS3_LOAD_OVERCURRENT			0x00010000
#define PS3_LOAD_OVERVOLTAGE			0x00020000
#define PS3_DCLINK_OVERVOLTAGE			0x00040000
#define PS3_DCLINK_UNDERVOLTAGE			0x00080000
#define PS3_DCLINK_RELAY_FAIL			0x00100000
#define PS3_FUSE_FAIL					0x00200000
#define PS3_DRIVER_FAIL					0x00400000
#define PS3_OVERTEMP					0x00800000


// Power supply 4

#define PS4_ID							0x0008

#define PIN_OPEN_PS4_DCLINK_RELAY		GpioDataRegs.GPCCLEAR.bit.GPIO65 = 1;		// GPDO2: PS4_OUTPUT_CTRL (FBP v4.0)
#define PIN_CLOSE_PS4_DCLINK_RELAY		GpioDataRegs.GPCSET.bit.GPIO65 = 1;

#define PIN_STATUS_PS4_DCLINK_RELAY		GpioDataRegs.GPDDAT.bit.GPIO127				// GPDI2
#define PIN_STATUS_PS4_DRIVER_ERROR		GpioDataRegs.GPDDAT.bit.GPIO124				// GPDI3
#define PIN_STATUS_PS4_FUSE				GpioG2DataRegs.GPGDAT.bit.GPIO199			// GPDI16

#define PS4_LOAD_CURRENT				DP_Framework.NetSignals[11]					// HRADC3
#define PS4_LOAD_VOLTAGE				DP_Framework_MtoC.NetSignals[12]			// ANI5
#define PS4_DCLINK_VOLTAGE				DP_Framework_MtoC.NetSignals[8]				// ANI0
#define PS4_TEMPERATURE					DP_Framework_MtoC.NetSignals[16]			// I2C Add 0x4C

#define	PS4_LOAD_OVERCURRENT			0x01000000
#define PS4_LOAD_OVERVOLTAGE			0x02000000
#define PS4_DCLINK_OVERVOLTAGE			0x04000000
#define PS4_DCLINK_UNDERVOLTAGE			0x08000000
#define PS4_DCLINK_RELAY_FAIL			0x10000000
#define PS4_FUSE_FAIL					0x20000000
#define PS4_DRIVER_FAIL					0x40000000
#define PS4_OVERTEMP					0x80000000

#define PS_ALL_ID						0x000F

extern void main_Jiga_Bastidor(void);

#endif
