#ifndef FBPX4_100KHZ_H
#define FBPX4_100KHZ_H


#define INDEPENDENT_SETPOINT_TEST		1
#define CROSSTALK_TEST					0

/*
 * Especificações da fonte
 */

#define FONTE_MODO		OnePS4Q				// Topologia da fonte a ser controlada
#define	n_PS			1

#define PWM_FREQ	   			50000.0		// Frequencia do sinal PWM gerado [Hz]
#define PWM_DEAD_TIME			300			// Dead-time dos sinais PWM [ns]
#define PWM_MAX_DUTY    		0.9         // Maximo ciclo de trabalho para o PWM (em p.u.)
#define PWM_MIN_DUTY    		-0.9		// Minimo ciclo de trabalho para o PWM (em p.u.)
#define PWM_MAX_DUTY_OL			0.9			// Maximo ciclo de trabalho para o PWM em malha aberta (em p.u.)
#define PWM_MIN_DUTY_OL			-0.9		// Minimo ciclo de trabalho para o PWM em malha aberta (em p.u.)

#define MAX_REF					10.0		// Valor máximo da referência de corrente na carga [A]
#define MIN_REF					-10.0		// Valor mínimo da referência de corrente na carga [A]
#define MAX_LOAD				10.5		// Limite de corrente na carga para interlock [A]

#define MAX_REF_SLEWRATE		1000000.0	// Slew-rate máximo (A/s)
#define MAX_SR_SIGGEN_OFFSET	50.0		// Slew-rate máximo do offset do gerador senoidal (A/s)
#define MAX_SR_SIGGEN_AMP		100.0		// Slew-rate máximo da amplitude do gerador senoidal (A/s)

#define KP						1.9			//2.8
#define KI						559.0		//470.0

#define CONTROL_FREQ			(2.0*PWM_FREQ)
#define CONTROL_PERIOD			(1.0/CONTROL_FREQ)
#define DECIMATION_FACTOR		1
#define TRANSFER_BUFFER_SIZE	DECIMATION_FACTOR
#define HRADC_FREQ_SAMP			(float) CONTROL_FREQ*DECIMATION_FACTOR
#define HRADC_SPI_CLK			SPI_10_71MHz

#define	BUFFER_DECIMATION		1

#define TRANSDUCER_0_INPUT_RATED		12.5			// 			LEM ITN 12-P:
#define TRANSDUCER_0_OUTPUT_RATED		0.05			//   In_rated 	= +/- 12.5 A
#define TRANSDUCER_0_OUTPUT_TYPE		Iin_bipolar		//   Out_rated 	= +/- 50 mA
#if (HRADC_v2_0)
	#define TRANSDUCER_0_GAIN				-(TRANSDUCER_0_INPUT_RATED/TRANSDUCER_0_OUTPUT_RATED)
#endif

#if (HRADC_v2_1)
	#define TRANSDUCER_0_GAIN				(TRANSDUCER_0_INPUT_RATED/TRANSDUCER_0_OUTPUT_RATED)
#endif
#define HRADC_0_R_BURDEN				20.0			// Resistor Burden = 20 R

#define TRANSDUCER_1_INPUT_RATED		12.5			// 			LEM ITN 12-P:
#define TRANSDUCER_1_OUTPUT_RATED		0.05			//   In_rated 	= +/- 12.5 A
#define TRANSDUCER_1_OUTPUT_TYPE		Iin_bipolar		//   Out_rated 	= +/- 50 mA
#if (HRADC_v2_0)
	#define TRANSDUCER_1_GAIN				-(TRANSDUCER_1_INPUT_RATED/TRANSDUCER_1_OUTPUT_RATED)
#endif

#if (HRADC_v2_1)
	#define TRANSDUCER_1_GAIN				(TRANSDUCER_1_INPUT_RATED/TRANSDUCER_1_OUTPUT_RATED)
#endif
#define HRADC_1_R_BURDEN				20.0			// Resistor Burden = 20 R

#define TRANSDUCER_2_INPUT_RATED		12.5			// 			LEM ITN 12-P:
#define TRANSDUCER_2_OUTPUT_RATED		0.05			//   In_rated 	= +/- 12.5 A
#define TRANSDUCER_2_OUTPUT_TYPE		Iin_bipolar		//   Out_rated 	= +/- 50 mA
#if (HRADC_v2_0)
	#define TRANSDUCER_2_GAIN				-(TRANSDUCER_2_INPUT_RATED/TRANSDUCER_2_OUTPUT_RATED)
#endif

#if (HRADC_v2_1)
	#define TRANSDUCER_2_GAIN				(TRANSDUCER_2_INPUT_RATED/TRANSDUCER_2_OUTPUT_RATED)
#endif
#define HRADC_2_R_BURDEN				20.0				// Resistor Burden = 20 R

#define TRANSDUCER_3_INPUT_RATED		12.5			// 			LEM ITN 12-P:
#define TRANSDUCER_3_OUTPUT_RATED		0.05			//   In_rated 	= +/- 12.5 A
#define TRANSDUCER_3_OUTPUT_TYPE		Iin_bipolar		//   Out_rated 	= +/- 50 mA
#if (HRADC_v2_0)
	#define TRANSDUCER_3_GAIN				-(TRANSDUCER_3_INPUT_RATED/TRANSDUCER_3_OUTPUT_RATED)
#endif

#if (HRADC_v2_1)
	#define TRANSDUCER_3_GAIN				(TRANSDUCER_3_INPUT_RATED/TRANSDUCER_3_OUTPUT_RATED)
#endif
#define HRADC_3_R_BURDEN				20.0			// Resistor Burden = 20 R


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

#define PIN_OPEN_PS1_DCLINK_RELAY		GpioDataRegs.GPCCLEAR.bit.GPIO64 = 1;		// GPDO4: PS1_OUTPUT_CTRL (FBP v4.0)
#define PIN_CLOSE_PS1_DCLINK_RELAY		GpioDataRegs.GPCSET.bit.GPIO64 = 1;

#define PIN_OPEN_PS2_DCLINK_RELAY		GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;		// GPDO3: PS2_OUTPUT_CTRL (FBP v4.0)
#define PIN_CLOSE_PS2_DCLINK_RELAY		GpioDataRegs.GPCSET.bit.GPIO66 = 1;

#define PIN_OPEN_PS3_DCLINK_RELAY		GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;		// GPDO1: PS3_OUTPUT_CTRL (FBP v4.0)
#define PIN_CLOSE_PS3_DCLINK_RELAY		GpioDataRegs.GPCSET.bit.GPIO67 = 1;

#define PIN_OPEN_PS4_DCLINK_RELAY		GpioDataRegs.GPCCLEAR.bit.GPIO65 = 1;		// GPDO2: PS4_OUTPUT_CTRL (FBP v4.0)
#define PIN_CLOSE_PS4_DCLINK_RELAY		GpioDataRegs.GPCSET.bit.GPIO65 = 1;

extern void main_FBPx4_100kHz(void);

#endif
