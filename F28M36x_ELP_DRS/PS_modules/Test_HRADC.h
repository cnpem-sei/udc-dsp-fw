#ifndef TEST_HRADC_H
#define TEST_HRADC_H

/*
 * Especificações do teste
 */

#define PWM_FREQ	   			100000				// Frequencia do sinal PWM gerado [Hz]
#define CONTROL_FREQ			(1.0*PWM_FREQ)
#define CONTROL_PERIOD			(1.0/CONTROL_FREQ)
#define DECIMATION_FACTOR		1
#define TRANSFER_BUFFER_SIZE	DECIMATION_FACTOR
#define	N_HRADC_BOARDS			1

#define TRANSDUCER_0_INPUT_RATED		12.5				//
#define TRANSDUCER_0_OUTPUT_RATED		0.05				//   In_rated 	= +/- 10A V
#define TRANSDUCER_0_OUTPUT_TYPE		Iin_bipolar			//   Out_rated 	= +/- 10 V
#define TRANSDUCER_0_GAIN				(TRANSDUCER_0_INPUT_RATED/TRANSDUCER_0_OUTPUT_RATED)
#define HRADC_0_R_BURDEN				20.0				// Resistor Burden = 20 R

#define TRANSDUCER_1_INPUT_RATED		10.0				//
#define TRANSDUCER_1_OUTPUT_RATED		10.0				//   In_rated 	= +/- 10 A
#define TRANSDUCER_1_OUTPUT_TYPE		Vin_bipolar			//   Out_rated 	= +/- 100 mA
#define TRANSDUCER_1_GAIN				(TRANSDUCER_1_INPUT_RATED/TRANSDUCER_1_OUTPUT_RATED)
#define HRADC_1_R_BURDEN				5.0				// Resistor Burden = 5 R

#define TRANSDUCER_2_INPUT_RATED		110.0				// Divisor de Tensão + Isolador Verivolt IsoBlock V-4:
#define TRANSDUCER_2_OUTPUT_RATED		10.0				//   In_rated 	= +/- 110 V
#define TRANSDUCER_2_OUTPUT_TYPE		Vin_bipolar			//   Out_rated 	= +/- 10 V
#define TRANSDUCER_2_GAIN				(TRANSDUCER_2_INPUT_RATED/TRANSDUCER_2_OUTPUT_RATED)
#define HRADC_2_R_BURDEN				20.0				// Resistor Burden = 20 R

#define TRANSDUCER_3_INPUT_RATED		200.0				// LEM LF 210-S
#define TRANSDUCER_3_OUTPUT_RATED		-0.1				//   In_rated 	= +/- 200 A
#define TRANSDUCER_3_OUTPUT_TYPE		Iin_bipolar			//   Out_rated 	= +/- 100 mA
#define TRANSDUCER_3_GAIN				(TRANSDUCER_3_INPUT_RATED/TRANSDUCER_3_OUTPUT_RATED)
#define HRADC_3_R_BURDEN				10.0				// Resistor Burden = 10 R

#define HRADC_R_BURDEN				20.0				// Resistor Burden = 20 R
#define HRADC_VIN_BI_P_GAIN			(20.0/262144.0)
#define HRADC_IIN_BI_P_GAIN			(1.0/(HRADC_R_BURDEN * 131072.0))

extern void main_Test_HRADC(void);

#endif
