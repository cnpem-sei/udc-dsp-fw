#ifndef JIGA_HRADC_V2_1_H
#define JIGA_HRADC_V2_1_H

/*
 * Especificações do teste
 */

#define PWM_FREQ	   			100000				// Frequencia do sinal PWM gerado [Hz]
#define CONTROL_FREQ			(1.0*PWM_FREQ)
#define CONTROL_PERIOD			(1.0/CONTROL_FREQ)
#define DECIMATION_FACTOR		1
#define TRANSFER_BUFFER_SIZE	DECIMATION_FACTOR
#define	N_HRADC_BOARDS			1

#define HRADC_FREQ_SAMP			(float) CONTROL_FREQ*DECIMATION_FACTOR
#define HRADC_SPI_CLK			SPI_15MHz

#define TRANSDUCER_0_INPUT_RATED		10.0				//
#define TRANSDUCER_0_OUTPUT_RATED		10.0				//   In_rated 	= +/- 10 A
#define TRANSDUCER_0_OUTPUT_TYPE		Vin_bipolar			//   Out_rated 	= +/- 10 V
#define TRANSDUCER_0_GAIN				(TRANSDUCER_0_INPUT_RATED/TRANSDUCER_0_OUTPUT_RATED)
#define HRADC_0_R_BURDEN				0.1					// Resistor Burden = 0.1 R
															// This value makes Vin and Iin gain equal

#define TRANSDUCER_1_INPUT_RATED		10.0				//
#define TRANSDUCER_1_OUTPUT_RATED		10.0				//   In_rated 	= +/- 10 A
#define TRANSDUCER_1_OUTPUT_TYPE		Iin_bipolar			//   Out_rated 	= +/- 10 V
#define TRANSDUCER_1_GAIN				(TRANSDUCER_0_INPUT_RATED/TRANSDUCER_0_OUTPUT_RATED)
#define HRADC_1_R_BURDEN				0.1					// Resistor Burden = 0.1 R
															// This value makes Vin and Iin gain equal

#define TRANSDUCER_2_INPUT_RATED		10.0				//
#define TRANSDUCER_2_OUTPUT_RATED		10.0				//   In_rated 	= +/- 10 A
#define TRANSDUCER_2_OUTPUT_TYPE		Vref_bipolar_p			//   Out_rated 	= +/- 10 V
#define TRANSDUCER_2_GAIN				(TRANSDUCER_0_INPUT_RATED/TRANSDUCER_0_OUTPUT_RATED)
#define HRADC_2_R_BURDEN				0.1					// Resistor Burden = 0.1 R
															// This value makes Vin and Iin gain equal

#define TRANSDUCER_3_INPUT_RATED		10.0				//
#define TRANSDUCER_3_OUTPUT_RATED		10.0				//   In_rated 	= +/- 10 A
#define TRANSDUCER_3_OUTPUT_TYPE		Vref_bipolar_n			//   Out_rated 	= +/- 10 V
#define TRANSDUCER_3_GAIN				(TRANSDUCER_0_INPUT_RATED/TRANSDUCER_0_OUTPUT_RATED)
#define HRADC_3_R_BURDEN				0.1					// Resistor Burden = 0.1 R
															// This value makes Vin and Iin gain equal

extern void main_Jiga_HRADC_v2_1(void);

#endif
