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
#define	N_HRADC_BOARDS			4

#define HRADC_FREQ_SAMP			(float) CONTROL_FREQ*DECIMATION_FACTOR
#define HRADC_SPI_CLK			SPI_15MHz

#define TRANSDUCER_INPUT_RATED		10.0				//
#define TRANSDUCER_OUTPUT_RATED		10.0				//   In_rated 	= +/- 10 A
#define TRANSDUCER_OUTPUT_TYPE		Vin_bipolar			//   Out_rated 	= +/- 10 V
#define TRANSDUCER_GAIN				(TRANSDUCER_INPUT_RATED/TRANSDUCER_OUTPUT_RATED)
#define HRADC_R_BURDEN				20.0				// Resistor Burden = 20 R
															// This value makes Vin and Iin gain equal
extern void main_Jiga_HRADC_v2_1(void);

#endif
