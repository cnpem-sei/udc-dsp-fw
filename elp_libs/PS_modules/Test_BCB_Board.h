#ifndef TEST_BCB_BOARD_H
#define TEST_BCB_BOARD_H

/*
 * Test specifications
 */

#define PWM_FREQ					20000				// PWM Master frequency [Hz]
#define HRADC_FREQ_SAMP	   			20000				// Sampling frequency [Hz]
#define DECIMATION_FACTOR			1
#define HRADC_SPI_CLK				SPI_15MHz
#define TRANSFER_BUFFER_SIZE		DECIMATION_FACTOR
#define	N_HRADC_BOARDS				3

#define TRANSDUCER_0_INPUT_RATED	10.0				//
#define TRANSDUCER_0_OUTPUT_RATED	10.0				//   In_rated 	= +/- 10 V
#define TRANSDUCER_0_OUTPUT_TYPE	Vref_bipolar_p			//   Out_rated 	= +/- 10 V
#define TRANSDUCER_0_GAIN			(TRANSDUCER_0_INPUT_RATED/TRANSDUCER_0_OUTPUT_RATED)

#define HRADC_0_R_BURDEN			20.0				// Resistor Burden = 20 R
#define HRADC_0_GAIN_ERROR			1.0
#define HRADC_0_OFFSET_ERROR		0.0

#define TRANSDUCER_1_INPUT_RATED	10.0				//
#define TRANSDUCER_1_OUTPUT_RATED	10.0				//   In_rated 	= +/- 10 V
#define TRANSDUCER_1_OUTPUT_TYPE	Vref_bipolar_n			//   Out_rated 	= +/- 10 V
#define TRANSDUCER_1_GAIN			(TRANSDUCER_1_INPUT_RATED/TRANSDUCER_1_OUTPUT_RATED)

#define HRADC_1_R_BURDEN			20.0				// Resistor Burden = 20 R
#define HRADC_1_GAIN_ERROR			1.0
#define HRADC_1_OFFSET_ERROR		0.0

#define TRANSDUCER_2_INPUT_RATED	10.0				//
#define TRANSDUCER_2_OUTPUT_RATED	10.0				//   In_rated 	= +/- 10 V
#define TRANSDUCER_2_OUTPUT_TYPE	Vref_bipolar_p			//   Out_rated 	= +/- 10 V
#define TRANSDUCER_2_GAIN			(TRANSDUCER_2_INPUT_RATED/TRANSDUCER_2_OUTPUT_RATED)

#define HRADC_2_R_BURDEN			20.0				// Resistor Burden = 20 R
#define HRADC_2_GAIN_ERROR			1.0
#define HRADC_2_OFFSET_ERROR		0.0

#define TRANSDUCER_3_INPUT_RATED	10.0				//
#define TRANSDUCER_3_OUTPUT_RATED	10.0				//   In_rated 	= +/- 10 V
#define TRANSDUCER_3_OUTPUT_TYPE	Vref_bipolar_n			//   Out_rated 	= +/- 10 V
#define TRANSDUCER_3_GAIN			(TRANSDUCER_3_INPUT_RATED/TRANSDUCER_3_OUTPUT_RATED)

#define HRADC_3_R_BURDEN			20.0				// Resistor Burden = 20 R
#define HRADC_3_GAIN_ERROR			1.0
#define HRADC_3_OFFSET_ERROR		0.0

// Define the CPU frequency in MHz
// Define the SCI frequency in bps
// Low-speed clock, LSPCLK, is set to CPU_FREQ/4
// Calculate BRR: 16-bit baud rate register value
#define SCI_FREQ     6000000
#define LSPCLK_FREQ  C28_FREQ_MHZ*1e6/(LSPCLK_DV+1)
#define SCI_PRD     (LSPCLK_FREQ/(SCI_FREQ*8))-1

extern void main_Test_BCB_Board(void);

#endif
