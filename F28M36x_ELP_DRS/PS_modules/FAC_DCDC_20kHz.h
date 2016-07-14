#ifndef FAC_DCDC_20KHZ_H
#define FAC_DCDC_20KHZ_H

/*
 * Especificações da fonte
 */

#define FONTE_MODO		OnePS4Q				// Topologia da fonte a ser controlada
#define	n_PS			1

#define PWM_FREQ	   			10240		// Frequencia do sinal PWM gerado [Hz]
#define PWM_DEAD_TIME			4000		// Dead-time dos sinais PWM [ns]
#define PWM_MAX_DUTY    		0.9         // Maximo ciclo de trabalho para o PWM (em p.u.)
#define PWM_MIN_DUTY    		-0.9		// Minimo ciclo de trabalho para o PWM (em p.u.)
#define PWM_MAX_DUTY_OL			0.2			// Maximo ciclo de trabalho para o PWM em malha aberta (em p.u.)
#define PWM_MIN_DUTY_OL			-0.2		// Minimo ciclo de trabalho para o PWM em malha aberta (em p.u.)

#define MAX_REF					550.0		// Valor máximo da referência de corrente na carga [A]
#define MIN_REF					-550.0		// Valor mínimo da referência de corrente na carga [A]
#define MAX_LOAD				560.0		// Limite de corrente na carga para interlock [A]

#define MAX_REF_SLEWRATE		50.0		// Slew-rate máximo [A/s]
#define MAX_SR_SIGGEN_OFFSET	50.0		// Slew-rate máximo do offset do gerador senoidal [A/s]
#define MAX_SR_SIGGEN_AMP		100.0		// Slew-rate máximo da amplitude do gerador senoidal [A/s]

//#define KP						7.819075048934596e-01
//#define KI						2.150245638457013

#define KP						0.193
#define KI						0.175

#define CONTROL_FREQ			(2.0*PWM_FREQ)
#define CONTROL_PERIOD			(1.0/CONTROL_FREQ)
#define DECIMATION_FACTOR		6
#define TRANSFER_BUFFER_SIZE	DECIMATION_FACTOR

#define PWM_DAC_FREQ			(100.0*CONTROL_FREQ)
#define PWM_DAC_MODULE			&EPwm4Regs

#define	BUFFER_DECIMATION		40

/*#define TRANSDUCER_0_INPUT_RATED	600.0			// 		 HITEC Macc Plus:
#define TRANSDUCER_0_OUTPUT_RATED	0.6				//   In_rated 	= +/- 600 A
#define TRANSDUCER_0_OUTPUT_TYPE	Iin_bipolar		//   Out_rated 	= +/- 600 mA
#define TRANSDUCER_0_GAIN			TRANSDUCER_0_INPUT_RATED/TRANSDUCER_0_OUTPUT_RATED*/

#define TRANSDUCER_0_INPUT_RATED	1200.0			// DCCT EU-Kontroll ZCT:
#define TRANSDUCER_0_OUTPUT_RATED	10.0			//   In_rated 	= +/- 1200 A
#define TRANSDUCER_0_OUTPUT_TYPE	Vin_bipolar		//   Out_rated 	= +/- 10 V
#define TRANSDUCER_0_GAIN			TRANSDUCER_0_INPUT_RATED/TRANSDUCER_0_OUTPUT_RATED
#define HRADC_0_R_BURDEN			1.0				// Resistor Burden = 1 R

#define HRADC_R_BURDEN				1.0				// Resistor Burden = 1 R
#define HRADC_VIN_BI_P_GAIN			(20.0/262144.0)
#define HRADC_IIN_BI_P_GAIN			(1.0/(HRADC_R_BURDEN * 131072.0))

extern void main_FAC_DCDC_20kHz(void);

#endif
