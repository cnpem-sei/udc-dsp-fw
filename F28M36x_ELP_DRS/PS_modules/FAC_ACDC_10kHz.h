#ifndef FAC_ACDC_10KHZ_H
#define FAC_ACDC_10KHZ_H

/*
 * Especificações da fonte
 */

#define FONTE_MODO		OnePS1Q				// Topologia da fonte a ser controlada
#define	n_PS			1

#define PWM_FREQ	   		10240.0		// Frequencia do sinal PWM gerado [Hz]
#define PWM_DEAD_TIME		0			// Dead-time dos sinais PWM [ns]
#define PWM_MAX_DUTY    	0.9         // Maximo ciclo de trabalho para o PWM (em p.u.)
#define PWM_MIN_DUTY    	0			// Minimo ciclo de trabalho para o PWM (em p.u.)
#define PWM_MAX_DUTY_OL		0.8			// Maximo ciclo de trabalho para o PWM em malha aberta (em p.u.)
#define PWM_MIN_DUTY_OL		0			// Minimo ciclo de trabalho para o PWM em malha aberta (em p.u.)

#define MAX_REF				90.0		// Valor máximo da referência de tensão no banco de capacitores [V]
#define MIN_REF				0.0			// Valor máximo da referência de tensão no banco de capacitores [V]
#define MAX_LOAD			101.0		// Limite de tensão no banco de capacitores para interlock [V]
#define MAX_IIN_REF			170.0		// Limite da referência de corrente máxima na entrada do regulador da tensão no banco de capacitores [A]
#define MIN_IIN_REF			0.0			// Limite da referência de corrente mínima na entrada do regulador da tensão no banco de capacitores [A]
#define MAX_IIN				180.0		// Limite de corrente máxima na entrada para interlock [A]

#define MAX_REF_SLEWRATE		9.0		// Slew-rate máximo [V/s]
#define MAX_SR_SIGGEN_OFFSET	9.0		// Slew-rate máximo do offset do gerador senoidal [V/s]
#define MAX_SR_SIGGEN_AMP		9.0		// Slew-rate máximo da amplitude do gerador senoidal [V/s]

#define KP						0.0
#define KI						0.0

#define CONTROL_FREQ			PWM_FREQ
#define CONTROL_PERIOD			(1.0/CONTROL_FREQ)
#define V_CONTROL_DECIMATION	10

#define PWM_DAC_FREQ			(200.0*CONTROL_FREQ)
#define PWM_DAC_MODULE			&EPwm4Regs

#define	BUFFER_DECIMATION		5

#define NF_ALPHA				0.99

#define DECIMATION_FACTOR		4
#define TRANSFER_BUFFER_SIZE	DECIMATION_FACTOR

#define TRANSDUCER_0_INPUT_RATED		110.0				// Divisor de Tensão + Isolador Verivolt IsoBlock V-4:
#define TRANSDUCER_0_OUTPUT_RATED		10.0				//   In_rated 	= +/- 110 V
#define TRANSDUCER_0_OUTPUT_TYPE		Vin_bipolar			//   Out_rated 	= +/- 10 V
#define TRANSDUCER_0_GAIN				(TRANSDUCER_0_INPUT_RATED/TRANSDUCER_0_OUTPUT_RATED)

#define TRANSDUCER_1_INPUT_RATED		200.0				// LEM LF 210-S
#define TRANSDUCER_1_OUTPUT_RATED		-0.1				//   In_rated 	= +/- 200 A
#define TRANSDUCER_1_OUTPUT_TYPE		Iin_bipolar			//   Out_rated 	= +/- 100 mA
#define TRANSDUCER_1_GAIN				(TRANSDUCER_1_INPUT_RATED/TRANSDUCER_1_OUTPUT_RATED)

#define HRADC_R_BURDEN				5.0					// Resistor Burden = 5 R
#define HRADC_VIN_BI_P_GAIN			(20.0/262144.0)
#define HRADC_IIN_BI_P_GAIN			(1.0/(HRADC_R_BURDEN * 131072.0))

extern void main_FAC_ACDC_10kHz(void);

#endif
