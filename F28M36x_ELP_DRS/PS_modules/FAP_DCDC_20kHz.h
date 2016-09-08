#ifndef FAP_DCDC_20KHZ_H
#define FAP_DCDC_20KHZ_H

/*
 * Especifica��es da fonte
 */

#define FONTE_MODO		OnePS1Q				// Topologia da fonte a ser controlada
#define	n_PS			1

#define PWM_FREQ	   			20000.0		// Frequencia do sinal PWM gerado [Hz]
#define PWM_DEAD_TIME			0.0			// Dead-time dos sinais PWM [ns]
#define PWM_MAX_DUTY    		0.9         // Maximo ciclo de trabalho para o PWM (em p.u.)
#define PWM_MIN_DUTY    		0.0			// Minimo ciclo de trabalho para o PWM (em p.u.)
#define PWM_MAX_DUTY_OL			0.4			// Maximo ciclo de trabalho para o PWM em malha aberta (em p.u.)
#define PWM_MIN_DUTY_OL			0.0			// Minimo ciclo de trabalho para o PWM em malha aberta (em p.u.)

#define MAX_REF					200.0		// Valor m�ximo da refer�ncia de corrente na carga [A]
#define MIN_REF					0.0			// Valor m�nimo da refer�ncia de corrente na carga [A]
#define MAX_LOAD				210.0		// Limite de corrente na carga para interlock [A]

#define MAX_DCLINK				110.0		// Valor m�ximo de tens�o no DC-Link para interlock [V]
#define MIN_DCLINK				50.0		// Valor m�nimo de tens�o no DC-Link para interlock e para feed-forward operar [V]
#define NOM_VDCLINK				100.0		// Valor nominal da tens�o de DC-Link [V]

#define MAX_REF_SLEWRATE		20.0		// Slew-rate m�ximo [A/s]
#define MAX_SR_SIGGEN_OFFSET	20.0		// Slew-rate m�ximo do offset do gerador senoidal [A/s]
#define MAX_SR_SIGGEN_AMP		20.0		// Slew-rate m�ximo da amplitude do gerador senoidal [A/s]

#define PWM_MAX_SHARE_DUTY		0.01

#define KP						0.4524
#define KI						3.1416

#define KP2						0.00001
#define KI2						0.0027

#define CONTROL_FREQ			(2.0*PWM_FREQ)
#define CONTROL_PERIOD			(1.0/CONTROL_FREQ)
#define DECIMATION_FACTOR		4
#define TRANSFER_BUFFER_SIZE	DECIMATION_FACTOR
#define HRADC_FREQ_SAMP			(float) CONTROL_FREQ*DECIMATION_FACTOR

#define PWM_DAC_FREQ			(100.0*CONTROL_FREQ)
#define PWM_DAC_MODULE			&EPwm4Regs

#define	BUFFER_DECIMATION		1

#define TRANSDUCER_0_INPUT_RATED	250.0			// DCCT Holec Zero-Flux C.T. 300:
#define TRANSDUCER_0_OUTPUT_RATED	10.0			//   In_rated 	= +/- 250 A
#define TRANSDUCER_0_OUTPUT_TYPE	Vin_bipolar		//   Out_rated 	= +/- 10 V
#define TRANSDUCER_0_GAIN			TRANSDUCER_0_INPUT_RATED/TRANSDUCER_0_OUTPUT_RATED
#define HRADC_0_R_BURDEN			1.0				// Resistor Burden = 1 R

#define TRANSDUCER_1_INPUT_RATED	110.0			// Divisor de Tens�o + Isolador Verivolt IsoBlock V-4:
#define TRANSDUCER_1_OUTPUT_RATED	10.0			//   In_rated 	= +/- 110.0 V
#define TRANSDUCER_1_OUTPUT_TYPE	Vin_bipolar		//   Out_rated 	= +/- 10 V
#define TRANSDUCER_1_GAIN			TRANSDUCER_1_INPUT_RATED/TRANSDUCER_1_OUTPUT_RATED
#define HRADC_1_R_BURDEN			1.0				// Resistor Burden = 1 R

/*#define HRADC_R_BURDEN				1.0				// Resistor Burden = 1 R
#define HRADC_VIN_BI_P_GAIN			(20.0/262144.0)
#define HRADC_IIN_BI_P_GAIN			(1.0/(HRADC_R_BURDEN * 131072.0))*/

extern void main_FAP_DCDC_20kHz(void);

#endif
