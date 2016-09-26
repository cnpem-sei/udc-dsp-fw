#ifndef TEST_HRPWM_H
#define TEST_HRPWM_H

/*
 * Especificações do teste
 */

#define FONTE_MODO		OnePS1Q				// Topologia da fonte a ser controlada
#define	n_PS			1

#define PWM_FREQ	   			100000		// Frequencia do sinal PWM gerado [Hz]
#define PWM_DEAD_TIME			300			// Dead-time dos sinais PWM [ns]
#define PWM_MAX_DUTY    		0.95		// Maximo ciclo de trabalho para o PWM (em p.u.)
#define PWM_MIN_DUTY    		-0.95		// Minimo ciclo de trabalho para o PWM (em p.u.)
#define PWM_MAX_DUTY_OL			0.95		// Maximo ciclo de trabalho para o PWM em malha aberta (em p.u.)
#define PWM_MIN_DUTY_OL			-0.95		// Minimo ciclo de trabalho para o PWM em malha aberta (em p.u.)

#define MAX_REF					10.0		// Valor máximo da referência de corrente na carga [A]
#define MIN_REF					-10.0		// Valor mínimo da referência de corrente na carga [A]
#define MAX_LOAD				1000000.5		// Limite de corrente na carga para interlock [A]

#define CONTROL_FREQ			(2.0*PWM_FREQ)
#define CONTROL_PERIOD			(1.0/CONTROL_FREQ)
#define DECIMATION_FACTOR		5
#define TRANSFER_BUFFER_SIZE	DECIMATION_FACTOR
#define HRADC_FREQ_SAMP			(float) CONTROL_FREQ*DECIMATION_FACTOR
#define HRADC_SPI_CLK			SPI_15MHz

extern void main_Test_HRPWM(void);

#endif
