#ifndef FAP_DCDC_20KHZ_H
#define FAP_DCDC_20KHZ_H

/*
 * Especifica��es da fonte
 */

#define FONTE_MODO		OnePS1Q				// Topologia da fonte a ser controlada
#define	n_PS			1

#define PWM_FREQ	   			15000.0		// Frequencia do sinal PWM gerado [Hz]
#define PWM_DEAD_TIME			0.0			// Dead-time dos sinais PWM [ns]
#define PWM_MAX_DUTY    		0.9         // Maximo ciclo de trabalho para o PWM (em p.u.)
#define PWM_MIN_DUTY    		0.0			// Minimo ciclo de trabalho para o PWM (em p.u.)
#define PWM_MAX_DUTY_OL			0.4			// Maximo ciclo de trabalho para o PWM em malha aberta (em p.u.)
#define PWM_MIN_DUTY_OL			0.0			// Minimo ciclo de trabalho para o PWM em malha aberta (em p.u.)

#define MAX_REF					225.0		// Valor m�ximo da refer�ncia de corrente na carga [A]
#define MIN_REF					0.0			// Valor m�nimo da refer�ncia de corrente na carga [A]
#define MAX_LOAD				235.0		// Limite de corrente na carga para interlock [A]

#define MAX_REF_SLEWRATE		90.0		// Slew-rate m�ximo [A/s]
#define MAX_SR_SIGGEN_OFFSET	90.0		// Slew-rate m�ximo do offset do gerador senoidal [A/s]
#define MAX_SR_SIGGEN_AMP		90.0		// Slew-rate m�ximo da amplitude do gerador senoidal [A/s]

#define ISHARE_CONTROL_FREQ		1000.0
#define PWM_MAX_SHARE_DUTY		0.01

#define KP_ILOAD				0.4524
#define KI_ILOAD				3.1416

#define KP_ISHARE				0.00001
#define KI_ISHARE				0.0027

#define CONTROL_FREQ			(2.0*PWM_FREQ)
#define CONTROL_PERIOD			(1.0/CONTROL_FREQ)
#define DECIMATION_FACTOR		6
#define TRANSFER_BUFFER_SIZE	DECIMATION_FACTOR
#define HRADC_FREQ_SAMP			(float) CONTROL_FREQ*DECIMATION_FACTOR
#define HRADC_SPI_CLK			SPI_15MHz

#define PWM_DAC_FREQ			(100.0*CONTROL_FREQ)
#define PWM_DAC_MODULE			&EPwm4Regs

#define	BUFFER_DECIMATION		1

#define TRANSDUCER_0_INPUT_RATED	800.0/3.0		// DCCT FOELDI MPT 80-10C:
#define TRANSDUCER_0_OUTPUT_RATED	10.0			//   In_rated 	= +/- 266.6666 A (800 A with 3-turns on DCCT head)
#define TRANSDUCER_0_OUTPUT_TYPE	Vin_bipolar		//   Out_rated 	= +/- 10 V
#define TRANSDUCER_0_GAIN			TRANSDUCER_0_INPUT_RATED/TRANSDUCER_0_OUTPUT_RATED
#define HRADC_0_R_BURDEN			10.0				// Resistor Burden = 10 R

/*
 * DP modules mnemonics
 */

#define SRLIM_ILOAD_REFERENCE 		&DP_Framework.DPlibrary.ELP_SRLim[0]
#define ERROR_CALCULATOR			&DP_Framework.DPlibrary.ELP_Error[0]
#define ISHARE_ERROR_CALCULATOR		&DP_Framework.DPlibrary.ELP_Error[1]
#define	PI_DAWU_CONTROLLER_ILOAD	&DP_Framework.DPlibrary.ELP_PI_dawu[0]
#define	PI_DAWU_CONTROLLER_ISHARE	&DP_Framework.DPlibrary.ELP_PI_dawu[1]

#define SRLIM_SIGGEN_AMP	 		&DP_Framework.DPlibrary.ELP_SRLim[1]
#define SRLIM_SIGGEN_OFFSET 		&DP_Framework.DPlibrary.ELP_SRLim[2]

/*
 * Timeouts
 */

#define TIMEOUT_uS_AC_CONTACTOR		3000000

/*
 * Digital IO's defines
 */

#define PIN_STATUS_AC_CONTACTOR 	GpioDataRegs.GPDDAT.bit.GPIO126			// GPDI1

#define PIN_OPEN_AC_CONTACTOR		GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;	// GPDO1
#define PIN_CLOSE_AC_CONTACTOR		GpioDataRegs.GPCSET.bit.GPIO67 = 1;

/*
 * Functions prototypes
 */

extern void main_FAP_DCDC_15kHz_225A(void);

#endif
