#ifndef FAP_6U_DCDC_20KHZ_H
#define FAP_6U_DCDC_20KHZ_H

/*
 * Especificacoes da fonte
 */

#define FONTE_MODO		OnePS1Q				// Topologia da fonte a ser controlada
#define	n_PS			1

#define PWM_FREQ	   			20000.0		// Frequencia do sinal PWM gerado [Hz]
#define PWM_DEAD_TIME			0.0			// Dead-time dos sinais PWM [ns]
#define PWM_MAX_DUTY    		0.8         // Maximo ciclo de trabalho para o PWM (em p.u.)
#define PWM_MIN_DUTY    		0.0			// Minimo ciclo de trabalho para o PWM (em p.u.)
#define PWM_MAX_DUTY_OL			0.8			// Maximo ciclo de trabalho para o PWM em malha aberta (em p.u.)
#define PWM_MIN_DUTY_OL			0.0			// Minimo ciclo de trabalho para o PWM em malha aberta (em p.u.)

#define MAX_REF					200.0		// Valor maximo da referencia de corrente na carga [A]
#define MIN_REF					0.0			// Valor minimo da referencia de corrente na carga [A]
#define MAX_LOAD				210.0		// Limite de corrente na carga para interlock [A]
#define MAX_IMOD				130.0		// Limite de corrente nos módulos IGBT para interlock [A]

#define MAX_DCLINK				109.0		// Valor maximo de tensao no DC-Link para interlock [V]
#define MIN_DCLINK				50.0		// Valor minimo de tensao no DC-Link para interlock e para feed-forward operar [V]
#define NOM_VDCLINK				100.0		// Valor nominal da tensao de DC-Link [V]

#define MAX_REF_SLEWRATE		5.0		// Slew-rate maximo [A/s]
#define MAX_SR_SIGGEN_OFFSET	5.0		// Slew-rate maximo do offset do gerador senoidal [A/s]
#define MAX_SR_SIGGEN_AMP		5.0		// Slew-rate maximo da amplitude do gerador senoidal [A/s]

#define PWM_MAX_SHARE_DUTY		0.0015

//#define KP						0.4524		// PI  para carga de teste (Lo = 36 mH/  Ro = 250 mR)
//#define KI						3.1416		// fbw = 200 Hz / VDC-link = 100 V

#define KP						0.2513		// PI  para carga UVX (Lo = 20 mH/  Ro = 250 mR)
#define KI						3.1416		// fbw = 200 Hz / VDC-link = 100 V

#define KP2						0.00001
#define KI2						0.0027

#define CONTROL_FREQ			PWM_FREQ//(2.0*PWM_FREQ)
#define CONTROL_PERIOD			(1.0/CONTROL_FREQ)
#define DECIMATION_FACTOR		4
#define TRANSFER_BUFFER_SIZE	DECIMATION_FACTOR
#define HRADC_FREQ_SAMP			(float) CONTROL_FREQ*DECIMATION_FACTOR
#define HRADC_SPI_CLK			SPI_15MHz

#define	BUFFER_DECIMATION		1

#define TRANSDUCER_0_INPUT_RATED	320.0			// DCCT LEM ITZ-600 + DB-25 Adapter Ip = 320A
#define TRANSDUCER_0_OUTPUT_RATED	10.0			//   In_rated 	= +/- 320 A
#define TRANSDUCER_0_OUTPUT_TYPE	Vin_bipolar		//   Out_rated 	= +/- 10 V
#define TRANSDUCER_0_GAIN			TRANSDUCER_0_INPUT_RATED/TRANSDUCER_0_OUTPUT_RATED
#define HRADC_0_R_BURDEN			20.0				// Resistor Burden = 20 R
#define HRADC_0_GAIN_ERROR			1.0
#define HRADC_0_OFFSET_ERROR		0.0

#define TRANSDUCER_1_INPUT_RATED	110.0			// Divisor de Tensao + Isolador Verivolt IsoBlock V-4:
#define TRANSDUCER_1_OUTPUT_RATED	10.0			//   In_rated 	= +/- 110.0 V
#define TRANSDUCER_1_OUTPUT_TYPE	Vin_bipolar		//   Out_rated 	= +/- 10 V
#define TRANSDUCER_1_GAIN			TRANSDUCER_1_INPUT_RATED/TRANSDUCER_1_OUTPUT_RATED
#define HRADC_1_R_BURDEN			20.0				// Resistor Burden = 20 R

/*
 * DP modules mnemonics
 */

#define SRLIM_ILOAD_REFERENCE 		&DP_Framework.DPlibrary.ELP_SRLim[0]
#define ERROR_CALCULATOR			&DP_Framework.DPlibrary.ELP_Error[0]
#define ISHARE_ERROR_CALCULATOR		&DP_Framework.DPlibrary.ELP_Error[1]
#define	PI_DAWU_CONTROLLER_ILOAD	&DP_Framework.DPlibrary.ELP_PI_dawu[0]
#define	PI_DAWU_CONTROLLER_ISHARE	&DP_Framework.DPlibrary.ELP_PI_dawu[1]

#define ISHARE_CONTROL_FREQ			1000.0

#define IIR_2P2Z_LPF_VDCLINK		&DP_Framework.DPlibrary.ELP_IIR_2P2Z[0]
#define FF_DCLINK_ILOAD				&DP_Framework.DPlibrary.ELP_DCLink_FF[0]
#define FF_DCLINK_ISHARE			&DP_Framework.DPlibrary.ELP_DCLink_FF[1]

#define SRLIM_SIGGEN_AMP	 		&DP_Framework.DPlibrary.ELP_SRLim[1]
#define SRLIM_SIGGEN_OFFSET 		&DP_Framework.DPlibrary.ELP_SRLim[2]

/*
 * Timeouts
 */

#define TIMEOUT_uS_DCLINK_CONTACTOR		3000000

/*
 * Digital IO's defines
 */

#define MAX_ILOAD_MEASURED				DP_Framework.NetSignals[18]

#define PIN_STATUS_DCLINK_CONTACTOR		GpioDataRegs.GPDDAT.bit.GPIO126
#define PIN_OPEN_DCLINK_CONTACTOR		GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;
#define PIN_CLOSE_DCLINK_CONTACTOR		GpioDataRegs.GPCSET.bit.GPIO67 = 1;

#define PIN_STATUS_DCCT_ILOAD			GpioDataRegs.GPDDAT.bit.GPIO125



extern void main_FAP_6U_DCDC_20kHz(void);

#endif
