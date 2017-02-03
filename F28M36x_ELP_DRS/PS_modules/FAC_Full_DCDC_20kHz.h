#ifndef FAC_FULL_DCDC_20KHZ_H
#define FAC_FULL_DCDC_20KHZ_H

/*
 * Especifica��es da fonte
 */

#define FONTE_MODO		OnePS4Q				// Topologia da fonte a ser controlada
#define	n_PS			1

#define PWM_FREQ	   			10240		// Frequencia do sinal PWM gerado [Hz]
#define PWM_DEAD_TIME			4000		// Dead-time dos sinais PWM [ns]
#define PWM_MAX_DUTY    		0.9         // Maximo ciclo de trabalho para o PWM (em p.u.)
#define PWM_MIN_DUTY    		-0.9		// Minimo ciclo de trabalho para o PWM (em p.u.)
#define PWM_MAX_DUTY_OL			0.2			// Maximo ciclo de trabalho para o PWM em malha aberta (em p.u.)
#define PWM_MIN_DUTY_OL			-0.2		// Minimo ciclo de trabalho para o PWM em malha aberta (em p.u.)
#define PWM_MAX_SHARE_DUTY		0.01

#define MAX_REF					1100.0		// Valor m�ximo da refer�ncia de corrente na carga [A]
#define MIN_REF					-1100.0		// Valor m�nimo da refer�ncia de corrente na carga [A]
#define MAX_ILOAD_MEASURED		1110.0		// Limite de corrente na carga para interlock [A]

#define MAX_DCLINK				105.0		// Valor m�ximo de tens�o no DC-Link para interlock [V]
#define MIN_DCLINK				0.0			// Valor m�nimo de tens�o no DC-Link para feed-forward operar [V]
#define NOM_VDCLINK				90.0		// Valor nominal da tens�o de DC-Link [V]

#define MAX_REF_SLEWRATE		50.0		// Slew-rate m�ximo [A/s]
#define MAX_SR_SIGGEN_OFFSET	50.0		// Slew-rate m�ximo do offset do gerador senoidal [A/s]
#define MAX_SR_SIGGEN_AMP		100.0		// Slew-rate m�ximo da amplitude do gerador senoidal [A/s]

#define KP_ILOAD				0.040			// iLoad Kp coeff
#define KI_ILOAD				0.100			// iLoad Ki coeff

#define KP_ISHARE				0.00001		// iShare Kp coeff
#define KI_ISHARE				0.0001024	// iShare Ki coeff

#define CONTROL_FREQ			(2.0*PWM_FREQ)
#define CONTROL_PERIOD			(1.0/CONTROL_FREQ)
#define ISHARE_DECIMATION		20

#define DECIMATION_FACTOR		6
#define TRANSFER_BUFFER_SIZE	DECIMATION_FACTOR
#define HRADC_FREQ_SAMP			(float) CONTROL_FREQ*DECIMATION_FACTOR
#define HRADC_SPI_CLK			SPI_10_71MHz

#define	BUFFER_DECIMATION		10

#define PWM_DAC_FREQ			(100.0*CONTROL_FREQ)
#define PWM_DAC_MODULE			&EPwm4Regs

#define TRANSDUCER_0_INPUT_RATED	1200.0			// DCCT EU-Kontroll ZCT:
#define TRANSDUCER_0_OUTPUT_RATED	10.0			//   In_rated 	= +/- 1200 A
#define TRANSDUCER_0_OUTPUT_TYPE	Vin_bipolar		//   Out_rated 	= +/- 10 V
#define TRANSDUCER_0_GAIN			TRANSDUCER_0_INPUT_RATED/TRANSDUCER_0_OUTPUT_RATED
#define HRADC_0_R_BURDEN			1.0				// Resistor Burden = 1 R

#define TRANSDUCER_1_INPUT_RATED	1200.0			// Verivolt IsoBlock + Divisor resistivo
#define TRANSDUCER_1_OUTPUT_RATED	10.0			//   In_rated 	= +/- 1200 V
#define TRANSDUCER_1_OUTPUT_TYPE	Vin_bipolar		//   Out_rated 	= +/- 10 V
#define TRANSDUCER_1_GAIN			TRANSDUCER_0_INPUT_RATED/TRANSDUCER_0_OUTPUT_RATED
#define HRADC_1_R_BURDEN			1.0				// Resistor Burden = 1 R

#define TRANSDUCER_2_INPUT_RATED	1200.0			// Verivolt IsoBlock + Divisor resistivo
#define TRANSDUCER_2_OUTPUT_RATED	10.0			//   In_rated 	= +/- 1200 V
#define TRANSDUCER_2_OUTPUT_TYPE	Vin_bipolar		//   Out_rated 	= +/- 10 V
#define TRANSDUCER_2_GAIN			TRANSDUCER_0_INPUT_RATED/TRANSDUCER_0_OUTPUT_RATED
#define HRADC_2_R_BURDEN			1.0				// Resistor Burden = 1 R

/*#define HRADC_R_BURDEN				1.0				// Resistor Burden = 1 R
#define HRADC_VIN_BI_P_GAIN			(20.0/262144.0)
#define HRADC_IIN_BI_P_GAIN			(1.0/(HRADC_R_BURDEN * 131072.0))*/

/*
 * DP modules mnemonics
 */

#define SRLIM_ILOAD_REFERENCE 		&DP_Framework.DPlibrary.ELP_SRLim[0]

#define ERROR_CALCULATOR			&DP_Framework.DPlibrary.ELP_Error[0]
#define ISHARE_ERROR_CALCULATOR		&DP_Framework.DPlibrary.ELP_Error[1]

#define	PI_DAWU_CONTROLLER_ILOAD	&DP_Framework.DPlibrary.ELP_PI_dawu[0]
#define	PI_DAWU_CONTROLLER_ISHARE	&DP_Framework.DPlibrary.ELP_PI_dawu[1]

#define IIR_2P2Z_FF_REFERENCE		&DP_Framework.DPlibrary.ELP_IIR_2P2Z[0]

#define IIR_2P2Z_LPF_VDCLINK_MOD1	&DP_Framework.DPlibrary.ELP_IIR_2P2Z[1]
#define IIR_2P2Z_LPF_VDCLINK_MOD2	&DP_Framework.DPlibrary.ELP_IIR_2P2Z[2]

#define FF_VDCLINK_MOD1				&DP_Framework.DPlibrary.ELP_DCLink_FF[0]
#define FF_VDCLINK_MOD2				&DP_Framework.DPlibrary.ELP_DCLink_FF[1]

#define SRLIM_SIGGEN_AMP	 		&DP_Framework.DPlibrary.ELP_SRLim[1]
#define SRLIM_SIGGEN_OFFSET 		&DP_Framework.DPlibrary.ELP_SRLim[2]

/*
 * Digital IO's defines
 */

#define PIN_STATUS_ACDC_INTERLOCK	!(GpioDataRegs.GPDDAT.bit.GPIO126) 		// GPDI1: AC/DC Interlock

#define PIN_SET_DCDC_INTERLOCK		GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;	// GPDO1: DC/DC Interlock
#define PIN_CLEAR_DCDC_INTERLOCK	GpioDataRegs.GPCSET.bit.GPIO67 = 1;

/*
 * Functions prototypes
 */

extern void main_FAC_Full_DCDC_20kHz(void);

#endif
