#ifndef FBP_FAC_UFJF_H
#define FBP_FAC_UFJF_H

/*
 * Especificações da fonte
 */

#define PWM_FREQ	   			48000.0		// Frequencia do sinal PWM gerado [Hz]
#define PWM_DEAD_TIME			300			// Dead-time dos sinais PWM [ns]
#define PWM_MAX_DUTY    		0.9         // Maximo ciclo de trabalho para o PWM (em p.u.)
#define PWM_MIN_DUTY    		-0.9		// Minimo ciclo de trabalho para o PWM (em p.u.)
#define PWM_MAX_DUTY_OL			0.9			// Maximo ciclo de trabalho para o PWM em malha aberta (em p.u.)
#define PWM_MIN_DUTY_OL			-0.9		// Minimo ciclo de trabalho para o PWM em malha aberta (em p.u.)

#define MAX_REF					10.0		// Valor máximo da referência de corrente na carga [A]
#define MIN_REF					-10.0		// Valor mínimo da referência de corrente na carga [A]
#define MAX_LOAD				10.5		// Limite de corrente na carga para interlock [A]
#define MAX_IMOD                10.5        // Limite de corrente nos módulos para interlock [A]

#define MAX_REF_SLEWRATE		10000000.0	// Slew-rate máximo (A/s)
#define MAX_SR_SIGGEN_OFFSET	50.0		// Slew-rate máximo do offset do gerador senoidal (A/s)
#define MAX_SR_SIGGEN_AMP		100.0		// Slew-rate máximo da amplitude do gerador senoidal (A/s)

#define CONTROL_FREQ			PWM_FREQ
#define CONTROL_PERIOD			(1.0/CONTROL_FREQ)
#define DECIMATION_FACTOR		1
#define TRANSFER_BUFFER_SIZE	DECIMATION_FACTOR
#define HRADC_FREQ_SAMP			(float) CONTROL_FREQ*DECIMATION_FACTOR
#define HRADC_SPI_CLK			SPI_15MHz

#define	BUFFER_DECIMATION		1
#define WFMREF_SAMPLING_FREQ    8000.0

#define N_HRADC                 3

#define TRANSDUCER_0_INPUT_RATED		12.5			// 			LEM ITN 12-P:
#define TRANSDUCER_0_OUTPUT_RATED		0.05			//   In_rated 	= +/- 12.5 A
#define TRANSDUCER_0_OUTPUT_TYPE		Iin_bipolar		//   Out_rated 	= +/- 50 mA
#define TRANSDUCER_0_GAIN				(TRANSDUCER_0_INPUT_RATED/TRANSDUCER_0_OUTPUT_RATED)
#define HRADC_0_R_BURDEN				20.0			// Resistor Burden = 20 R

#define TRANSDUCER_1_INPUT_RATED		12.5			// 			LEM ITN 12-P:
#define TRANSDUCER_1_OUTPUT_RATED		0.05			//   In_rated 	= +/- 12.5 A
#define TRANSDUCER_1_OUTPUT_TYPE		Iin_bipolar		//   Out_rated 	= +/- 50 mA
#define TRANSDUCER_1_GAIN				(TRANSDUCER_1_INPUT_RATED/TRANSDUCER_1_OUTPUT_RATED)
#define HRADC_1_R_BURDEN				20.0			// Resistor Burden = 20 R

#define TRANSDUCER_2_INPUT_RATED		150.0/7			// 	    PM Topacc 150 A
#define TRANSDUCER_2_OUTPUT_RATED		10.0			//   In_rated 	= +/- 150/7 A (7 voltas na cabeça)
#define TRANSDUCER_2_OUTPUT_TYPE		Vin_bipolar		//   Out_rated 	= +/- 50 mA
#define TRANSDUCER_2_GAIN				(TRANSDUCER_2_INPUT_RATED/TRANSDUCER_2_OUTPUT_RATED)
#define HRADC_2_R_BURDEN				20.0				// Resistor Burden = 20 R


/*
 * DP modules mnemonics
 */

//#define FULL_OBSERVER
#define REDUCED_OBSERVER

#ifdef  FULL_OBSERVER

#define REFERENCE               DP_Framework.NetSignals[15]

#define ILOAD                   DP_Framework.NetSignals[1]
#define Y                       ILOAD

#define IMOD1                   DP_Framework.NetSignals[11]
#define IMOD2                   DP_Framework.NetSignals[12]
#define IMOD3                   DP_Framework.NetSignals[13]
#define IMOD4                   DP_Framework.NetSignals[14]

#define VLOAD_MOD1              DP_Framework_MtoC.NetSignals[9] // ANI6
#define VDCLINK_MOD1            DP_Framework_MtoC.NetSignals[5] // ANI2

#define DUTY_MOD1               DP_Framework.DutySignals[0]
#define U1                      DP_Framework.NetSignals[2]

#define XMOD1                   DP_Framework.NetSignals[3]     // Estimated states
#define XMOD1_0                 DP_Framework.NetSignals[3]     // iLoad
#define XMOD1_1                 DP_Framework.NetSignals[4]     // iL_Mod1
#define XMOD1_2                 DP_Framework.NetSignals[5]     // vD
#define XMOD1_3                 DP_Framework.NetSignals[6]     // v_Mod1

#define XMOD1_FUT               DP_Framework.NetSignals[7]     // Estimated future states
#define XMOD1_0_FUT             DP_Framework.NetSignals[7]     // iLoad
#define XMOD1_1_FUT             DP_Framework.NetSignals[8]     // iL_Mod1
#define XMOD1_2_FUT             DP_Framework.NetSignals[9]     // vD
#define XMOD1_3_FUT             DP_Framework.NetSignals[10]    // v_Mod1

#define STATE_OBSERVER          &Observer
#define STATE_OBSERVER_IN       &DP_Framework.NetSignals[1]
#define STATE_OBSERVER_OUT      &DP_Framework.NetSignals[7]
#define NUM_ROWS_OBSERVER       4
#define NUM_COLUMNS_OBSERVER    6
#define NUM_INPUT_OUTPUT        9

#define TIMESLICER_WFMREF       0
#define TIMESLICER_BUFFER       1

#endif

#ifdef REDUCED_OBSERVER

#define REFERENCE               DP_Framework.NetSignals[10]
#define ILOAD                   DP_Framework.NetSignals[6]

#define VLOAD_MOD1_ARM          DP_Framework_MtoC.NetSignals[9] // ANI6
#define VLOAD_MOD1_DSP          DP_Framework.NetSignals[7]

#define IMOD1                   DP_Framework.NetSignals[11]
#define IMOD2                   DP_Framework.NetSignals[12]

#define X_A_MOD1                DP_Framework.NetSignals[6]      // Current measured states: x_a[k]
#define X_A_MOD1_0              DP_Framework.NetSignals[6]      // iLoad[k]
#define X_A_MOD1_1              DP_Framework.NetSignals[7]      // vLoad[k]

#define X_A_MOD1_PAST           DP_Framework.NetSignals[1]      // Past measured states: x_a[k-1]
#define X_A_MOD1_0_PAST         DP_Framework.NetSignals[1]      // iLoad[k-1]
#define X_A_MOD1_1_PAST         DP_Framework.NetSignals[2]      // vLoad[k-1]

#define DUTY_MOD1               DP_Framework.DutySignals[0]     // Current control input: u[k]
#define U_MOD1_PAST             DP_Framework.NetSignals[5]      // Past control input: u[k-1]

#define VDCLINK_MOD1            DP_Framework_MtoC.NetSignals[5] // ANI2

#define X_B_MOD1                DP_Framework.NetSignals[8]      // Current estimated states: x_b[k]
#define X_B_MOD1_0              DP_Framework.NetSignals[8]      // iL_Mod1[k]
#define X_B_MOD1_1              DP_Framework.NetSignals[9]      // vD[k]

#define X_B_MOD1_PAST           DP_Framework.NetSignals[3]     // Last estimated states: x_b[k-1]
#define X_B_MOD1_0_PAST         DP_Framework.NetSignals[3]     // iL_Mod1[k-1]
#define X_B_MOD1_1_PAST         DP_Framework.NetSignals[4]     // vD[k-1]

#define STATE_OBSERVER          &Observer
#define STATE_OBSERVER_IN       &DP_Framework.NetSignals[1]       // Input: NetSignals[1..7]
#define STATE_OBSERVER_OUT      &DP_Framework.NetSignals[8]       // Output: NetSignals[8..9]
#define NUM_ROWS_OBSERVER       2
#define NUM_COLUMNS_OBSERVER    7

#define STATE_CONTROLLER        &Controller
#define STATE_CONTROLLER_IN     &DP_Framework.NetSignals[6]       // Input: NetSignals[6..10]
#define STATE_CONTROLLER_OUT    &DUTY_MOD1                        // Output: DutySignals[0]
#define NUM_ROWS_CONTROLLER     1
#define NUM_COLUMNS_CONTROLLER  5

#define TIMESLICER_WFMREF       0
#define TIMESLICER_BUFFER       1

#endif


/*
 * Digital IO's defines
 */

#define PIN_OPEN_PS1_DCLINK_RELAY		GpioDataRegs.GPCCLEAR.bit.GPIO64 = 1;		// GPDO4: PS1_OUTPUT_CTRL (FBP v4.0)
#define PIN_CLOSE_PS1_DCLINK_RELAY		GpioDataRegs.GPCSET.bit.GPIO64 = 1;

#define PIN_OPEN_PS2_DCLINK_RELAY		GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;		// GPDO3: PS2_OUTPUT_CTRL (FBP v4.0)
#define PIN_CLOSE_PS2_DCLINK_RELAY		GpioDataRegs.GPCSET.bit.GPIO66 = 1;

#define PIN_OPEN_PS3_DCLINK_RELAY		GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;		// GPDO1: PS3_OUTPUT_CTRL (FBP v4.0)
#define PIN_CLOSE_PS3_DCLINK_RELAY		GpioDataRegs.GPCSET.bit.GPIO67 = 1;

#define PIN_OPEN_PS4_DCLINK_RELAY		GpioDataRegs.GPCCLEAR.bit.GPIO65 = 1;		// GPDO2: PS4_OUTPUT_CTRL (FBP v4.0)
#define PIN_CLOSE_PS4_DCLINK_RELAY		GpioDataRegs.GPCSET.bit.GPIO65 = 1;

extern void main_FBP_FAC_UFJF(void);

/*
 * PWM defines
 */
#define PWM_MOD1_A                      PWM_Modules.PWM_Regs[6]
#define PWM_MOD1_B                      PWM_Modules.PWM_Regs[7]

#define PWM_MOD2_A                      PWM_Modules.PWM_Regs[4]
#define PWM_MOD2_B                      PWM_Modules.PWM_Regs[5]

#define PWM_MOD3_A                      PWM_Modules.PWM_Regs[2]
#define PWM_MOD3_B                      PWM_Modules.PWM_Regs[3]

#define PWM_MOD4_A                      PWM_Modules.PWM_Regs[0]
#define PWM_MOD4_B                      PWM_Modules.PWM_Regs[1]

#endif
