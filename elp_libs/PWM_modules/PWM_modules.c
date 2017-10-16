/*
 * 		FILE: 		PWM_modules.h
 * 		PROJECT: 	DRS v2.0
 * 		CREATION:	01/26/2016
 * 		MODIFIED:	01/26/2016
 *
 * 		AUTHOR: 	Gabriel O. B.  (LNLS/ELP)
 *
 * 		DESCRIPTION:
 *		Source code for configuration and operation of 16 PWM channels
 *		(channels A and B from HRPWM1 to HRPWM8).
 *
 *		TODO: Verificar se configuração dos registradores HR está correta
 */

#include "PWM_modules.h"

/* Prototype statements for functions found within this file */

#pragma CODE_SECTION(SetPWMDutyCycle_ChA, "ramfuncs");
#pragma CODE_SECTION(SetPWMDutyCycle_ChB, "ramfuncs");
#pragma CODE_SECTION(SetPWMDutyCycle_HBridge, "ramfuncs");
#pragma CODE_SECTION(EnablePWMOutput, "ramfuncs");
#pragma CODE_SECTION(DisablePWMOutput, "ramfuncs");
#pragma CODE_SECTION(EnablePWMOutputs, "ramfuncs");
#pragma CODE_SECTION(DisablePWMOutputs, "ramfuncs");

Uint16 SetPWMFreq(volatile struct EPWM_REGS *pwmPtr, double freq);
void   SetPWMDeadTime(volatile struct EPWM_REGS *pwmPtr, Uint16 dt_ns);
void   SetPWMSyncPhase(volatile struct EPWM_REGS *pwmPtr, Uint16 degrees);
void   SetPWMCounterSync(volatile struct EPWM_REGS *pwmPtr, ePWMSync sync_mode, Uint16 phase_degrees);
void   SetPWMComplementary(volatile struct EPWM_REGS *pwmPtr, ePWMComp comp_mode);
void   LinkPWM_Modules(volatile struct EPWM_REGS *pwmPtr, Uint16 primary_module);

void SetPWMDutyCycle_ChA(volatile struct EPWM_REGS *pwmPtr, float duty_pu);
void SetPWMDutyCycle_ChB(volatile struct EPWM_REGS *pwmPtr, float duty_pu);
void SetPWMDutyCycle_HBridge(volatile struct EPWM_REGS *pwmPtr, float duty_pu);

void InitPWMModule(volatile struct EPWM_REGS *pwmPtr, double freq, Uint16 primary_module,
		           ePWMSync sync_mode, Uint16 phase_degrees, ePWMComp comp_mode, Uint16 dt_ns);

void InitPWMDAC(volatile struct EPWM_REGS *pwmPtr, double freq);

void EnablePWMOutput(Uint16 pwm_module);
void DisablePWMOutput(Uint16 pwm_module);
void EnablePWMOutputs(void);
void DisablePWMOutputs(void);

void EnablePWM_TBCLK(void);
void DisablePWM_TBCLK(void);

void EnablePWMInterrupt(volatile struct EPWM_REGS *pwmPtr);
void DisablePWMInterrupt(volatile struct EPWM_REGS *pwmPtr);

void InitPWM_MEP_SFO(void);
void TunningPWM_MEP_SFO(void);

/*
 * Global variables instantiation
 *
 * Structs gather all information regarding used HRADC boards.
 * Place them in shared memory RAMS1
 *
 *		 The following declarations are required in order to use
 *		 the SFO library functions:
 */

Uint16	SFO_status; 			/* HRPWM calibration status */
int 	MEP_ScaleFactor; 		/* Scale factor resulted from HRPWM calibration
 	 	 	 	 	 	 	 	 * Result can be used for all HRPWM channels. This variable
								 *  is also copied to HRMSTEP register by SFO() function. */

/* ePWM[0] is defined as dummy value not used in the example */

volatile struct EPWM_REGS *ePWM[9] = {  &EPwm1Regs,
										&EPwm1Regs,
										&EPwm2Regs,
										&EPwm3Regs, 
										&EPwm4Regs, 
										&EPwm5Regs, 
										&EPwm6Regs, 
										&EPwm7Regs, 
										&EPwm8Regs};

tPWMmodules PWM_Modules;		/* PWM module to access modulators registers */

/*
 * Set frequency for specified PWM module.
 * Also, returns period in system clocks (125/150 MHz)
 */

Uint16 SetPWMFreq(volatile struct EPWM_REGS *pwmPtr, double freq)
{
	Uint16 period;
	period = ((double) C28_FREQ_MHZ * (double) 1E6) / freq - 1;
	pwmPtr->TBPRD = period;
	return period;
}

/*
 * Set specified PWM module with given dead-time [nanoseconds]
 */

void SetPWMDeadTime(volatile struct EPWM_REGS *pwmPtr, Uint16 dt_ns)
{
	Uint16 dt_clk;
	dt_clk = ((float) ((Uint32) C28_FREQ_MHZ * dt_ns) * 1E-3);
	pwmPtr->DBFED = dt_clk;			// Falling-edge
    pwmPtr->DBRED = dt_clk;			// Rising-edge
}

/*
 * Set sync phase between specified PWM module and its master
 */

void SetPWMSyncPhase(volatile struct EPWM_REGS *pwmPtr, Uint16 degrees)
{
	Uint16 phase;
	phase = (float) degrees * ((float) pwmPtr->TBPRD / 360.0);
	pwmPtr->TBPHS.half.TBPHS = phase;
}

/*
 * Configure counter synchronization of specified PWM module as master or slave
 */
void SetPWMCounterSync(volatile struct EPWM_REGS *pwmPtr, ePWMSync sync_mode, Uint16 phase_degrees)
{
	Uint16 phase;

	switch(sync_mode)
	{
		case MasterPWM:
			pwmPtr->TBPHS.half.TBPHS = 0;
			pwmPtr->TBCTL.bit.PHSEN = TB_DISABLE;
			pwmPtr->TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
			break;

		case SlavePWM:
			//phase = (float) phase_degrees * ((float) pwmPtr->TBPRD / 360.0);
			phase = (360.0 - (float) phase_degrees) * ((float) pwmPtr->TBPRD /360.0);
			pwmPtr->TBPHS.half.TBPHS = phase;
			pwmPtr->TBCTL.bit.PHSEN = TB_ENABLE;
			pwmPtr->TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
			break;
	}
}

/*
 * Configure channel B as complementary from channel A of specified PWM module
 */

void SetPWMComplementary(volatile struct EPWM_REGS *pwmPtr, ePWMComp mode)
{
	if(mode)
	{
		// Channel B = NOT(Channel A)
		pwmPtr->DBCTL.bit.IN_MODE 	= DBA_ALL;
		pwmPtr->DBCTL.bit.POLSEL 	= DB_ACTV_HIC;
		pwmPtr->DBCTL.bit.OUT_MODE	= DB_FULL_ENABLE;	// Enable Dead-Band
	}
	else
	{
		// B independent from A
		pwmPtr->DBCTL.bit.IN_MODE 	= DBA_RED_DBB_FED;
		pwmPtr->DBCTL.bit.POLSEL 	= DB_ACTV_HI;
		pwmPtr->DBCTL.bit.OUT_MODE	= DB_DISABLE;		// Bypass Dead-Band
	}
}

/*
 * Link given PWM module with specified primary module.
 *
 * 		Each time the period or compare registers are updated
 *		in the primary module, the linked registers automatically
 *		get updated with the same value.
 *		To keep it independent, use primary_module = 0.
 */

void LinkPWM_Modules(volatile struct EPWM_REGS *pwmPtr, Uint16 primary_module)
{
	Uint16 aux = primary_module + 1;

    pwmPtr->EPWMXLINK.bit.TBPRDLINK = aux;
    pwmPtr->EPWMXLINK.bit.CMPALINK  = aux;
    pwmPtr->EPWMXLINK.bit.CMPBLINK  = aux;
}

/*
 * Set duty-cycle (0.0 to 1.0 ) from channel A of specified PWM module
 */
void SetPWMDutyCycle_ChA(volatile struct EPWM_REGS *pwmPtr, float duty_pu)
{
    Uint16 duty_int;
    Uint16 duty_frac;
    float duty;

    duty = duty_pu * (float)pwmPtr->TBPRD;

    duty_int  = (Uint16) duty;
    duty_frac = ((Uint16) ((duty - (float)duty_int) * MEP_ScaleFactor)) << 8;
    duty_frac += 0x0180;

    pwmPtr->CMPAM2.half.CMPA	= duty_int;
    pwmPtr->CMPAM2.half.CMPAHR	= duty_frac;
}

/*
 * Set duty-cycle (0.0 to 1.0) from channel B of specified PWM module
 */

void SetPWMDutyCycle_ChB(volatile struct EPWM_REGS *pwmPtr, float duty_pu)
{
    Uint16 duty_int;
    Uint16 duty_frac;
    float duty;

    duty = duty_pu * (float)pwmPtr->TBPRD;

    duty_int  = (Uint16) duty;
    duty_frac = ((Uint16) ((duty - (float)duty_int) * MEP_ScaleFactor)) << 8;
    duty_frac += 0x0180;

    pwmPtr->CMPBM.half.CMPB		= duty_int;
    pwmPtr->CMPBM.half.CMPBHR	= duty_frac;
}

/*
 * Set duty-cycle (-1.0 to 1.0) from specified PWM module for a H-bridge modulator
 *
 * 		This function only works for PWM modulators configured as COMPLEMENTARY
 */

void SetPWMDutyCycle_HBridge(volatile struct EPWM_REGS *pwmPtr, float duty_pu)
{
    Uint16 duty_int;
    Uint16 duty_frac;
    float duty;

    duty = (0.5 * duty_pu + 0.5) * (float)pwmPtr->TBPRD;

    duty_int  = (Uint16) duty;
    duty_frac = ((Uint16) ((duty - (float)duty_int) * MEP_ScaleFactor)) << 8;
    duty_frac += 0x0180;

    pwmPtr->CMPAM2.half.CMPA	= duty_int;
    pwmPtr->CMPAM2.half.CMPAHR	= duty_frac;
}

/*
 * Initialization of specified PWM module
 *
 * This function initializes both channels with digital counter
 * configured as a saw-tooth carrier waveform.
 */

void InitPWMModule(volatile struct EPWM_REGS *pwmPtr, double freq, Uint16 primary_module, ePWMSync sync_mode, Uint16 phase_degrees, ePWMComp comp_mode, Uint16 dt_ns)
{
	/* Counter-register configuration */
    pwmPtr->TBCTL.bit.HSPCLKDIV = TB_DIV1;
    pwmPtr->TBCTL.bit.CLKDIV = TB_DIV1;
    pwmPtr->TBCTL.bit.PRDLD = TB_IMMEDIATE;
   	pwmPtr->TBCTL.bit.CTRMODE = TB_COUNT_UP;
    pwmPtr->TBCTR = 0;

	/* Parametric configuration */
	SetPWMFreq(pwmPtr,freq);
	SetPWMDutyCycle_ChA(pwmPtr,0.0);
	SetPWMDutyCycle_ChB(pwmPtr,0.0);
    SetPWMCounterSync(pwmPtr, sync_mode, phase_degrees);
    SetPWMComplementary(pwmPtr, comp_mode);
    SetPWMDeadTime(pwmPtr, dt_ns);
    LinkPWM_Modules(pwmPtr, primary_module);

    /* Action-Qualifier configuration */
	pwmPtr->AQCTLA.bit.ZRO = AQ_SET;
	pwmPtr->AQCTLA.bit.PRD = AQ_NO_ACTION;
	pwmPtr->AQCTLA.bit.CAU = AQ_CLEAR;
	pwmPtr->AQCTLA.bit.CAD = AQ_NO_ACTION;
	pwmPtr->AQCTLA.bit.CBU = AQ_NO_ACTION;
	pwmPtr->AQCTLA.bit.CBD = AQ_NO_ACTION;

	pwmPtr->AQCTLB.bit.ZRO = AQ_SET;
	pwmPtr->AQCTLB.bit.PRD = AQ_NO_ACTION;
	pwmPtr->AQCTLB.bit.CAU = AQ_NO_ACTION;
	pwmPtr->AQCTLB.bit.CAD = AQ_NO_ACTION;
	pwmPtr->AQCTLB.bit.CBU = AQ_CLEAR;
	pwmPtr->AQCTLB.bit.CBD = AQ_NO_ACTION;

  	/* Compare registers configuration */
    pwmPtr->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    pwmPtr->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    pwmPtr->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    pwmPtr->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    /* Interruption configuration */
    pwmPtr->ETSEL.bit.INTSEL = ET_CTR_ZERO;       // Select INT on Zero event
    pwmPtr->ETSEL.bit.INTEN = 0;                  // Disable INT
    pwmPtr->ETPS.bit.INTPRD = ET_1ST;             // Generate INT on 1st event

    EALLOW;

    /* High-resolution feature configuration */
    pwmPtr->HRCNFG.all = 0x0;
   	pwmPtr->HRCNFG.bit.EDGMODE = HR_FEP;          		// MEP control on both edges
	pwmPtr->HRCNFG.bit.EDGMODEB = HR_REP;          		// MEP control on both edges
	pwmPtr->HRCNFG.bit.HRLOAD  = HR_CTR_ZERO; 			// load on CTR = 0
	pwmPtr->HRCNFG.bit.HRLOADB  = HR_CTR_ZERO; 			// load on CTR = 0
    pwmPtr->HRCNFG.bit.AUTOCONV = 0;              		// Enable autoconversion for HR period
    pwmPtr->HRCNFG.bit.CTLMODE = HR_CMP;          		// CMPAHR and TBPRDHR HR control
    pwmPtr->HRCNFG.bit.CTLMODEB = HR_CMP;          		// CMPBHR and TBPRDHR HR control
    pwmPtr->HRCNFG.bit.HRLOAD  = HR_CTR_ZERO_PRD; 		// load on CTR = 0 and CTR = TBPRD
    pwmPtr->HRCNFG.bit.HRLOADB  = HR_CTR_ZERO_PRD; 		// load on CTR = 0 and CTR = TBPRD

    /* Trip Zone - used to enable/disable PWM outputs via software */
    GpioG1TripRegs.GPTRIP1SEL.bit.GPTRIP1SEL = 29;		// GPIO29
    pwmPtr->TZSEL.bit.OSHT1 = 0;                  		// One-shot trip
    pwmPtr->TZCTL.bit.TZA = TZ_FORCE_LO;          		//
    pwmPtr->TZCTL.bit.TZB = TZ_FORCE_LO;          		// PWM outputs forced to low stateduring trip.
    pwmPtr->TZFRC.bit.OST = 1;
    EDIS;
}

/*
 * Initialization of PWM DAC (Digital-to-Analog Converter)
 */

void InitPWMDAC(volatile struct EPWM_REGS *pwmPtr, double freq)
{
	InitPWMModule(pwmPtr, freq, 0, SlavePWM, 0, NO_COMPLEMENTARY, 0);
}


/*
 * Enable outputs from specified PWM module
 */

void EnablePWMOutput(Uint16 pwm_module)
{
	// Clear trip flag, enabling PWM outputs
    EALLOW;
    PWM_Modules.PWM_Regs[pwm_module]->TZCLR.bit.OST = 1;
	PWM_Modules.OnOff[pwm_module] = PWM_ON;
    EDIS;
}

/*
 * Disable outputs from specified PWM module
 */

void DisablePWMOutput(Uint16 pwm_module)
{
	// Force trip via software, disabling PWM outputs
    EALLOW;
    PWM_Modules.PWM_Regs[pwm_module]->TZFRC.bit.OST = 1;
	PWM_Modules.OnOff[pwm_module] = PWM_OFF;
    EDIS;
}


/*
 * Enable outputs from all PWM modules
 */

void EnablePWMOutputs(void)
{
	Uint16 i;

	// Clear trip flags, enabling PWM outputs
    EALLOW;
	for(i = 0; i < PWM_Modules.N_modules; i++)
	{
		PWM_Modules.PWM_Regs[i]->TZCLR.bit.OST = 1;
		PWM_Modules.OnOff[i] = PWM_ON;
	}
    EDIS;
}

/*
 * Disable outputs from all PWM modules
 */

void DisablePWMOutputs(void)
{
	Uint16 i;

	// Force trip via software, disabling PWM outputs
    EALLOW;
	for(i = 0; i < PWM_Modules.N_modules; i++)
	{
		PWM_Modules.PWM_Regs[i]->TZFRC.bit.OST = 1;
		PWM_Modules.OnOff[i] = PWM_OFF;
	}
    EDIS;
}


/*
 * Enable time-base clock for PWM modules
 */

void EnablePWM_TBCLK(void)
{
	Uint16 i;

	for(i = 0; i < N_MAX_PWM_MODULES; i++)
	{
		PWM_Modules.PWM_Regs[i]->TBCTR = PWM_Modules.PWM_Regs[i]->TBPHS.half.TBPHS;
		PWM_Modules.PWM_Regs[i]->ETCLR.bit.INT = 1;
	}

	PieCtrlRegs.PIEIFR3.all = 0x0000;
	PieCtrlRegs.PIEACK.all |= M_INT3;
	IFR &= ~M_INT3;

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
}

/*
 * Disable time-base clock for PWM modules
 */

void DisablePWM_TBCLK(void)
{
	Uint16 i;

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    /* Adicionado 11/03/2016, vindo do FW da FAC/DCDC */
	for(i = 0; i < N_MAX_PWM_MODULES; i++)
	{
		PWM_Modules.PWM_Regs[i]->TBCTR = PWM_Modules.PWM_Regs[i]->TBPHS.half.TBPHS;
		PWM_Modules.PWM_Regs[i]->ETCLR.bit.INT = 1;
	}

	PieCtrlRegs.PIEIFR3.all = 0x0000;
	PieCtrlRegs.PIEACK.all |= M_INT3;
	IFR &= ~M_INT3;
}

/*
 * Enable interrupts for the specified PWM module
 */

void EnablePWMInterrupt(volatile struct EPWM_REGS *pwmPtr)
{
	EALLOW;
	pwmPtr->ETSEL.bit.INTEN = 1;
	EDIS;
}

/*
 * Disable interrupts for the specified PWM module
 */
void DisablePWMInterrupt(volatile struct EPWM_REGS *pwmPtr)
{
	EALLOW;
	pwmPtr->ETSEL.bit.INTEN = 0;
	pwmPtr->ETCLR.bit.INT = 1;
	EDIS;
}

/*
 * Initialization of calibration algorithm (SFO) for HRPWM MEP
 */

void InitPWM_MEP_SFO(void)
{
	SFO_status = SFO_INCOMPLETE;
    while(SFO_status == SFO_INCOMPLETE)
    {
        SFO_status = SFO();
        if (SFO_status == SFO_ERROR)
        {
            //error();		// SFO function returns 2 if an error occurs & # of MEP steps/coarse step
        }             		// exceeds maximum of 255.
    }
}


/*
 * Runs calibration algorithm (SFO) for HRPWM MEP
 */

void TunningPWM_MEP_SFO(void)
{
    /*SFO_status = SFO();
    if (SFO_status == SFO_ERROR)
    {
        //error();
    }          
    return;*/
}
