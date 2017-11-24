#ifndef PWM_MODULES_H
#define PWM_MODULES_H

typedef enum {
		OnePS1Q,					// 1 Fonte com 1 quadrante
		OnePS2Q,					// 1 Fonte com 2 quadrantes
		OnePS4Q,					// 1 Fonte com 4 quadrantes  (H-Bridge)
		nPS1Q,						// n Fontes com 1 quadrante  (Interleaving)
		nPS2Q						// n Fontes com 2 quadrantes (Interleaving)
} ePWMConfig;

typedef enum {
		MasterPWM,
		SlavePWM
} ePWMSync;

typedef enum {
		NO_COMPLEMENTARY,
		COMPLEMENTARY
} ePWMComp;

#define PWM_ON		1
#define	PWM_OFF		0

#define PWM_CH_ENABLE	1
#define PWM_CH_DISABLE	0

#define STATUS_SUCCESS 	1
#define STATUS_FAIL    	0
#define AUTOCONVERT 	0      	 // 1 = Turn auto-conversion ON, 0 = Turn auto-conversion OFF

#define TZ_ONE_SHOT     1

#define N_MAX_PWM_MODULES	8

//#include "F28M36x_ELP_DRS.h"
#include "boards/udc_c28.h"
#include "SFO_V7.h"

typedef volatile struct
{
	Uint16 		N_modules;
	Uint16 		OnOff[N_MAX_PWM_MODULES];
	volatile struct EPWM_REGS *PWM_Regs[N_MAX_PWM_MODULES];
} tPWMmodules;

extern int MEP_ScaleFactor;
extern tPWMmodules PWM_Modules;

//*****************************************************************************
// Prot�tipo das fun��es 
//*****************************************************************************
extern Uint16 SetPWMFreq(volatile struct EPWM_REGS *pwmPtr, double freq);
extern void   SetPWMDeadTime(volatile struct EPWM_REGS *pwmPtr, Uint16 dt_ns);
extern void   SetPWMSyncPhase(volatile struct EPWM_REGS *pwmPtr, Uint16 degrees);
extern void   SetPWMCounterSync(volatile struct EPWM_REGS *pwmPtr, ePWMSync sync_mode, Uint16 phase_degrees);
extern void   SetPWMComplementary(volatile struct EPWM_REGS *pwmPtr, ePWMComp comp_mode);
extern void   LinkPWM_Modules(volatile struct EPWM_REGS *pwmPtr, Uint16 primary_module);

extern void SetPWMDutyCycle_ChA(volatile struct EPWM_REGS *pwmPtr, float duty_pu);
extern void SetPWMDutyCycle_ChB(volatile struct EPWM_REGS *pwmPtr, float duty_pu);
extern void SetPWMDutyCycle_HBridge(volatile struct EPWM_REGS *pwmPtr, float duty_pu);

extern void InitPWMModule(volatile struct EPWM_REGS *pwmPtr, double freq, Uint16 primary_module,
		           ePWMSync sync_mode, Uint16 phase_degrees, ePWMComp comp_mode, Uint16 dt_ns);
extern void InitPWMDAC(volatile struct EPWM_REGS *pwmPtr, double freq);

extern void EnablePWMOutput(Uint16 pwm_module);
extern void DisablePWMOutput(Uint16 pwm_module);
extern void EnablePWMOutputs(void);
extern void DisablePWMOutputs(void);

extern void EnablePWM_TBCLK(void);
extern void DisablePWM_TBCLK(void);

extern void EnablePWMInterrupt(volatile struct EPWM_REGS *pwmPtr);
extern void DisablePWMInterrupt(volatile struct EPWM_REGS *pwmPtr);

extern void InitPWM_MEP_SFO(void);
extern void TunningPWM_MEP_SFO(void);

#endif /*PWM_CONFIG_H_*/
