/*
 * 		FILE: 		IPC_modules.c
 * 		PROJECT: 	DRS v2.0
 * 		CREATION:	05/11/2015
 * 		MODIFIED:	05/11/2015
 *
 * 		AUTHOR: 	Ricieri  (LNLS/ELP)
 *
 * 		DESCRIPTION:
 *		Source code for interprocessor communications (IPC)
 *
 *		TODO:
 *				- ReadDPModule
 *				- ConfigDPModules só executa caso a fonte esteja desligada?
 *				- Ajustar WfmSync
 *				- Incluir flag no IPC_MtoC_Msg para ARM avisar C28 quando está pronto
 */

#include "IPC_modules.h"

#pragma DATA_SECTION(samplesBuffer, "SHARERAMS45")
#pragma DATA_SECTION(wfmRef_Curve, "SHARERAMS67")
#pragma DATA_SECTION(IPC_CtoM_Msg, "CTOM_MSG_RAM")
#pragma DATA_SECTION(IPC_MtoC_Msg, "MTOC_MSG_RAM")
#pragma DATA_SECTION(IPC_MtoC_Param, "MTOC_MSG_RAM")

#pragma CODE_SECTION(SendIpcFlag, "ramfuncs");
#pragma CODE_SECTION(isr_IPC_Channel_1, "ramfuncs");
#pragma CODE_SECTION(isr_IPC_Channel_2, "ramfuncs");
//#pragma CODE_SECTION(isr_IPC_Channel_3, "ramfuncs");
//#pragma CODE_SECTION(isr_IPC_Channel_4, "ramfuncs");
#pragma CODE_SECTION(ConfigPSOpMode, "ramfuncs");

void InitIPC(void (*ps_turnOn)(void),void (*ps_turnOff)(void), void (*isr_SoftItlk)(void), void (*isr_HardItlk)(void));
void SendIpcFlag(Uint32 flag);

interrupt void isr_IPC_Channel_1(void);
interrupt void isr_IPC_Channel_2(void);
//interrupt void isr_IPC_Channel_3(void);
//interrupt void isr_IPC_Channel_4(void);

void ConfigPSOpMode(ePSOpMode opMode);

volatile Uint16 		wfmSyncFlag;
volatile Uint16 		sigGenSyncFlag = 1;
volatile float 			samplesBuffer[SIZE_SAMPLES_BUFFER];
volatile union uWfmRef	wfmRef_Curve;

tIPC_CTOM_MSG_RAM		IPC_CtoM_Msg;
tIPC_MTOC_MSG_RAM		IPC_MtoC_Msg;
tIPC_MTOC_PARAM_RAM		IPC_MtoC_Param;
tIPC_PS_FUNCS			IPC_PS_funcs;


void InitIPC(void (*ps_turnOn)(void), void (*ps_turnOff)(void), void (*isr_SoftItlk)(void), void (*isr_HardItlk)(void))
{
	IPC_CtoM_Msg.PSModule.OnOff = 0;
	IPC_CtoM_Msg.PSModule.OpMode = SlowRef;
	IPC_CtoM_Msg.PSModule.OpenLoop = OPEN_LOOP;
	IPC_CtoM_Msg.PSModule.SoftInterlocks = 0x00000000;
	IPC_CtoM_Msg.PSModule.HardInterlocks = 0x00000000;
	IPC_CtoM_Msg.PSModule.BufferOnOff = 0;
	IPC_CtoM_Msg.PSModule.IRef = 0.0;
	IPC_CtoM_Msg.PSModule.ErrorMtoC = NO_ERROR_MTOC;

	InitBuffer(&IPC_CtoM_Msg.SamplesBuffer, samplesBuffer, SIZE_SAMPLES_BUFFER);
	IPC_CtoM_Msg.SamplesBuffer.BufferBusy = Buffer_All;
	InitBuffer(&IPC_CtoM_Msg.WfmRef.BufferInfo, wfmRef_Curve.WfmRef_Block.A, SIZE_WFMREF_BLOCK);
	IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferK = IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferEnd + 1;

	IPC_CtoM_Msg.WfmRef.Gain = 1.0;
	IPC_CtoM_Msg.WfmRef.Offset = 0.0;

	IPC_PS_funcs.PS_turnOn = ps_turnOn;
	IPC_PS_funcs.PS_turnOff = ps_turnOff;

	EALLOW;

	/*
    *  Set WfmRef synchronization via EPWMSYNCI pin (defined by UDC version)
    *
    *  	Qualification is asynchronous
    *  	Define GPIO32/38 as interrupt source XINT2
    *  	Enable XINT2
    *  	EPWMSYNCI negative-edge triggering
    */

	if(UDC_V2_0)
	{
	    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;
	    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 0;
	    GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 1;

	    GpioTripRegs.GPTRIP5SEL.bit.GPTRIP5SEL = 32;
		XIntruptRegs.XINT2CR.bit.ENABLE = 1;
		XIntruptRegs.XINT2CR.bit.POLARITY = 0;

		/*GpioTripRegs.GPTRIP6SEL.bit.GPTRIP6SEL = 32;
	    XIntruptRegs.XINT3CR.bit.ENABLE = 0;
	    XIntruptRegs.XINT3CR.bit.POLARITY = 0;*/
	}

	else if(UDC_V2_1)
	{
	    GpioCtrlRegs.GPBMUX1.bit.GPIO38 = 0;
	    GpioCtrlRegs.GPBDIR.bit.GPIO38 = 0;
	    GpioCtrlRegs.GPBQSEL1.bit.GPIO38 = 1;

	    GpioTripRegs.GPTRIP5SEL.bit.GPTRIP5SEL = 38;
		XIntruptRegs.XINT2CR.bit.ENABLE = 1;
		XIntruptRegs.XINT2CR.bit.POLARITY = 0;

		/*GpioTripRegs.GPTRIP6SEL.bit.GPTRIP6SEL = 38;
	    XIntruptRegs.XINT3CR.bit.ENABLE = 0;
	    XIntruptRegs.XINT3CR.bit.POLARITY = 0;*/
	}



	/* Map IPC_MtoC interrupts */

	PieVectTable.MTOCIPC_INT1 = &isr_IPC_Channel_1;		// IPC-MtoC low-priority msg
	PieVectTable.MTOCIPC_INT2 = &isr_IPC_Channel_2;		// WfmSync
	PieVectTable.XINT2 		  = &isr_IPC_Channel_2;		// WfmSync
	//PieVectTable.XINT3 		  = &isr_IPC_Channel_2;		// WfmSync
	PieVectTable.MTOCIPC_INT3 = isr_SoftItlk;		// Soft interlock
	PieVectTable.MTOCIPC_INT4 = isr_HardItlk;		// Hard interlock

	/* Enable IPC_MtoC interrupts */

	PieCtrlRegs.PIEIER11.bit.INTx1 = 1;					// MTOCIPCINT1
	PieCtrlRegs.PIEIER11.bit.INTx2 = 1;					// MTOCIPCINT2 	--- Enable only in
	PieCtrlRegs.PIEIER1.bit.INTx5  = 1;  				//    XINT2		--- WfmRef OpMode
	//PieCtrlRegs.PIEIER12.bit.INTx1 = 1;  				//    XINT3		--- WfmRef OpMode
	PieCtrlRegs.PIEIER11.bit.INTx3 = 1;					// MTOCIPCINT3
	PieCtrlRegs.PIEIER11.bit.INTx4 = 1;					// MTOCIPCINT4

	EDIS;
}

//*****************************************************************************
// Function to set CTOM_IPCSET register
//*****************************************************************************
void SendIpcFlag(Uint32 flag)
{
	CtoMIpcRegs.CTOMIPCSET.all |= flag;
	return;
}


interrupt void isr_IPC_Channel_1(void)
{
	static Uint32 aux;

	aux = (CtoMIpcRegs.MTOCIPCSTS.all & 0xFFFFFFF1);

	switch(aux)
	{
		case IPC_PS_ON_OFF: //IPC1 +IPC5
		{
			CtoMIpcRegs.MTOCIPCACK.all = IPC_PS_ON_OFF;

			if(IPC_MtoC_Msg.PSModule.OnOff)
			{
				Disable_ELP_SigGen(&SignalGenerator);
				IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferK = IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferEnd + 1;
				IPC_PS_funcs.PS_turnOn();
			}
			else
			{
				IPC_PS_funcs.PS_turnOff();
				Disable_ELP_SigGen(&SignalGenerator);
				IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferK = IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferEnd + 1;
			}

			PieCtrlRegs.PIEACK.all |= M_INT11;
			break;
		}

		case OPERATING_MODE: //IPC1 +IPC6
		{
			CtoMIpcRegs.MTOCIPCACK.all = OPERATING_MODE;
			ConfigPSOpMode(IPC_MtoC_Msg.PSModule.OpMode);
			PieCtrlRegs.PIEACK.all |= M_INT11;
			break;
		}

		case OPEN_CLOSE_LOOP: //IPC1 +IPC7
		{
			CtoMIpcRegs.MTOCIPCACK.all = OPEN_CLOSE_LOOP;
			IPC_CtoM_Msg.PSModule.OpenLoop = IPC_MtoC_Msg.PSModule.OpenLoop;
			PieCtrlRegs.PIEACK.all |= M_INT11;
			break;
		}

		case SLOWREF_UPDATE: //IPC1 +IPC8
		{
			CtoMIpcRegs.MTOCIPCACK.all = SLOWREF_UPDATE;

			if(IPC_CtoM_Msg.PSModule.OpMode == SlowRef)
			{
				IPC_CtoM_Msg.PSModule.IRef = IPC_MtoC_Msg.PSModule.ISlowRef;
			}
			else
			{
				IPC_CtoM_Msg.PSModule.ErrorMtoC = INVALID_OPMODE;
				SendIpcFlag(MTOC_MESSAGE_ERROR);
			}

			PieCtrlRegs.PIEACK.all |= M_INT11;
			break;
		}

		case SIGGEN_ENABLE: //IPC1 +IPC9
		{
			CtoMIpcRegs.MTOCIPCACK.all = SIGGEN_ENABLE;

			if(IPC_MtoC_Msg.SigGen.Enable)
			{
				Enable_ELP_SigGen(&SignalGenerator);
			}
			else
			{
				Reset_ELP_SigGen(&SignalGenerator);
				Disable_ELP_SigGen(&SignalGenerator);
			}

			PieCtrlRegs.PIEACK.all |= M_INT11;
			break;
		}

		case SIGGEN_CONFIG: //IPC1 +IPC10
		{
			CtoMIpcRegs.MTOCIPCACK.all = SIGGEN_CONFIG;

			Init_ELP_SigGen(&SignalGenerator, IPC_MtoC_Msg.SigGen.Type, IPC_MtoC_Msg.SigGen.PhaseStart, IPC_MtoC_Msg.SigGen.PhaseEnd, IPC_MtoC_Msg.SigGen.Ncycles,
							SignalGenerator.FreqSample, SignalGenerator.ptr_FreqSignal, SignalGenerator.ptr_Amp, SignalGenerator.ptr_Offset, SignalGenerator.ptr_Aux, DP_Framework.Ref);

			PieCtrlRegs.PIEACK.all |= M_INT11;
			break;
		}

		case DPMODULES_CONFIG: //IPC1 +IPC11
		{
			CtoMIpcRegs.MTOCIPCACK.all = DPMODULES_CONFIG;

			if(!ConfigDP_Module(&DP_Framework, IPC_MtoC_Msg.DPModule.ID, IPC_MtoC_Msg.DPModule.DPclass, IPC_MtoC_Msg.DPModule.Coeffs))
			{
				IPC_CtoM_Msg.PSModule.ErrorMtoC = INVALID_DP_MODULE;
				SendIpcFlag(MTOC_MESSAGE_ERROR);
			}


			PieCtrlRegs.PIEACK.all |= M_INT11;
			break;
		}

		case SAMPLES_BUFFER_ON_OFF: //IPC1 + IPC12
		{
			CtoMIpcRegs.MTOCIPCACK.all = SAMPLES_BUFFER_ON_OFF;
			IPC_CtoM_Msg.PSModule.BufferOnOff = IPC_MtoC_Msg.PSModule.BufferOnOff;
			IPC_CtoM_Msg.SamplesBuffer.BufferBusy = IPC_MtoC_Msg.PSModule.BufferOnOff;
			PieCtrlRegs.PIEACK.all |= M_INT11;
			break;
		}

		case RESET_INTERLOCKS: // IPC1 + IPC13
		{
			CtoMIpcRegs.MTOCIPCACK.all = RESET_INTERLOCKS;
			IPC_CtoM_Msg.PSModule.SoftInterlocks = 0;
			IPC_CtoM_Msg.PSModule.HardInterlocks = 0;
			PieCtrlRegs.PIEACK.all |= M_INT11;
			break;
		}

		case RESET_WFMREF: 	// IPC1 + IPC14
		{
			CtoMIpcRegs.MTOCIPCACK.all = RESET_WFMREF;
			IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferK = IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferEnd + 1;
			PieCtrlRegs.PIEACK.all |= M_INT11;
			break;
		}

		case HRADC_SAMPLING_DISABLE: //IPC1 +IPC28
		{
			CtoMIpcRegs.MTOCIPCACK.all = HRADC_SAMPLING_DISABLE;
			Disable_HRADC_Sampling();
			PieCtrlRegs.PIEACK.all |= M_INT11;
			break;
		}

		case HRADC_SAMPLING_ENABLE: //IPC1 +IPC29
		{
			CtoMIpcRegs.MTOCIPCACK.all = HRADC_SAMPLING_ENABLE;
			Enable_HRADC_Sampling();
			PieCtrlRegs.PIEACK.all |= M_INT11;
			break;
		}

		case HRADC_OPMODE: //IPC1 +IPC30
		{
			CtoMIpcRegs.MTOCIPCACK.all = HRADC_OPMODE;
			switch(IPC_MtoC_Msg.HRADCConfig.OpMode)
			{
				case HRADC_Sampling:
				{
					Config_HRADC_Sampling_OpMode(IPC_MtoC_Msg.HRADCConfig.ID);
					break;
				}
				case HRADC_UFM:
				{
					Config_HRADC_UFM_OpMode(IPC_MtoC_Msg.HRADCConfig.ID);
					break;
				}

				default:
				{
					IPC_CtoM_Msg.PSModule.ErrorMtoC = INVALID_OPMODE;
					SendIpcFlag(MTOC_MESSAGE_ERROR);
					break;
				}
			}
			PieCtrlRegs.PIEACK.all |= M_INT11;
			break;
		}

		case HRADC_CONFIG: //IPC1 +IPC31
		{
			CtoMIpcRegs.MTOCIPCACK.all = HRADC_CONFIG;

			Config_HRADC_SoC(IPC_MtoC_Msg.HRADCConfig.FreqSampling);

			if(Try_Config_HRADC_board(HRADCs_Info.HRADC_boards[IPC_MtoC_Msg.HRADCConfig.ID],
			   IPC_MtoC_Msg.HRADCConfig.InputType,
			   IPC_MtoC_Msg.HRADCConfig.EnableHeater,
			   IPC_MtoC_Msg.HRADCConfig.EnableMonitor))
			{
				IPC_CtoM_Msg.PSModule.ErrorMtoC = HRADC_CONFIG_ERROR;
				SendIpcFlag(MTOC_MESSAGE_ERROR);
			}

			PieCtrlRegs.PIEACK.all |= M_INT11;
			break;
		}

		case CTOM_MESSAGE_ERROR: //IPC1 +IPC32
		{
			CtoMIpcRegs.MTOCIPCACK.all = CTOM_MESSAGE_ERROR;

			PieCtrlRegs.PIEACK.all |= M_INT11;
			break;
		}

		default:
		{
			CtoMIpcRegs.MTOCIPCACK.all = 0x000000001;

			IPC_CtoM_Msg.PSModule.ErrorMtoC = IPC_LOW_PRIORITY_MSG_FULL;
			SendIpcFlag(MTOC_MESSAGE_ERROR);

			PieCtrlRegs.PIEACK.all |= M_INT11;
			break;
		}
	}
}

interrupt void isr_IPC_Channel_2(void)
{
	/*
	 * WfmRef sync via software
	 *
	 * 		TODO: Incluir um condicional para esta sincronização via software,
	 * 			  uma vez que via hardware também força o sincronismo (de maneira
	 * 			  até mais determinística) e aciona esta ISR, o que resetaria o
	 * 			  contador do PWM duas vezes consecutivas, levando à possíveis
	 * 			  glitches nos sinais PWM. A flag PIEIFR não pode ser usada, pois
	 * 			  a CPU reseta ela assim que entra nesta ISR. Talvez usar o contador
	 * 			  XIntruptRegs.XINT2CTR. Verificar se também precisa condicionar o
	 * 			  ACK dos grupos INT1 (XINT2) e INT11 (MTOCIPCINT2).
	 */

	//checkar a flag do IPC para fazer condicional

	SET_DEBUG_GPIO1;

	if(IPC_CtoM_Msg.PSModule.OpMode == WfmRef)
	{
		/*if(CtoMIpcRegs.MTOCIPCSTS.bit.IPC2)
		{
			EPwm1Regs.TBCTL.bit.SWFSYNC = 1;
		}*/

		switch(IPC_CtoM_Msg.WfmRef.SyncMode)
		{
			case OneShot:
			{
				IPC_CtoM_Msg.WfmRef = IPC_MtoC_Msg.WfmRef;
				TimeSlicer.Counter[0] = TimeSlicer.FreqRatio[0];
				wfmSyncFlag = 1;
				break;
			}

			case SampleBySample:
			{
				if(IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferK++ == IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferEnd)
				{
					IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferK = IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferEnd;
				}
				else if(IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferK > IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferEnd)
				{
					IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferK = IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferStart;
					IPC_CtoM_Msg.WfmRef.Gain = IPC_MtoC_Msg.WfmRef.Gain;
					IPC_CtoM_Msg.WfmRef.Offset = IPC_MtoC_Msg.WfmRef.Offset;
				}

				break;
			}

			case SampleBySample_Continuous:
			{
				if(IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferK++ >= IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferEnd)
				{
					IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferK = IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferStart;
					IPC_CtoM_Msg.WfmRef.Gain = IPC_MtoC_Msg.WfmRef.Gain;
					IPC_CtoM_Msg.WfmRef.Offset = IPC_MtoC_Msg.WfmRef.Offset;
				}
				break;
			}
		}
	}
	else if(IPC_CtoM_Msg.PSModule.OpMode == SigGen && IPC_MtoC_Msg.PSModule.Model == FAP_DCDC_20kHz && sigGenSyncFlag == 1)
	{
		Enable_ELP_SigGen(&SignalGenerator);
		sigGenSyncFlag = 0;
	}
	else
	{
		IPC_CtoM_Msg.PSModule.ErrorMtoC = INVALID_OPMODE;
		SendIpcFlag(MTOC_MESSAGE_ERROR);
	}

	CtoMIpcRegs.MTOCIPCACK.all = WFMREF_SYNC;
	PieCtrlRegs.PIEACK.all |= M_INT1;
	PieCtrlRegs.PIEACK.all |= M_INT11;

	//CLEAR_DEBUG_GPIO1;
}

/*interrupt void isr_IPC_Channel_3(void)
{
	IPC_PS_funcs.PS_turnOff();
	IPC_CtoM_Msg.PSModule.SoftInterlocks = 1;
	CtoMIpcRegs.MTOCIPCACK.all = SOFT_INTERLOCK_MTOC;
	PieCtrlRegs.PIEACK.all |= M_INT11;
}

interrupt void isr_IPC_Channel_4(void)
{
	IPC_PS_funcs.PS_turnOff();
	IPC_CtoM_Msg.PSModule.HardInterlocks = 1;
	CtoMIpcRegs.MTOCIPCACK.all = HARD_INTERLOCK_MTOC;
	PieCtrlRegs.PIEACK.all |= M_INT11;
}*/

/***************************************/
void ConfigPSOpMode(ePSOpMode opMode)
{
	switch(opMode)
	{
		case SlowRef:

//			PieCtrlRegs.PIEIER11.bit.INTx2 = 0;
//			PieCtrlRegs.PIEIER1.bit.INTx5  = 0;

//			IPC_CtoM_Msg.PSModule.IRef = IPC_MtoC_Msg.PSModule.ISlowRef;

			break;

		case FastRef:

//			PieCtrlRegs.PIEIER11.bit.INTx2 = 0;
//			PieCtrlRegs.PIEIER1.bit.INTx5  = 0;
			break;

		case WfmRef:

			// TODO:
			IPC_CtoM_Msg.WfmRef = IPC_MtoC_Msg.WfmRef;
			IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferK = IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferEnd + 1;

			/*
			 * TODO:	- Clear XINT2 FLAGs
			 * 			- Procurar forma correta de setar/resetar
			 * 			os bits do PIEIER
			 */
//			PieCtrlRegs.PIEIER11.bit.INTx2 = 1;
//			PieCtrlRegs.PIEIER1.bit.INTx5  = 1;
			break;

		case SigGen:
			sigGenSyncFlag = 1;
			Disable_ELP_SigGen(&SignalGenerator);
			Init_ELP_SigGen(&SignalGenerator, IPC_MtoC_Msg.SigGen.Type, IPC_MtoC_Msg.SigGen.PhaseStart, IPC_MtoC_Msg.SigGen.PhaseEnd, IPC_MtoC_Msg.SigGen.Ncycles,
							SignalGenerator.FreqSample, SignalGenerator.ptr_FreqSignal, SignalGenerator.ptr_Amp, SignalGenerator.ptr_Offset, SignalGenerator.ptr_Aux, DP_Framework.Ref);
//			PieCtrlRegs.PIEIER11.bit.INTx2 = 1;
//			PieCtrlRegs.PIEIER1.bit.INTx5  = 0;
			break;

		default:

			break;
	}

	IPC_CtoM_Msg.PSModule.OpMode = opMode;
	//IPC_CtoM_Msg.PSModule.IRef = 0;
}
