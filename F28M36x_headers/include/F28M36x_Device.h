//###########################################################################
// FILE:   F28M36x_Device.h
// TITLE:  F28M36x Device Definitions.
//###########################################################################
// $TI Release: F28M36x Support Library v202 $
// $Release Date: Tue Apr  8 12:36:34 CDT 2014 $
//###########################################################################

#ifndef F28M36x_DEVICE_H
#define F28M36x_DEVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#define   TARGET                     1
//---------------------------------------------------------------------------
// User To Select Target Device:


#define   DSP28_F28M36Hx_100MHZ      0
#define   DSP28_F28M36Hx_150MHZ      0
#define   DSP28_F28M36Px_125MHZ      0
#define   DSP28_F28M36Px_150MHZ      TARGET

//---------------------------------------------------------------------------
// Common CPU Definitions:

extern __cregister volatile unsigned int IFR;
extern __cregister volatile unsigned int IER;

#define  EINT            __asm(" clrc INTM")
#define  DINT            __asm(" setc INTM")
#define  ERTM            __asm(" clrc DBGM")
#define  DRTM            __asm(" setc DBGM")
#define  EALLOW          __asm(" EALLOW")
#define  EDIS            __asm(" EDIS")
#define  ESTOP0          __asm(" ESTOP0")

#define M_INT1           0x0001
#define M_INT2           0x0002
#define M_INT3           0x0004
#define M_INT4           0x0008
#define M_INT5           0x0010
#define M_INT6           0x0020
#define M_INT7           0x0040
#define M_INT8           0x0080
#define M_INT9           0x0100
#define M_INT10          0x0200
#define M_INT11          0x0400
#define M_INT12          0x0800
#define M_INT13          0x1000
#define M_INT14          0x2000
#define M_DLOG           0x4000
#define M_RTOS           0x8000

#define BIT0             0x0001
#define BIT1             0x0002
#define BIT2             0x0004
#define BIT3             0x0008
#define BIT4             0x0010
#define BIT5             0x0020
#define BIT6             0x0040
#define BIT7             0x0080
#define BIT8             0x0100
#define BIT9             0x0200
#define BIT10            0x0400
#define BIT11            0x0800
#define BIT12            0x1000
#define BIT13            0x2000
#define BIT14            0x4000
#define BIT15            0x8000

//---------------------------------------------------------------------------
// For Portability, User Is Recommended To Use Following Data Type Size
// Definitions For 16-bit and 32-Bit Signed/Unsigned Integers:

#ifndef DSP28_DATA_TYPES
#define DSP28_DATA_TYPES
typedef int int16;
typedef long int32;
typedef long long int64;
typedef unsigned long long Uint64;
typedef float float32;
typedef long double float64;
#endif

#ifndef _TI_STD_TYPES
#define _TI_STD_TYPES

/*
 * These types are also defined in DSP/BIOS 5.x's <std.h> and the
 * SYS/BIOS 6.x's <xdc/std.h> files.  We need to protect their
 * definition with the #ifndef/#define guard to avoid the duplicate
 * definition warning.
 *
 * SYS/BIOS requires that the <xdc/std.h> file be included before
 * any other .h files.
 */

typedef int Int;
typedef unsigned Uns;
typedef char Char;
typedef char            *String;
typedef void            *Ptr;
typedef unsigned short Bool;

typedef unsigned long Uint32;
typedef unsigned int Uint16;
typedef unsigned char Uint8;

typedef long Int32;
typedef int Int16;
typedef char Int8;

#endif


//---------------------------------------------------------------------------
// Include All Peripheral Header Files:

#include "F28M36x_Adc.h"                  // ADC Registers
#include "F28M36x_AnalogSysCtrl.h"        // C28 AnalogSysCtrl Registers
#include "F28M36x_Comp.h"                 // Comparator Registers
#include "F28M36x_CpuTimers.h"            // 32-bit CPU Timers
#include "F28M36x_DevEmu.h"               // Device Emulation Registers
#include "F28M36x_Dma.h"                  // DMA Registers
#include "F28M36x_Ecap.h"                 // Enhanced Capture
#include "F28M36x_Epwm.h"                 // Enhanced PWM
#include "F28M36x_Eqep.h"                 // Enhanced QEP
#include "F28M36x_Flash.h"                // Flash Error Registers
#include "F28M36x_Gpio.h"                 // General Purpose I/O Registers
#include "F28M36x_HWBist.h"               // HWBIST Registers
#include "F28M36x_I2c.h"                  // I2C Registers
#include "F28M36x_Ipc.h"                  // C28 Inter-processor Communication
                                          // Registers
#include "F28M36x_Mcbsp.h"                // McBSP Registers
#include "F28M36x_NmiIntrupt.h"           // NMI Interrupt Registers
#include "F28M36x_PieCtrl.h"              // PIE Control Registers
#include "F28M36x_PieVect.h"              // PIE Vector Table
#include "F28M36x_Ram.h"                  // RAM Registers
#include "F28M36x_Sci.h"                  // SCI Registers
#include "F28M36x_Spi.h"                  // SPI Registers
#include "F28M36x_Sysctrl.h"              // System Control/Power Modes
#include "F28M36x_XIntrupt.h"             // External Interrupts

#if (DSP28_F28M36Hx_100MHZ)
// Timer definitions based on System Clock
// 100 MHz devices
    #define      mSec0_5           50000          // 0.5 mS
    #define      mSec0_75          75000          // 0.75 mS
    #define      mSec1             100000         // 1.0 mS
    #define      mSec2             200000         // 2.0 mS
    #define      mSec5             500000         // 5.0 mS
    #define      mSec7_5           750000         // 7.5 mS
    #define      mSec10            1000000        // 10 mS
    #define      mSec20            2000000        // 20 mS
    #define      mSec50            5000000        // 50 mS
    #define      mSec75            7500000        // 75 mS
    #define      mSec100           10000000       // 100 mS
    #define      mSec200           20000000       // 200 mS
    #define      mSec500           50000000       // 500 mS
    #define      mSec750           75000000       // 750 mS
    #define      mSec1000          100000000      // 1000 mS
    #define      mSec2000          200000000      // 2000 mS
    #define      mSec5000          500000000      // 5000 mS
#endif

#if (DSP28_F28M36Hx_150MHZ)
// Timer definitions based on System Clock
// 150 MHz devices
    #define      mSec0_5           75000          // 0.5 mS
    #define      mSec0_75          112500         // 0.75 mS
    #define      mSec1             150000         // 1.0 mS
    #define      mSec2             300000         // 2.0 mS
    #define      mSec5             750000         // 5.0 mS
    #define      mSec7_5           1125000        // 7.5 mS
    #define      mSec10            1500000        // 10 mS
    #define      mSec20            3000000        // 20 mS
    #define      mSec50            7500000        // 50 mS
    #define      mSec75            11250000       // 75 mS
    #define      mSec100           15000000       // 100 mS
    #define      mSec200           30000000       // 200 mS
    #define      mSec500           75000000       // 500 mS
    #define      mSec750           112500000      // 750 mS
    #define      mSec1000          150000000      // 1000 mS
    #define      mSec2000          300000000      // 2000 mS
    #define      mSec5000          750000000      // 5000 mS
#endif

#if (DSP28_F28M36Px_125MHZ)
// Timer definitions based on System Clock
// 125 MHz devices
    #define      mSec0_5           62500          // 0.5 mS
    #define      mSec0_75          93750          // 0.75 mS
    #define      mSec1             125000         // 1.0 mS
    #define      mSec2             250000         // 2.0 mS
    #define      mSec5             625000         // 5.0 mS
    #define      mSec7_5           937500         // 7.5 mS
    #define      mSec10            1250000        // 10 mS
    #define      mSec20            2500000        // 20 mS
    #define      mSec50            6250000        // 50 mS
    #define      mSec75            9375000        // 75 mS
    #define      mSec100           12500000       // 100 mS
    #define      mSec200           25000000       // 200 mS
    #define      mSec500           62500000       // 500 mS
    #define      mSec750           93750000       // 750 mS
    #define      mSec1000          125000000      // 1000 mS
    #define      mSec2000          250000000      // 2000 mS
    #define      mSec5000          625000000      // 5000 mS
#endif

#if (DSP28_F28M36Px_150MHZ)
// Timer definitions based on System Clock
// 150 MHz devices
    #define      mSec0_5           75000          // 0.5 mS
    #define      mSec0_75          112500         // 0.75 mS
    #define      mSec1             150000         // 1.0 mS
    #define      mSec2             300000         // 2.0 mS
    #define      mSec5             750000         // 5.0 mS
    #define      mSec7_5           1125000        // 7.5 mS
    #define      mSec10            1500000        // 10 mS
    #define      mSec20            3000000        // 20 mS
    #define      mSec50            7500000        // 50 mS
    #define      mSec75            11250000       // 75 mS
    #define      mSec100           15000000       // 100 mS
    #define      mSec200           30000000       // 200 mS
    #define      mSec500           75000000       // 500 mS
    #define      mSec750           112500000      // 750 mS
    #define      mSec1000          150000000      // 1000 mS
    #define      mSec2000          300000000      // 2000 mS
    #define      mSec5000          750000000      // 5000 mS
#endif


#ifdef __cplusplus
}
#endif  /* extern "C" */

#endif  // end of F28M36x_DEVICE_H definition


