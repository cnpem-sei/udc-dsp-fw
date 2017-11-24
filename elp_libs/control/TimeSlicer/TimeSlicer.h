/*
 * 		FILE: 		TimeSlicer.h
 * 		PROJECT: 	DRS v2.0
 * 		CREATION:	05/04/2016
 * 		MODIFIED:	05/04/2016
 *
 * 		AUTHOR: 	Gabriel O. B.  (LNLS/ELP)
 *
 * 		DESCRIPTION:
 *		Definition of constants, structs, macros and prototypes statements
 *		for variables and functions	from source code of Time Slicer
 *
 *		TODO:
 */

#ifndef	TIMESLICER_H
#define	TIMESLICER_H

#define N_MAX_TIMESLICERS	6

#define RUN_TIMESLICE(id)	if(TimeSlicer.Counter[id]++ == TimeSlicer.FreqRatio[id]){
#define	END_TIMESLICE(id)	TimeSlicer.Counter[id] = 1;}

//#include "F28M36x_ELP_DRS.h"
#include "boards/udc_c28.h"

typedef volatile struct
{
	Uint16	FreqRatio[N_MAX_TIMESLICERS];
	Uint16	Counter[N_MAX_TIMESLICERS];
} tTimeSlicer;

extern tTimeSlicer TimeSlicer;

extern void Reset_TimeSlicers(void);
extern void Set_TimeSlicer(Uint16 ID, Uint16 ratio);

#endif
