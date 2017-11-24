/*
 * 		FILE: 		TimeSlicer.h
 * 		PROJECT: 	DRS v2.0
 * 		CREATION:	05/04/2016
 * 		MODIFIED:	05/04/2016
 *
 * 		AUTHOR: 	Gabriel O. B.  (LNLS/ELP)
 *
 * 		DESCRIPTION:
 *		Source code for Time Slicer module
 *
 *		TODO:
 */

#include <common/timeslicer/TimeSlicer.h>

tTimeSlicer TimeSlicer;

void Reset_TimeSlicers(void);
void Set_TimeSlicer(Uint16 ID, Uint16 ratio);

void Reset_TimeSlicers(void)
{
	Uint16 i;

	for(i = 0; i < N_MAX_TIMESLICERS; i++)
	{
		TimeSlicer.Counter[i] = TimeSlicer.FreqRatio[i];
	}
}

void Set_TimeSlicer(Uint16 ID, Uint16 ratio)
{
	TimeSlicer.FreqRatio[ID] = ratio;
	TimeSlicer.Counter[ID] 	 = ratio;
}

