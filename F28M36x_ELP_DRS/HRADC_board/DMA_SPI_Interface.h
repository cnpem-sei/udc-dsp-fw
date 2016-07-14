//#include "DSP28x_Project.h"
//#include "../C28 Project/config.h"
#include "F28M36x_ELP_DRS.h"

#ifndef DMA_SPI_INTERFACE_H
#define DMA_SPI_INTERFACE_H

#define DMA_TRANSFER_SIZE DECIMATION_FACTOR

typedef volatile struct
{
	Uint32 buffer_0[DMA_TRANSFER_SIZE];
	Uint32 buffer_1[DMA_TRANSFER_SIZE];
	Uint32 buffer_2[DMA_TRANSFER_SIZE];
	Uint32 buffer_3[DMA_TRANSFER_SIZE];
} tbuffers_HRADC;

extern __interrupt void local_D_INTCH1_ISR(void);
extern __interrupt void local_D_INTCH2_ISR(void);
extern void Init_DMA_32bits_McBSP(void);
extern void Init_DMA_McBSP_nBuffers(Uint16 n_buffers, Uint16 size_buffers);
extern void start_DMA(void);
extern void stop_DMA(void);

extern volatile Uint32 i_rdata;
extern volatile Uint32 dummy_data;
extern volatile tbuffers_HRADC buffers_HRADC;

#endif	/* DMA_SPI_INTERFACE_H */
