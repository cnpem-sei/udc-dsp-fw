#ifndef MCBSP_SPI_INTERFACE_H
#define MCBSP_SPI_INTERFACE_H

//#include "DSP28x_Project.h"
//#include "../../config.h"
#include "F28M36x_ELP_DRS.h"


#define SPI_25MHz			2
#define SPI_18_75MHz		3
#define SPI_15MHz			4
#define SPI_12MHz			5

#define McBSP_CLKGDV 		SPI_15MHz
#define McBSP_WORD_SIZE    	20

extern void Init_SPIMaster_McBSP(void);
extern void Init_SPIMaster_Gpio(void);
 
#endif
