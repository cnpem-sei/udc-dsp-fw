/*--------------------------------------------------------*/
/* F021\Constants\Concerto.h                                */
/*                                                        */
/* Copyright (c) 2010-2011 Texas Instruments Incorporated */
/*                                                        */
/*--------------------------------------------------------*/

/*!
    \file F021\Constants\Concerto.h
    \brief A set of Constant Values for the Concerto Family.
*/
#ifndef F021_CONSTANTS_CONCERTO_H_
#define F021_CONSTANTS_CONCERTO_H_

/*!
    \brief Specifies the bit mask for determining all address bits exclusive of
    the offest imposed by the memory map register
*/
#define F021_PROGRAM_ADDRESS_MASK    0x07FFFFFF

/*!
    \brief Specifies the Offset to the TI OTP
*/
#if defined (_C28X)
	#define F021_PROGRAM_TIOTP_OFFSET    0x00240000 //TI OTP start on C28x
#else
    #define F021_PROGRAM_TIOTP_OFFSET    0x00680000 //TI OTP start on M3
#endif


/* Final values to be determined */

/*!
    \brief Maximum FClck Value
*/
#define F021_FCLK_MAX    0x18


/*!
    \brief PGM_OSU Max Value
*/
#define F021_PGM_OSU_MAX    0xFF

/*!
    \brief PGM_OSU Min Value
*/
#define F021_PGM_OSU_MIN    0x02

/*!
    \brief ERA_OSU Max Value
*/
#define F021_ERA_OSU_MAX    0xFF

/*!
    \brief ERA_OSU Min Value
*/
#define F021_ERA_OSU_MIN    0x02

/*!
    \brief ADD_EXZ Max Value
*/
#define F021_ADD_EXZ_MAX    0x0F

/*!
    \brief ADD_EXZ Min Value
*/
#define F021_ADD_EXZ_MIN    0x02

/*!
    \brief EXE_VALD Max Value
*/
#define F021_EXE_VALD_MAX    0x0F

/*!
    \brief EXE_VALD Min Value
*/
#define F021_EXE_VALD_MIN    0x02

/*!
    \brief PROG_PUL_WIDTH Max Value
*/
#define F021_PROG_PUL_WIDTH_MAX    0xFFFF

/*!
    \brief PROG_PUL_WIDTH Min Value
*/
#define F021_PROG_PUL_WIDTH_MIN    0x0002

/*!
    \brief ERA_PUL_WIDTH Max Value
*/
#define F021_ERA_PUL_WIDTH_MAX    0xFFFFFFFF

/*!
    \brief ERA_PUL_WIDTH Min Value
*/
#define F021_ERA_PUL_WIDTH_MIN    0x00000002

/*!
 *  FMC memory map defines
 */

#if defined (_C28X) //C28x Memory map
	#define F021_FLASH_MAP_BEGIN     0x00100000
	#define F021_FLASH_MAP_END       0x0013FFFF
	#define F021_OTP_MAP_BEGIN       0x00240400 //Customer OTP start
	#define F021_OTP_MAP_END         0x002407FF //Customer OTP End
	#define F021_OTPECC_MAP_BEGIN    0x00280080
	#define F021_OTPECC_MAP_END      0x002800FF
	#define F021_FLASHECC_MAP_BEGIN  0x00200000
	#define F021_FLASHECC_MAP_END    0x00207FFF
	#define F021_EEPROM_MAP_BEGIN    0xFFFFFFFF
	#define F021_EEPROM_MAP_END      0xFFFFFFFF
	#define F021_EEPROMECC_MAP_BEGIN 0xFFFFFFFF
	#define F021_EEPROMECC_MAP_END   0xFFFFFFFF
#else //M3 memory map
/*  Users have to uncomment the necessary #defines and comment the unnecessary #defines below based on their selected device */
/*  By default #defines for F28M35x max Flash configuration are uncommented and rest are commented */

/*  Below defines are for maximum Flash configuration F28M35x Master subsystem */
	#define F021_FLASH_MAP_BEGIN     0x00200000
	#define F021_FLASH_MAP_END       0x0027FFFF
	#define F021_OTP_MAP_BEGIN       0x00680800 //Customer OTP start
	#define F021_OTP_MAP_END         0x00680FFF //Customer OTP End
	#define F021_OTPECC_MAP_BEGIN    0x00700100
	#define F021_OTPECC_MAP_END      0x007001FF
	#define F021_FLASHECC_MAP_BEGIN  0x00600000
	#define F021_FLASHECC_MAP_END    0x0060FFFF
	#define F021_EEPROM_MAP_BEGIN    0xFFFFFFFF
	#define F021_EEPROM_MAP_END      0xFFFFFFFF
	#define F021_EEPROMECC_MAP_BEGIN 0xFFFFFFFF
	#define F021_EEPROMECC_MAP_END   0xFFFFFFFF

/*  Below defines are for maximum Flash configuration F28M36x Master subsystem */
/*
	#define F021_FLASH_MAP_BEGIN     0x00200000
	#define F021_FLASH_MAP_END       0x002FFFFF
	#define F021_OTP_MAP_BEGIN       0x00681000 //Customer OTP start
	#define F021_OTP_MAP_END         0x00681FFF //Customer OTP End
	#define F021_OTPECC_MAP_BEGIN    0x00700200
	#define F021_OTPECC_MAP_END      0x007003FF
	#define F021_FLASHECC_MAP_BEGIN  0x00600000
	#define F021_FLASHECC_MAP_END    0x0061FFFF
	#define F021_EEPROM_MAP_BEGIN    0xFFFFFFFF
	#define F021_EEPROM_MAP_END      0xFFFFFFFF
	#define F021_EEPROMECC_MAP_BEGIN 0xFFFFFFFF
	#define F021_EEPROMECC_MAP_END   0xFFFFFFFF
*/
#endif

/*!
    \brief Define to map the direct access to the FMC registers.
*/
#if defined (_C28X) //C28x Memory map
	#define F021_CPU0_REGISTER_ADDRESS 0x00004000
#else //M3 memory map
	#define F021_CPU0_REGISTER_ADDRESS 0x400FA000
#endif

/*!
 *  Specific TI OTP Offsets
 */
#if defined (_C28X) //C28x
	#define F021_TIOTP_PER_BANK_SIZE 0x800 //Even though TI OTP is 2K, Size is mentioned here as 0x800 (4K) including the Customer OTP because code uses this to find the offset of TI OTP relative to OTP Base address
	#define F021_TIOTP_SETTINGS_BASE 0xA8
	#define F021_TIOTP_BANK_SECTOR_OFFSET 0xAC
#else //M3
	#define F021_TIOTP_PER_BANK_SIZE 0x1000 //Even though TI OTP is 2K, Size is mentioned here as 0x1000 (4K) including the Customer OTP because code uses this to find the offset of TI OTP relative to OTP Base address
	                                        //F021_TIOTP_PER_BANK_SIZE mentioned here is for F28M35x device but not for F28M36x device
	                                        //Since F28M35x and F28M36x devices have only one bank for each core, this value does not have to be updated
	#define F021_TIOTP_SETTINGS_BASE 0x150
	#define F021_TIOTP_BANK_SECTOR_OFFSET 0x158
#endif

#endif /* F021_CONSTANTS_CONCERTO_H_ */
