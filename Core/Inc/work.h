//-----------------------------------------------------------------------------
//! \file       work.h
//! \author     R. Weimar
//! \brief      Contains routines for the work module
//! \details
//! \Attention
//-----------------------------------------------------------------------------
#ifndef INC_WORK_H_
#define INC_WORK_H_
//-----------------------------------------------------------------------------
#define LED_SPI 1
//! \brief  includes
#include "defines.h"
#include <stdint.h>

#define SW_VER_MAJOR	2   //0 - 15
#define SW_VER_MINOR	2   //0 - 15
#define SW_VER_BUGFIX	3   //0 - 255
#define	STAYINBOOTLOADER 0
//-----------------------------------------------------------------------------
uint8_t WRK_HandleCopyBlocks (uint32_t newNrOfBlocks, uint32_t newSourceAddress, uint32_t newDestinationAddress);
void FLA_HandleCorrupt (void);
void FLA_HandleRollBack(void);
void FLA_HandleUpdated(void);
void FLA_HandleUpdating(void);
//-----------------------------------------------------------------------------
extern uint8_t Initialized;
extern volatile uint8_t Flg1ms;
extern uint16_t Progress;
extern int32_t ProgressOld;
extern uint32_t WRK_GetSoftwareVersion (void);
extern void WRK_HandleTickTime (void);
extern void WRK_Handle (void);
extern void WRK_StartApplication(void);
#endif /* INC_WORK_H_ */

