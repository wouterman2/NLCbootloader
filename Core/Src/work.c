//-----------------------------------------------------------------------------
//! \file       work.c
//! \author     R. Weimar. RMB Holland bv
//! \brief      Contains routines for the general sequence
//! \details
//! \Attention
//! \Created on: May 5, 2022
//-----------------------------------------------------------------------------

#include "work.h"
#include "serial.h"
#include "flash.h"
#include "eeprom.h"
#include "led.h"
#include "spi.h"
#include "iwdg.h"
#include <string.h>

//! \Global tick time flag
volatile uint8_t Flg1ms = 0;
//! \Global flag to indicate all modules are initialized
uint8_t Initialized;
//! \Global progress indicator
uint16_t Progress;
int32_t ProgressOld;
//-----------------------------------------------------------------------------
//! \brief      Handles the tick time counter
//! \details    Handles 1 ms time interrupt
//! \param      None
void WRK_HandleTickTime (void)
{
	if (!Initialized) return;
	HAL_IWDG_Refresh(&hiwdg);
	SER_Handle();
	LED_Handle();
}
//-----------------------------------------------------------------------------
//! \brief      Copies blocks of data from one address to another
//! \details    Arranges the erase of the page to write to.
//! \param[in]  uint32_t newNrOfBlocks, uint32_t newSourceAddress, uint32_t newDestinationAddress
//! \param[out] Result  1 = OK, 2 = Error
uint8_t WRK_HandleCopyBlocks (uint32_t newNrOfBlocks, uint32_t newSourceAddress, uint32_t newDestinationAddress)
{
	uint8_t Result = 0;
	uint8_t DataLength;
	uint32_t CurrentSourceAddress;
	uint32_t CurrentDestinationAddress;
	Progress = 0;
	ProgressOld = -1;
	for (CurrentBlock = 0; CurrentBlock < newNrOfBlocks  ;CurrentBlock++)
	{
		HAL_IWDG_Refresh(&hiwdg);
		CurrentSourceAddress = newSourceAddress + (CurrentBlock * FLA_BLOCKSIZE);
		CurrentDestinationAddress = newDestinationAddress + (CurrentBlock * FLA_BLOCKSIZE);
		//Check if a page needs to be erased
		if (CurrentBlock % FLA_BLOCKSPERPAGE == 0)//new page
		{
			//Erase the new application page in order to be able to write to it
			if (FLA_ErasePage (CurrentDestinationAddress) != HAL_OK)
			{
			  SER_SendReturnMessage (10,MStatErasePageError);
			  CurrentBlock = 65535;
			}
		}
		//Clear the array to zero before filling the next block
		memset(ImageData, 0, sizeof(ImageData)); //Reset the buffer
		FLA_ReadBlock (CurrentSourceAddress);
		//Write the block to the destinationaddress
		FLA_WriteBlock (CurrentDestinationAddress);
		//Verify the written page
		if (FLA_VerifyBlock (CurrentDestinationAddress) == 0)
		{
			SER_SendReturnMessage (10,MStatVerifyError);
			CurrentBlock = 65535;
		}
		else
		{
			Progress = ((CurrentBlock + 1) * 100) / newNrOfBlocks;
			if (Progress != ProgressOld)
			{
				ProgressOld = Progress;
				if (newSourceAddress == FLA_MAINADDRESS) //Backup
					DataLength = SER_FillBuffer(10, MTypeGetBackupProgress);
				else if (newSourceAddress == FLA_ROLLBACKADDRESS) //Restore
					DataLength = SER_FillBuffer(10, MTypeGetRestoreProgress);
				else
					DataLength = SER_FillBuffer(10, MTypeGetUpdateProgress);
				UartReturn = 2; //Select UART2
				SER_SendReturnMessage (DataLength + 10,MStatSuccess);
			}
		}
	}
	Result = (CurrentBlock == 65535) + 1; //1 = OK, 2 = Error
	return Result;
}
//-----------------------------------------------------------------------------
//! \brief      Handles the situation when the Application is CORRUPT
//! \details    Checks if there is a VALID ROLLBACK partition. If so,
//! \details	copies ROLLBACK to APP partition, otherwise remains
//! \details	in bootloader and red light blinking.
//! \param      None
void FLA_HandleCorrupt (void)
{
	LED_Set(STANDBYLED, BLINKING, 255, 0, 0, 50, 100, 100, 5, 0, 0); //Blinking fast RED
}
//-----------------------------------------------------------------------------
//! \brief      Handles the situation when the Application status is ROLLBACK
//! \details    Checks if there is a VALID ROLLBACK available. If so,
//! \details	Copies the current ROLLBACK to the APP partition,
//! \details	Copies the UPDATE to the APP partition, starts application
//! \param      None
void FLA_HandleRollBack(void)
{
	uint8_t Result = 0;
	uint8_t DataLength;
	if (FLA[pRollBack].Status != fVALID) //RollBack image is not valid. Abort Roll back
	{
		//Set Bootloader to ERROR state
		BootloaderStatus = sUNITERROR;
		//Rollback is canceled. Set Application status to VALID again. Otherwise next reboot, the upgrade will be tried again.
		if (FLA[pApplication].Version > 0)
			FLA_SetStatus(pApplication,fVALID);
		//Send UART message
		DataLength = SER_FillBuffer(10, MTypeGetBootloaderStatus);
		SER_SendReturnMessage (DataLength + 10,MStatImageNotValid);
		WRK_StartApplication(); //Start the old application
		LED_Set(STANDBYLED, BLINKING, 255, 0, 0, 50, 100, 100, 5, 0, 0); //Blinking fast RED
	}
	else
	{
		DataLength = SER_FillBuffer(10, MTypeGetBootloaderStatus);
		SER_SendReturnMessage (DataLength + 10,MStatNone);
		LED_Set(STANDBYLED, SWITCHEDON, 255, 0, 0, 50, 500, 500, 5, 0, 0); //On RED
		LED_Handle();
		//Perform the copy of the image
		//Backup the current application
		Result = WRK_HandleCopyBlocks (FLA[pRollBack].NrOfBlocks, FLA_ROLLBACKADDRESS, FLA_MAINADDRESS);
		if (Result == 1)
		{
			//Set Bootloader to IDLE state
			BootloaderStatus = sIDLE;
			//Write the application status
			FLA_SetStatus(pApplication, fUPDATED);
			FLA_SetVersion(pApplication, FLA[pRollBack].Version);
			//Send the status
			DataLength = SER_FillBuffer(10, MTypeGetBootloaderStatus);
			SER_SendReturnMessage (DataLength + 10,MStatNone);
			//Send ready message for Kumkeo
			uint8_t Temp[] = "/10A291D01";
			HAL_UART_Transmit(&huart2, Temp ,sizeof(Temp),100);
			WRK_StartApplication(); //Try the application
		}
		else
		{
			//Corrupted application. Rollback failed. Stay in bootloader
			FLA_SetStatus(pApplication,fCORRUPT);
			FLA_SetVersion(pApplication,0);
			LED_Set(STANDBYLED, BLINKING, 255, 0, 0, 50, 100, 100, 5, 0, 0); //Blinking fast RED
		}
	}
}
//-----------------------------------------------------------------------------
//! \brief      Handles the situation when the Application is UPDATING
//! \details    Checks if there is a VALID UPDATE available. If so,
//! \details	Copies the current APP to the rollback partition,
//! \details	Copies the UPDATE to the APP partition, starts application
//! \param      None
void FLA_HandleUpdating(void)
{
	uint8_t Result = 0;
	uint8_t DataLength;
	if (FLA[pUpdate].Status != fVALID) //Upgrade image is not valid. Abort upgrade
	{
		LED_Set(STANDBYLED, BLINKING, 255, 0, 0, 50, 100, 100, 5, 0, 0); //Blinking fast RED
		//Set Bootloader to ERROR state
		BootloaderStatus = sUNITERROR;
		//Upgrade is canceled. Set Application status to VALID again. Otherwise next reboot, the upgrade will be tried again.
		if (FLA[pApplication].Version > 0)
			FLA_SetStatus(pApplication,fVALID);
		//Send UART message
		DataLength = SER_FillBuffer(10, MTypeGetBootloaderStatus);
		SER_SendReturnMessage (DataLength + 10,MStatImageNotValid);
		WRK_StartApplication(); //Start the old application
		LED_Set(STANDBYLED, BLINKING, 255, 0, 0, 50, 100, 100, 5, 0, 0); //Blinking fast RED
	}
	else
	{
		DataLength = SER_FillBuffer(10, MTypeGetBootloaderStatus);
		SER_SendReturnMessage (DataLength + 10,MStatNone);
		LED_Set(STANDBYLED, SWITCHEDON, 0, 0, 255, 50, 500, 500, 5, 0, 0); //On BLUE
		LED_Handle();
		//Perform the copy of the image
		//Backup the current application if present. If not, skip this step
		if ((FLA[pApplication].Version > 0) && (((*(__IO uint32_t*) FLA_MAINADDRESS) & 0x2FFE0000) == 0x20000000))//Application is present
		{
			FLA_SetStatus(pRollBack, fUPDATING);
			Result = WRK_HandleCopyBlocks (FLA_LENGTH/FLA_BLOCKSIZE, FLA_MAINADDRESS, FLA_ROLLBACKADDRESS);
			if (Result == 1) //Copy succeeded. Set Statuses for rollback version
			{
				FLA_SetStatus(pRollBack, fVALID);
				FLA_SetVersion(pRollBack, FLA[pApplication].Version);
			}
			else
			{
				//Corrupted application. Stay in bootloader
				FLA_SetStatus(pRollBack,fCORRUPT);
				FLA_SetVersion(pRollBack,0);
				LED_Set(STANDBYLED, BLINKING, 255, 0, 0, 50, 100, 100, 5, 0, 0); //Blinking fast RED
			}
		}
		else
		{
			Result = 1;
		}
		//Do the actual reading and writing
		if (Result == 1) //Copy succeeded. Copy the application
		{
			Result = WRK_HandleCopyBlocks (FLA[pUpdate].NrOfBlocks, FLA_UPGRADEADDRESS, FLA_MAINADDRESS);
			if (Result == 1)
			{
				LED_Set(STANDBYLED, SWITCHEDON, 0, 0, 255, 50, 500, 500, 5, 0, 0); //On BLUE
				//Set Bootloader to IDLE state
				BootloaderStatus = sIDLE;
				//Write the application status
				FLA_SetStatus(pApplication, fUPDATED);
				FLA_SetVersion(pApplication, FLA[pUpdate].Version);
				//Send the status
				HAL_Delay(100);
				DataLength = SER_FillBuffer(10, MTypeGetBootloaderStatus);
				SER_SendReturnMessage (DataLength + 10,MStatSuccess);
				//Send ready message for Kumkeo
				uint8_t Temp[] = "/10A22F101";
				HAL_UART_Transmit(&huart2, Temp ,sizeof(Temp),100);
				WRK_StartApplication(); //Try the application
			}
			else
			{
				//Corrupted application. Set rollback if available and restart the system
				if (FLA[pRollBack].Status == fVALID)
				{
					FLA_SetStatus(pApplication,fROLLBACK);
					FLA_HandleRollBack();
				}
				else //Corrupted application. No Roll back available. Stay in Bootloader
				{
					FLA_SetStatus(pApplication,fCORRUPT);
					FLA_SetVersion(pApplication,0);
					LED_Set(STANDBYLED, BLINKING, 255, 0, 0, 50, 100, 100, 5, 0, 0); //Blinking fast RED
				}
			}
		}
		else
		{
			//Saving of roll back partition failed.
			DataLength = SER_FillBuffer(10, MTypeGetBootloaderStatus);
			SER_SendReturnMessage (DataLength + 10,MStatCreateRollBackError);
			//Set Application status to VALID again. Otherwise next reboot, the upgrade will be tried again.
			if (FLA[pApplication].Version > 0)
				FLA_SetStatus(pApplication,fVALID);
			WRK_StartApplication(); //Try to start the old application
		}
	}
}

//-----------------------------------------------------------------------------
//! \brief      Handles the situation when the Application status is UPDATED
//! \details    This happens only if the application couldnot start -> Roll back
//! \details    Checks if there is a VALID ROLLBACK available. If so,
//! \details	Copies the current ROLLBACK to the APP partition,
//! \details	Copies the UPDATE to the APP partition, starts application
//! \param      None
void FLA_HandleUpdated(void)
{
	uint8_t Result = 0;
	uint8_t DataLength;
	if (FLA[pRollBack].Status == fVALID)
	{
		LED_Set(STANDBYLED, SWITCHEDON, 255, 0, 0, 50, 500, 500, 5, 0, 0); //On RED
		LED_Handle();
		Result = WRK_HandleCopyBlocks (FLA_LENGTH/FLA_BLOCKSIZE, FLA_ROLLBACKADDRESS, FLA_MAINADDRESS); //Handle retry and or roll back
		if (Result == 1)
		{
			//Set Bootloader to IDLE state
			BootloaderStatus = sIDLE;
			//Write the application status
			FLA_SetStatus(pApplication, fUPDATED);
			FLA_SetVersion(pApplication, FLA[pRollBack].Version);
			//Send the status
			DataLength = SER_FillBuffer(10, MTypeGetBootloaderStatus);
			SER_SendReturnMessage (DataLength + 10,MStatNone);
			WRK_StartApplication(); //Try the application
		}
		else
		{
			//Corrupted application. Stay in bootloader
			FLA_SetStatus(pApplication,fCORRUPT);
			FLA_SetVersion(pApplication,0);
			LED_Set(STANDBYLED, BLINKING, 255, 0, 0, 50, 100, 100, 5, 0, 0); //Blinking fast RED
		}
	}
	else
	{
		//Corrupted application. Stay in bootloader
		FLA_SetStatus(pApplication,fCORRUPT);
		FLA_SetVersion(pApplication,0);
		LED_Set(STANDBYLED, BLINKING, 255, 0, 0, 50, 100, 100, 5, 0, 0); //Blinking fast RED
	}
}
//-----------------------------------------------------------------------------
//! \brief      Handles the startup of the system
//! \details    Checks if there is an update or ollback pending. If not, starts the application
//! \details    If there is no valid application will be ready to receive UART commands
//! \param      None
void WRK_Handle(void)
{
	uint8_t DataLength;
	BootloaderStatus = sACTIVE;
	DataLength = SER_FillBuffer(10, MTypeGetBootloaderStatus);
	SER_SendReturnMessage (DataLength + 10,MStatNone);
	if (FLA[pApplication].Status == fCORRUPT)
	{
		//Corrupted application. Stay in bootloader
		FLA_HandleCorrupt();
	}
	else if (FLA[pApplication].Status == fUPDATING)
	{
		FLA_HandleUpdating();

	}
	else if (FLA[pApplication].Status == fROLLBACK)
	{
		FLA_HandleRollBack();
	}
	else if (FLA[pApplication].Status == fUPDATED)//Application didn't start and change status to VALID, so roll back
	{
		FLA_HandleUpdated();
	}
	else if (FLA[pApplication].Status == fVALID)//Normal startup
	{
		WRK_StartApplication();
	}
	else if (FLA[pApplication].Status == fUNDEFINED)//EEprom is empty at startup
	{
		WRK_StartApplication();
	}
	else //Cannot start, because there is no valid partition. Stay in bootloader, so an application can be loaded
	{
		LED_Set(STANDBYLED, BLINKING, 255, 0, 0, 50, 100, 100, 5, 0, 0); //Blinking fast RED
	}

}
//-----------------------------------------------------------------------------
//! \brief      Returns the software version
//! \details    Makes 1 integer of the software version that can be sent by UART
//! \params		None
uint32_t WRK_GetSoftwareVersion (void)
{
	return (SW_VER_MAJOR << 12) + (SW_VER_MINOR << 8) + SW_VER_BUGFIX;
}
//-----------------------------------------------------------------------------
//! \brief      Starts the application
//! \details    Sets the stack pointer to the application partition
//! \param      None
void WRK_StartApplication(void)
{
	uint8_t DataLength;
	//Stay in bootloader if flag is set
	if (STAYINBOOTLOADER) return;
	//Stay in bootloader if no VALID app is present.
	if (FLA[pApplication].Version == 0)
	{
		FLA_SetStatus(pApplication,fUNDEFINED);
		LED_Set(STANDBYLED, BLINKING, 255, 0, 0, 50, 100, 100, 5, 0, 0); //Blinking fast RED
		//Inform the Linux
		DataLength = SER_FillBuffer(10, MTypeGetBootloaderStatus);
		SER_SendReturnMessage (DataLength + 10,MStatImageNotValid);
		return;
	}
	//Stay in bootloader if no valid image is found at APPLICATION memory address
	if (((*(__IO uint32_t*) FLA_MAINADDRESS) & 0x2FFE0000) != 0x20000000)
	{
		//No valid application found, so unvalidate version and status
		FLA_SetStatus(pApplication,fUNDEFINED);
		FLA_SetVersion(pApplication,0);
		//Inform the Linux
		DataLength = SER_FillBuffer(10, MTypeGetBootloaderStatus);
		SER_SendReturnMessage (DataLength + 10,MStatImageNotValid);
		LED_Set(STANDBYLED, BLINKING, 255, 0, 0, 50, 100, 100, 5, 0, 0); //Blinking fast RED
		return;
	}
    //just a function pointer to hold the address of the reset handler of the user app.
    void (*app_reset_handler)(void);
    //Deinit all
    HAL_RCC_DeInit();
    HAL_SPI_MspDeInit(&hspi1);
    HAL_DeInit();
    //Disable systick interrupts
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    //disable interrupts
    __set_PRIMASK(1);
    __disable_irq();
    //Set vector offset
     SCB->VTOR = FLA_MAINADDRESS;
    //Set MSP address
    uint32_t msp_value = *(__IO uint32_t *)FLA_MAINADDRESS;
    __set_MSP(msp_value);
    //Set reset handler address
    uint32_t resethandler_address = *(__IO uint32_t *) (FLA_MAINADDRESS + 4);
    app_reset_handler = (void*) resethandler_address;
    //Jump to reset handler of the application
    app_reset_handler();
}
