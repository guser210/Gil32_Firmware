/*
 * eeprom.cpp
 *
 *  Created on: Aug 7, 2025
 *      Author: Gil Vargas
 */

#include "main.h"
#include "eeprom.h"
#include "string.h"

#define LOCAL_FLASH_KEY1 0x45670123;
#define LOCAL_FLASH_KEY2 0xCDEF89AB;


void read_memory(uint8_t *data, uint16_t size, uint32_t address)
{
	if( address < APPLICATION_ADDRESS)
		return;

	uint32_t pEEPROM =  (BASE_ADDRESS + address);
	memcpy((void*)data,(const void**)pEEPROM, size);
}

void wait_flash_is_busy(void)
{
	while(FLASH->SR & FLASH_SR_BSY1){}
}
void unlock_flash(void)
{
	wait_flash_is_busy();
	if( FLASH->CR & FLASH_CR_LOCK)
	{
		FLASH->KEYR = LOCAL_FLASH_KEY1;
		FLASH->KEYR = LOCAL_FLASH_KEY2;
		wait_flash_is_busy();
	}
}
void lock_flash(void)
{
	FLASH->CR |= FLASH_CR_LOCK;
	wait_flash_is_busy();
}
void erase_page(uint8_t page)
{
	FLASH->SR |= 0xC3FB; // clear all errors.

	FLASH->CR &= ~FLASH_CR_PNB; // Clear page settings.
	FLASH->CR |= FLASH_CR_PER; 	// Page erase enabled.
	FLASH->CR |= page<<FLASH_CR_PNB_Pos; // Set page to erase.
	FLASH->CR |= FLASH_CR_STRT; // Start erase.

	wait_flash_is_busy();

	FLASH->CR &= ~FLASH_CR_PER; // Page erase disabled.

	FLASH->SR |= 0xC3FB; // clear all errors.

}


void write_memory(uint8_t* data,const uint16_t size,const uint32_t address){

	if( address < APPLICATION_ADDRESS)
		return;

	const uint8_t  PAGE 		= (address / PAGE_SIZE);
	const uint8_t  ERASE_PAGE 	= !(address % PAGE_SIZE);

	const uint8_t SIZE_DIV_BY4 = (size / WORD_SIZE);

	uint32_t memorySettings[SIZE_DIV_BY4] = {};

	memcpy((void*)&memorySettings,(const void*)data,size);

	unlock_flash();

	if( ERASE_PAGE )
	{
		erase_page(PAGE);
	}


	FLASH->SR |= 0xC3FB; // clear all errors.

	for( uint16_t index = 0; index < (SIZE_DIV_BY4); index +=2){
		FLASH->CR |= FLASH_CR_PG; // Flash programming enabled.

    	*(__IO uint32_t*)((BASE_ADDRESS + address) + (index * WORD_SIZE)) = memorySettings[index];
    	*(__IO uint32_t*)((BASE_ADDRESS + address) + ((index + 1) * WORD_SIZE)) = memorySettings[index+1];

    	while(FLASH->SR & FLASH_SR_BSY1){}

    	if( FLASH->SR & FLASH_SR_EOP)
		{
			FLASH->SR |= FLASH_SR_EOP;
		}

    	FLASH->SR |= 0xC3FB; // clear all errors.
    	FLASH->CR &= ~FLASH_CR_PG; // Flash programming enabled.
	}

	lock_flash();

}
