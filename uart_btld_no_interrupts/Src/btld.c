#include "btld.h"



void hal_independent_flash_unlock (){

	// Wait for the flash memory not to be busy
	while ((FLASH->SR & FLASH_SR_BSY) != 0 );
	// Check if the controller is unlocked already
	if ((FLASH->CR & FLASH_CR_LOCK) != 0 ){
		// Write the first key
		FLASH->KEYR = FLASH_FKEY1;
		// Write the second key
		FLASH->KEYR = FLASH_FKEY2;
	}

}

void hal_independent_erase_flash_sector( ){

	FLASH->CR |= FLASH_CR_SER; // Page erase operation
	FLASH->CR |= FLASH_CR_SNB_0;     // Set the address to the page to be written
	FLASH->CR |= FLASH_CR_STRT;// Start the page erase

	// Wait until page erase is done
	while ((FLASH->SR & FLASH_SR_BSY) != 0);
	// If the end of operation bit is set...
	if ((FLASH->SR & FLASH_SR_EOP) != 0){
	    // Clear it, the operation was successful
	    FLASH->SR |= FLASH_SR_EOP;
	}
	//Otherwise there was an error
	else{
	    // Manage the error cases
	}
	// Get out of page erase mode
	FLASH->CR &= ~FLASH_CR_SER;

}

void hal_independent_write_flash_32(uint32_t Address, uint32_t Data){

	FLASH->CR |= (FLASH_CR_PG | FLASH_PSIZE_WORD);     // Programing mode 32 bits

	*(__IO uint32_t*)(Address) = Data;       // Write data

	// Wait until the end of the operation
	while ((FLASH->SR & FLASH_SR_BSY) != 0);
	// If the end of operation bit is set...
	if ((FLASH->SR & FLASH_SR_EOP) != 0){
	    // Clear it, the operation was successful
	     FLASH->SR |= FLASH_SR_EOP;
	}
	//Otherwise there was an error
	else{
	    // Manage the error cases
	}
	FLASH->CR &= ~FLASH_CR_PG;

}

void hal_independent_write_flash_8(uint32_t Address, uint8_t Data){

	FLASH->CR |= (FLASH_CR_PG );     // Programing mode 32 bits

	*(__IO uint8_t*)(Address) = Data;       // Write data

	// Wait until the end of the operation
	while ((FLASH->SR & FLASH_SR_BSY) != 0);
	// If the end of operation bit is set...
	if ((FLASH->SR & FLASH_SR_EOP) != 0){
	    // Clear it, the operation was successful
	     FLASH->SR |= FLASH_SR_EOP;
	}
	//Otherwise there was an error
	else{
	    // Manage the error cases
	}
	FLASH->CR &= ~FLASH_CR_PG;

}


void hal_independent_flash_lock(){

	FLASH->CR |= FLASH_CR_LOCK;

}
