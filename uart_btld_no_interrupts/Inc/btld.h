#include "main.h"

#define FLASH_FKEY1 0x45670123
#define FLASH_FKEY2 0xCDEF89AB


void hal_independent_flash_unlock ();
void hal_independent_erase_flash_sector();
void hal_independent_write_flash_32(uint32_t Address, uint32_t Data);
void hal_independent_write_flash_8(uint32_t Address, uint8_t Data);
void hal_independent_flash_lock();
