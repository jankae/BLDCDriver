#include "Persistance.hpp"

#include "stm32f3xx_hal.h"
#include "Logging.hpp"

extern uint32_t _sflashpersist;
extern uint32_t _sccmpersist;
extern uint32_t _eccmpersist;

void Persistance::Load() {
	uint32_t *from = &_sflashpersist;
	for (uint32_t* i = &_sccmpersist; i != &_eccmpersist; i++) {
		*i = *from++;
	}
}

bool Persistance::Store() {
	FLASH_EraseInitTypeDef erase;
	erase.NbPages = 1;
	erase.PageAddress = &_sflashpersist;
	erase.TypeErase = FLASH_TYPEERASE_PAGES;

	uint32_t error;
	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&erase, &error);
	if(error != 0xFFFFFFFF) {
		Log::Uart(Log::Lvl::Err, "Failed to erase flash");
		HAL_FLASH_Lock();
		return false;
	}

	uint32_t *to = &_sflashpersist;
	for (uint32_t *i = &_sccmpersist; i != &_eccmpersist; i++) {
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t) to, *i) != HAL_OK) {
			Log::Uart(Log::Lvl::Err,
					"Failed to write data: from %p to %p, value %lu", i, to,
					*i);
			HAL_FLASH_Lock();
			return false;
		}
		to++;
	}

	HAL_FLASH_Lock();

	// verify written content
	uint32_t *flash = &_sflashpersist;
	for (uint32_t* i = &_sccmpersist; i != &_eccmpersist; i++) {
		if (*i != *flash) {
			Log::Uart(Log::Lvl::Err,
					"Peristance verify error: %p (%d) vs %p (%d)", i, *i, flash,
					*flash);
			return false;
		}
		flash++;
	}

	Log::Uart(Log::Lvl::Inf, "Persistance stored in flash");
	return true;
}
