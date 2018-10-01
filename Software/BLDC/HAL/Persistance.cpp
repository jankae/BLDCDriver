#include "Persistance.hpp"

#include "stm32f3xx_hal.h"

extern uint32_t _sflashpersist;
extern uint32_t _sccmpersist;
extern uint32_t _eccmpersist;

void Persistance::Load() {
	uint32_t *from = &_sflashpersist;
	for (uint32_t* i = &_sccmpersist; i != &_eccmpersist; i++) {
		*i = *from++;
	}
}

void Persistance::Store() {
	FLASH_EraseInitTypeDef erase;
	erase.NbPages = 1;
	erase.PageAddress = _sflashpersist;
	erase.TypeErase = FLASH_TYPEERASE_PAGES;

	uint32_t error;
	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&erase, &error);

	uint32_t *to = &_sflashpersist;
	for (uint32_t *i = &_sccmpersist; i != &_eccmpersist; i++) {
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t) to++, *i);
	}

	HAL_FLASH_Lock();
}
