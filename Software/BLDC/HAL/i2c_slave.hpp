
#ifndef HAL_I2C_SLAVE_HPP_
#define HAL_I2C_SLAVE_HPP_

#include "stm32f3xx_hal.h"

class I2CSlave {
public:
	using Callback = void (*)(void *p);

	I2CSlave(I2C_TypeDef *interface, uint8_t ownAddress);
	void SetReadBase(void *read, uint16_t size);
	void SetWriteBase(void *write, uint16_t size);
	void SetCallback(Callback cb, void *p);

	void EventInterrupt();
private:
	I2C_TypeDef *i2c;
	uint8_t *read;
	uint8_t *write;
	uint16_t readSize, writeSize;
	int16_t MemoryAddress;
	Callback callback;
	void *ptr;
	bool callbackDue;
};

#endif
