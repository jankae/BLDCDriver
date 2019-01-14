#include "i2c_slave.hpp"

I2CSlave::I2CSlave(I2C_TypeDef* interface, uint8_t ownAddress) {
	i2c = interface;
	i2c->OAR1 &= ~I2C_OAR1_OA1EN;
	i2c->OAR1 = ownAddress;
	i2c->OAR1 |= I2C_OAR1_OA1EN;
	MemoryAddress = 0;
	read = write = nullptr;
	readSize = 0;
	writeSize = 0;
	callback = nullptr;
	ptr = nullptr;
	callbackDue = false;
	// enable address match interrupt
	i2c->CR1 |= I2C_CR1_ADDRIE | I2C_CR1_TXIE | I2C_CR1_RXIE | I2C_CR1_NACKIE
			| I2C_CR1_STOPIE;
    // disable NACK
    i2c->CR2 &= ~I2C_CR2_NACK;
	// enable periperal
	i2c->CR1 |= I2C_CR1_PE;
}

void I2CSlave::SetReadBase(void* read, uint16_t size) {
	this->read = (uint8_t*) read;
	readSize = size;
}

void I2CSlave::SetWriteBase(void* write, uint16_t size) {
	this->write = (uint8_t*) write;
	writeSize = size;
}

void I2CSlave::SetCallback(Callback cb, void *p) {
	callback = cb;
	ptr = p;
}

void I2CSlave::EventInterrupt() {
	if (i2c->ISR & I2C_ISR_ADDR) {
		if (i2c->ISR & I2C_ISR_DIR) {
			// slave transmitter, set TXE according to datasheet
			i2c->ISR |= I2C_ISR_TXE;
		} else {
			MemoryAddress = -1;
		}
		// clear flag
		i2c->ICR = I2C_ICR_ADDRCF;
	}
	if (i2c->ISR & I2C_ISR_RXNE) {
		uint8_t data = i2c->RXDR;
		if (MemoryAddress == -1) {
			MemoryAddress = data;
		} else if (MemoryAddress < writeSize) {
			write[MemoryAddress++] = data;
			callbackDue = true;
		}
	}
	if (i2c->ISR & I2C_ISR_TXIS) {
		if (MemoryAddress < readSize) {
			i2c->TXDR = read[MemoryAddress++];
		} else {
			i2c->TXDR = 0;
		}
	}
	if (i2c->ISR & I2C_ISR_STOPF) {
		// clear flag
		i2c->ICR = I2C_ICR_STOPCF;
		if (callbackDue) {
			if (callback) {
				callback(ptr);
			}
			callbackDue = false;
		}
	}
}
