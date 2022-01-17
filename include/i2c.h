#ifndef TEENSY36_I2C_H
#define TEENSY36_I2C_H
#include <stdbool.h>
#include <stdint.h>
#include <FreeRTOS.h>
#include <semphr.h>

class I2C {
public:
	void begin();
	void end();
	bool write(uint8_t address, const uint8_t *data, int length);
	bool read(uint8_t address, uint8_t *buffer, int length);

private:
	void isr();
	friend void i2c0_isr();

	uint8_t txAddress;

	SemaphoreHandle_t lock_mutex;
	SemaphoreHandle_t wait_sem;

	const uint8_t *txBuffer;
	volatile int txBufferLength;
	volatile int txBufferIndex;

	uint8_t *rxBuffer;
	volatile int rxBufferLength;
	volatile int rxBufferIndex;

	volatile bool transmitting;
	volatile bool writing;
	volatile bool done;
	volatile bool had_error;
	volatile bool address_sent;
};


extern I2C i2c0;
#endif // TEENSY36_I2C_H
