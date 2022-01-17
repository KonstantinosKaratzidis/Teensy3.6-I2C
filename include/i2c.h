#ifndef TEENSY36_I2C_H
#define TEENSY36_I2C_H
#include <stdbool.h>
#include <stdint.h>

class I2C {
public:
	void begin();
	bool write(uint8_t address, const uint8_t *data, int length);
	bool write_to(uint8_t address, uint8_t reg, uint8_t data){
		const uint8_t buffer[2] = {reg, data};
		return write(address, buffer, 2);
	}

	bool read(uint8_t address, uint8_t *buffer, int length);
	bool read_from(uint8_t address, uint8_t reg, uint8_t *buffer, int length){
		if(!write(address, &reg, 1))
			return false;
		return read(address, buffer, length);
	}

private:
	void isr();
	friend void i2c0_isr();

	uint8_t txAddress;

	const uint8_t *txBuffer;
	int txBufferLength;
	int txBufferIndex;

	uint8_t *rxBuffer;
	int rxBufferLength;
	int rxBufferIndex;

	bool transmitting;
	bool writing;
	bool done;
	bool had_error;
	bool address_sent;
};


extern I2C i2c0;
#endif // TEENSY36_I2C_H
