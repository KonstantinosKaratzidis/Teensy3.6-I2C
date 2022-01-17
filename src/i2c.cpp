#include <Arduino.h>
#include <kinetis.h>
#include "../include/i2c.h"
#include <FreeRTOS.h>
#include <semphr.h>
#include <uart.h>

#define I2C_FLT_SSIE (1 << 5)
#define I2C_FLT_STARTF (1 << 4)

void I2C::begin(){
	// initilialize the mutex
	lock_mutex = xSemaphoreCreateBinary();
	wait_sem = xSemaphoreCreateBinary();
	xSemaphoreGive(lock_mutex);

	SIM_SCGC4 |= SIM_SCGC4_I2C0; // clock the module
	I2C0_C1 = 0;

	// configure sda and scl pins
	PORTB_PCR0 = PORT_PCR_MUX(2) | PORT_PCR_ODE | PORT_PCR_SRE | PORT_PCR_DSE;
	PORTB_PCR1 = PORT_PCR_MUX(2) | PORT_PCR_ODE | PORT_PCR_SRE | PORT_PCR_DSE;
	
	// set clock and filter
	I2C0_F = 0x2C;	// 104 kHz
	I2C0_FLT = 4;

	I2C0_C2 = I2C_C2_HDRS; // high drive strength
	I2C0_C1 = I2C_C1_IICEN | I2C_C1_IICIE; // enable i2c module
	I2C0_FLT |= I2C_FLT_SSIE;
	NVIC_ENABLE_IRQ(IRQ_I2C0);
}

void I2C::end(){
	vSemaphoreDelete(lock_mutex);
	vSemaphoreDelete(wait_sem);
}

bool I2C::write(uint8_t address, const uint8_t *data, int length) {
	if(xSemaphoreTake(lock_mutex, pdMS_TO_TICKS(100)) != pdTRUE){
		return false;
	}

	txBufferIndex = 0;
	txBufferLength = length;
	txAddress = address << 1;
	txBuffer = data;
	transmitting = true;
	writing = true;
	done = false;
	had_error = false;
	address_sent = false;
	
	I2C0_C1 |= I2C_C1_MST | I2C_C1_TX;

	bool ret;
	if(!xSemaphoreTake(wait_sem, pdMS_TO_TICKS(100)))
		ret = false;
	else
		ret = !had_error;

	xSemaphoreGive(lock_mutex);
	return ret;
}

bool I2C::read(uint8_t address, uint8_t *buffer, int length){
	if(!xSemaphoreTake(lock_mutex, pdMS_TO_TICKS(100)))
		return false;

	rxBuffer = buffer;
	rxBufferIndex = 0;
	rxBufferLength = length;
	txAddress = address << 1 | 1;
	transmitting = true;
	writing = false;
	done = false;
	had_error = false;
	address_sent = false;

	I2C0_C1 |= I2C_C1_MST | I2C_C1_TX;

	bool ret;
	if(!xSemaphoreTake(wait_sem, pdMS_TO_TICKS(100)))
		ret = false;
	else
		ret = !had_error;

	xSemaphoreGive(lock_mutex);
	return ret;

}

void I2C::isr(){
	// if stop flag
	if(I2C0_FLT & I2C_FLT_STOPF){
		I2C0_FLT |= I2C_FLT_STOPF;
		I2C0_S |= I2C_S_IICIF;
		done = true;
		xSemaphoreGiveFromISR(wait_sem, NULL);
		return;
	}

	// if start flag
	if(I2C0_FLT & I2C_FLT_STARTF){
		I2C0_FLT |= I2C_FLT_STARTF;
	}

	I2C0_S |= I2C_S_IICIF;

	if(writing){
		if(!address_sent){
			address_sent = true;
			I2C0_D = txAddress;
			return;
		}

		if(I2C0_S & I2C_S_RXAK){ // nack
			I2C0_C1 &= ~(I2C_C1_MST ); // send stop
			had_error = true;
			return;
		}

		if(txBufferIndex == txBufferLength){ // last byte, generate STOP
			I2C0_C1 &= ~(I2C_C1_MST );
		}
		else {
			I2C0_D = txBuffer[txBufferIndex++];
		}
	}
	else if(!writing && transmitting){ // read, writing the address
		if(!address_sent){
			address_sent = true;
			I2C0_D = txAddress;
			return;
		}

		if(I2C0_S & I2C_S_RXAK){ // nack
			I2C0_C1 &= ~(I2C_C1_MST ); // send stop
			had_error = true;
			return;
		}

		transmitting = 0;
		if(rxBufferLength - rxBufferIndex == 0){
			I2C0_C1 &= ~(I2C_C1_MST ); // send stop
			return;
		}
		else if(rxBufferLength - rxBufferIndex == 1){
			I2C0_C1 |= I2C_C1_TXAK; // send nack after receiving next byte
		}
		else{
			I2C0_C1 &= ~I2C_C1_TXAK; // send ack after receiving next byte
		}

		I2C0_C1 &= ~I2C_C1_TX; // switch to rx mode
		volatile uint8_t __attribute__((unused)) dummy;
		dummy = I2C0_D;
		return;
	}
	else if(!writing && !transmitting){ // we are reading
		int remaining = rxBufferLength - rxBufferIndex; // including current
		if(remaining == 2){
			I2C0_C1 |= I2C_C1_TXAK; // send nack after receiving next byte
		}
		else if(remaining == 1){ // we are at the last byte
			I2C0_C1 &= ~(I2C_C1_MST); // send stop
		}
		rxBuffer[rxBufferIndex++] = I2C0_D;
		return;
	}
}

I2C i2c0;

void i2c0_isr(){
	i2c0.isr();
}
