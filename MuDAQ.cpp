/*
 * \brief Library for operating the MuDAQ.
 *
 * This library has all necessary functions for using the ADS124S08
 * and DATAFLASH part on the Arduino MuDAQ.
 * Copyright (c) 2012 by Jonny Dyer <jonny.dyer@gmail.com>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include "pins_arduino.h"
#include "MuDAQ.h"
#include "ADS124S08.h"

#define SPI_MODE_MASK 0x0C  	// CPOL = bit 3, CPHA = bit 2 on SPCR

static inline uint8_t spiTransferByte(uint8_t data);
static inline void setSPIMode(uint8_t mode);

MuDAQClass MuDAQ;

bool MuDAQClass::begin(void)
{
	uint8_t temp;
	
	// Set pin idle states
	digitalWrite(SCK, LOW);
	digitalWrite(MOSI, LOW);
	digitalWrite(SS, HIGH);
	digitalWrite(MISO, LOW);
	digitalWrite(ADS124S08_RESET, HIGH);
	digitalWrite(ADS124S08_START, LOW);
	digitalWrite(STAT_LED_1, LED_OFF);
	
	// Set pin directions
	pinMode(SCK, OUTPUT);
	pinMode(MOSI, OUTPUT);
	pinMode(SS, OUTPUT);
	pinMode(MISO,INPUT);
	pinMode(ADS124S08_RESET,OUTPUT);
	pinMode(ADS124S08_START,OUTPUT);
	pinMode(ADS124S08_DRDY,INPUT);
	pinMode(STAT_LED_1, OUTPUT);
	
	// Init SPI - f_cpu/128
	SPCR = (1<<MSTR) | (1<<SPE) | (0<<SPR1) | (1<<SPR0);
	SPSR |= (1<<SPI2X);				//2x speed
	// clear status by doing a read
	SPSR;SPDR;
	
	// Reset components
	//resetDataFlash();
	reset();
    Serial.print("\nReset");
    dumpRegs();
	temp = readReg(ADS124S08_OFCAL0);
	writeReg(ADS124S08_OFCAL0, 0x05);
	if(readReg(ADS124S08_OFCAL0) == 0x05)
	{
		// Indicate that we are inited
		digitalWrite(STAT_LED_1, LED_ON);
		writeReg(ADS124S08_OFCAL0,temp); 		//Return to reset state
		// Perform self offset calibration
		// selfOffsetCal();
		return true;
	}
	return false;
	
}

void MuDAQClass::end(void)
{
	SPCR &= ~(1<<SPE);
	digitalWrite(STAT_LED_1, LED_OFF);
}

void MuDAQClass::enableIntRef(void)
{
	writeReg(
            ADS124S08_REF,readReg(ADS124S08_REF) | 
            (ADS124S08_REFSEL_INTVREF | ADS124S08_REFCON_INT_ON)
    );
}

/*void MuDAQClass::selectRef(uint8_t ref_mux)
{
	writeReg(ADS124S08_MUX1,(readReg(ADS124S08_MUX1) & ~ADS124S08_REFSEL_MASK) | (ref_mux & ADS124S08_REFSEL_MASK));
}*/

/*void MuDAQClass::selectMuxCal(uint8_t muxcal)
{
	writeReg(ADS124S08_MUX1,(readReg(ADS124S08_MUX1) & ~ADS124S08_MUXCAL_MASK) | (muxcal & ADS124S08_MUXCAL_MASK));
}*/

void MuDAQClass::reset(void)
{
	digitalWrite(ADS124S08_RESET,LOW);
	delay(1);			// Min 4*t_osc
	digitalWrite(ADS124S08_RESET,HIGH);
	delay(1);			// min .6ms
}

/* float sample(uint8_t ch_pos, uint8_t ch_neg, uint8_t gain, uint8_t ref)
 * 
 * Perform a blocking read of one channel pair of the ADC.  Enables the internal reference
 * if not already enabled.
 * TODO: put timeout on DRDY in case the ADS124S08 doesn't return
 */
float MuDAQClass::sample(uint8_t ch_pos, uint8_t ch_neg, uint8_t gain, uint8_t sampleRate)
{
	int32_t res;
	// Enable START to allow writing to registers
	//digitalWrite(ADS124S08_START, HIGH);
    //
    // TODO: reference turn on and check must be done before this function

	// Setup gain and sample rate
    setPGA(gain);
    setSampleRate(sampleRate);
    
    // TODO: This should really be an option.
    // Set sinc filter
    writeReg(ADS124S08_DATARATE, (0xEF & readReg(ADS124S08_DATARATE)) |
            0x00);
	// Choose channel
	writeReg(ADS124S08_INPMUX, (ADS124S08_MUXN_MASK & ch_neg) | ((ADS124S08_MUXP_MASK & ch_pos) << 4));
	// Disable START to reset sample
	//digitalWrite(ADS124S08_START, LOW);
	delayMicroseconds(1);
	// Start conversion
	startSingle();
	// Wait for completion - TODO: put timeout on DRDY in case the ADS124S08 doesn't return
	while(digitalRead(ADS124S08_DRDY) == HIGH){};
	// Retrieve result

	res = readData();
	// Scale result
    // TODO: Removed logic to scale conversion differently for non INTREF.
    // Should replace this if needed.
    return (float)res / ADS124S08_MAX_VAL * 0.001 * ADS124S08_INT_REF_MV;
	
}

void MuDAQClass::startSingle(void)
{
	digitalWrite(ADS124S08_START, HIGH);
	delayMicroseconds(2);				// Datasheet specifies minimum 3*t_osc which would really be < 1us
	digitalWrite(ADS124S08_START, LOW);
}

void MuDAQClass::setPGA(uint8_t gain)
{
	writeReg(ADS124S08_PGA, (ADS124S08_PGA_GAIN_MASK & gain) | ADS124S08_PGA_EN);
}

void MuDAQClass::setSampleRate(uint8_t rate)
{
    writeReg(ADS124S08_DATARATE, (0xF0 & readReg(ADS124S08_DATARATE)) | 
            rate);
}

// len can be no more than 16 bytes
uint8_t MuDAQClass::readReg(uint8_t addr)
{
	uint8_t ret;
	setSPIMode(ADS124S08_SPI_MODE);
	digitalWrite(SS, LOW);  // Pull CS low
	spiTransferByte((addr & 0x0F) | ADS124S08_CMD_RREG);
	spiTransferByte(0x00);
	ret = spiTransferByte(ADS124S08_CMD_NOP);
	digitalWrite(SS, HIGH);  // Pull CS high
	return ret;
}

int32_t MuDAQClass::readData(void)
{
	uint32_t ret;
	uint8_t byte;
	setSPIMode(ADS124S08_SPI_MODE);
	digitalWrite(SS, LOW);  // Pull CS low
	byte = spiTransferByte(ADS124S08_CMD_NOP);
	ret = (((uint32_t)byte) << 16) | ((byte & 0x80) ? 0xFF000000:0x00000000);
	ret |= (((uint32_t)spiTransferByte(ADS124S08_CMD_NOP)) << 8);
	ret |= (uint32_t)spiTransferByte(ADS124S08_CMD_NOP);
	digitalWrite(SS, HIGH);  // Pull CS high
	return ret;
}

void MuDAQClass::stopContinuous(void)
{
	setSPIMode(ADS124S08_SPI_MODE);
	digitalWrite(SS, LOW);  // Pull CS low
	spiTransferByte(ADS124S08_CMD_STOP);
	digitalWrite(SS, HIGH);  // Pull CS high
}

void MuDAQClass::writeReg(uint8_t addr, uint8_t data)
{
	digitalWrite(SS, HIGH);  // Just to be sure
	setSPIMode(ADS124S08_SPI_MODE);
	digitalWrite(SS, LOW);  // Pull CS low
	spiTransferByte((addr & 0x0F) | ADS124S08_CMD_WREG);
	spiTransferByte(0x00);
	spiTransferByte(data);
	digitalWrite(SS, HIGH);  // Pull CS high
}

void MuDAQClass::wake(void)
{
	setSPIMode(ADS124S08_SPI_MODE);
	digitalWrite(SS, LOW);  // Pull CS low
	spiTransferByte(ADS124S08_CMD_WAKE);
	digitalWrite(SS, HIGH);  // Pull CS high
}

void MuDAQClass::sleep(void)
{
	setSPIMode(ADS124S08_SPI_MODE);
	digitalWrite(SS, LOW);  // Pull CS low
	spiTransferByte(ADS124S08_CMD_POWERDOWN);
	digitalWrite(SS, HIGH);  // Pull CS high
}

/*void MuDAQClass::selfOffsetCal(void)
{
	uint8_t MUX1_old = readReg(ADS124S08_MUX1);
	digitalWrite(ADS124S08_START, HIGH);
	enableIntRef();
	selectRef(ADC_INTREF);
	digitalWrite(ADS124S08_START, LOW);
	delay(1);			// Wait for ref to settle
	setSPIMode(ADS124S08_SPI_MODE);
	digitalWrite(SS, LOW);  // Pull CS low
	spiTransferByte(ADS124S08_CMD_SLEEP);
	digitalWrite(SS, HIGH);  // Pull CS high
	// TODO add timeout here
	while(digitalRead(ADS124S08_DRDY) == HIGH){};
	digitalWrite(ADS124S08_START, HIGH);
	writeReg(ADS124S08_MUX1,MUX1_old);
	digitalWrite(ADS124S08_START, LOW);
}*/

void MuDAQClass::dumpRegs(void)
{
	digitalWrite(ADS124S08_START, HIGH);
	for(uint8_t i = 0;i<0x40;i++)
	{
		Serial.print("Reg 0x");
		Serial.print(i,16);
		Serial.print(" : 0x");
		Serial.print(readReg(i),16);
		Serial.print("\n");
	}
	digitalWrite(ADS124S08_START, LOW);
}

// Remember that this function does NOT control the chip select line
uint8_t spiTransferByte(uint8_t data)
{
	SPDR = data;
	// wait for transfer to complete
	while(!(SPSR & (1<<SPIF)));
	return SPDR;
}
// Mode is 0 or 3 for dataflash and 1 for ADS124S08
void setSPIMode(uint8_t mode)
{
	SPCR = (SPCR & ~SPI_MODE_MASK) | mode;
}
