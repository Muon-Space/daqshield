/*
 * \brief Library for operating the DAQShield.
 *
 * This library has all necessary functions for using the ADS1248
 * and DATAFLASH part on the Arduino DAQShield.
 * Copyright (c) 2012 by Jonny Dyer <jonny.dyer@gmail.com>
 * 
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef _MUDAQ_H_INCLUDED
#define _MUDAQ_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "pins_arduino.h"

// LEDS
#define STAT_LED_1 13
#define LED_ON 0				// Open drain configuration
#define LED_OFF 1


class MuDAQClass {
public:
	bool begin(void);
	void end(void);
	
	void reset(void);
	
	void wake(void);
	void sleep(void);
	void stopContinuous(void);
	
	void startSingle(void);
	float sample(uint8_t ch_pos, uint8_t ch_neg, uint8_t gain, uint8_t sampleRate);
	
	void setPGA(uint8_t gain);
	void setSampleRate(uint8_t rate);
	void enableIntRef(void);
	//void selectRef(uint8_t ref_mux);
	//void selectMuxCal(uint8_t muxcal);
	//void selfOffsetCal(void);
	void dumpRegs(void);
	
	uint8_t readReg(uint8_t addr);
	int32_t readData(void);
	void writeReg(uint8_t addr, uint8_t data);
private:
	void initSPI(void);
};

extern MuDAQClass MuDAQ;

#endif
