/*
AUTHOR: Micah Black, A2D Electronics
DATE: Nov 15, 2023
PURPOSE: This example prints out the voltage of all 4 channels every 1s.
*/

#include "A2D_4CH_Isolated_ADC.h"
#include <Wire.h>

A2D_4CH_Isolated_ADC adc = A2D_4CH_Isolated_ADC();

void setup() {
	Wire.begin();
	Serial.begin(115200);
	
	delay(2000); //delay to wait to start up serial monitor
	
	Serial.println("A2D 4CH Isolated ADC Test");
	adc.init();
}

void loop() {
	for(uint8_t ch = 0; ch < A2D_4CH_ISO_ADC_NUM_CHANNELS; ch++)
	{
		Serial.print("Ch ");
		Serial.print(ch);
		Serial.print("  At ADC: ");
		Serial.print(adc.measure_raw_voltage(ch), 5);
		Serial.print("V,  At Input: ");
		Serial.print(adc.measure_voltage(ch), 5);
		Serial.println("V");
	}
	Serial.println("----------------");
	Serial.flush();
	delay(500);
}
