/*
AUTHOR: Micah Black, A2D Electronics
DATE: Jan 28, 2024
PURPOSE: This example sets the current target on the eload
		 and prints it out to make sure it was set properly.
*/

#include "A2D_Eload.h"
#include <Wire.h>

A2D_Eload eload;

void setup() {
	Wire.begin();
	Serial.begin(115200);
	
	delay(2000); //delay to wait to start up serial monitor
	
	Serial.println("A2D Eload Current Setting Test");
	
	eload.init();
	Serial.println("Initialization Complete");
	
	//Wait for 24V supply to be applied
	while(!eload.check_24v_supply())
	{
		delay(500);
	}

	float current_target = 0.5;

	eload.set_current_target(current_target);
	Serial.print("Set Value: ");
	Serial.println(current_target);

	eload.set_relay(true);
	eload.set_fan(true);
	eload.set_led(true);

	Serial.print("Actual Value: ");
	Serial.println(eload.get_current_target());

	delay(10000);

	eload.set_current_target(0.0);
	eload.set_relay(false);
	eload.set_fan(false);
	eload.set_led(false);

	Serial.println("Output off");
}

void loop() {
	delay(1000);
}
