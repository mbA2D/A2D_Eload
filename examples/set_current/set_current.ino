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
	
	float current_target = 1.0;

	eload.set_current_target(current_target);
	Serial.print("Set Value: ");
	Serial.print(current_target);

	Serial.print(" Actual Value: ");
	Serial.println(eload.get_current_target());
}

void loop() {
	delay(1000);
}
