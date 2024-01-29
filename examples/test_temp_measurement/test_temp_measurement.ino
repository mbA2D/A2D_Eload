#include "A2D_Eload.h"
#include <Wire.h>

A2D_Eload eload;

void setup()
{
    Wire.begin();
    Serial.begin(115200);
    eload.init();
    analogReadResolution(12);
    Serial.println("Initialization Complete");

    eload.set_fan(false);

    Serial.println("Setup Complete");
}

void loop()
{
    delay(100);
    Serial.print("Temperature: ");
    Serial.println(eload.measure_temperature());
}