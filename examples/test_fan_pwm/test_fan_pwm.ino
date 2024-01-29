#include "A2D_Eload.h"
#include <Wire.h>

A2D_Eload eload;

void setup()
{
    Wire.begin();
    Serial.begin(115200);
    eload.init();

    Serial.println("Initialization Complete");

    eload.set_fan(false);

    Serial.println("Setup Complete");
}

void loop()
{
    for(float speed = 0.0; speed <= 1.0; speed += 0.1)
    {
        eload.set_fan_speed(speed);
        delay(2500); 
    }
    for(float speed = 1.0; speed >= 0.0; speed -= 0.1)
    {
        eload.set_fan_speed(speed);
        delay(2500); 
    }
}