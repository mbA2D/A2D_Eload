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
    eload.set_relay(false);
    eload.set_led(false);

    Serial.println("Setup Complete");
}

void loop()
{
    delay(5000);

    Serial.println("ON");
    eload.set_fan(true);
    eload.set_relay(true);
    eload.set_led(true);

    delay(5000);

    Serial.println("OFF");
    eload.set_fan(false);
    eload.set_relay(false);
    eload.set_led(false);

}