#include <EEPROM.h>


#define SERIAL_EEPROM_TEST "0001"

#define SERIAL_LEN 5
char _serial[SERIAL_LEN];
int _ee_addr_serial;

void setup() {
  _ee_addr_serial = 1;

  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(2000);
  Serial.println("EEPROM Test");
  
  Serial.print("SERIAL: ");
  Serial.println(SERIAL_EEPROM_TEST);

  strcpy(_serial, SERIAL_EEPROM_TEST);

  Serial.print("_serial: ");
  Serial.println(_serial);

  //put in EEPROM
  for(int i = 0; i < (SERIAL_LEN-1); i++)
	{
		EEPROM.put(_ee_addr_serial + i*sizeof(*_serial), _serial[i]);
		Serial.print("Putting ");
		Serial.print(_serial[i]);
		Serial.print(" at ");
		Serial.println(_ee_addr_serial + i*sizeof(*_serial));
	}

  //get from EEPROM
  for(int i = 0; i < (SERIAL_LEN-1); i++)
	{
		EEPROM.get(_ee_addr_serial + i*sizeof(*_serial), _serial[i]); //serial number
		Serial.print("Read ");
		Serial.print(_serial[i]);
		Serial.print(" from ");
		Serial.println(_ee_addr_serial + i*sizeof(*_serial));
	}
  _serial[SERIAL_LEN-1] = '\0';

  //print out again
  Serial.print("_serial: ");
  Serial.println(_serial);

}

void loop() {
  // put your main code here, to run repeatedly:

}
