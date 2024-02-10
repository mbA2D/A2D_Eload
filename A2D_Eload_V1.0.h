//Board Description File for A2D_Sense_Board_V1.0 Board

#include "Arduino.h"

//I2C Address
#define A2D_ELOAD_DAC_I2C_ADDR          0b1001000

#define A2D_ELOAD_FAN_PIN               PB0
#define A2D_ELOAD_FAN_ON                1
#define A2D_ELOAD_FAN_OFF               0

#define A2D_ELOAD_RELAY_PIN             PB5
#define A2D_ELOAD_RELAY_ON              1
#define A2D_ELOAD_RELAY_OFF             0

#define A2D_ELOAD_VSENSE_PIN            PA0
#define A2D_ELOAD_ADC_FULL_SCALE        4096
#define A2D_ELOAD_ADC_VREF              3.3

#define A2D_ELOAD_NTC_PIN               PA1
#define A2D_ELOAD_NTC_TOP_RES           4700.0
#define A2D_ELOAD_NTC_SERIES_RES        1000.0

#define A2D_ELOAD_LED_PIN         	    PC13
#define A2D_ELOAD_LED_ON          	    0
#define A2D_ELOAD_LED_OFF         	    1

#define A2D_ELOAD_DEFAULT_RS485_ADDR	0
#define A2D_ELOAD_BASE_RS485_ADDR		0
#define A2D_ELOAD_NUM_CHANNELS          1

#define A2D_ELOAD_RS485_DE_PIN		    PB1
#define A2D_ELOAD_RS485_TRANSMIT        1
#define A2D_ELOAD_RS485_RECEIVE         0

#define A2D_ELOAD_CMD_SOURCE_USB		1
#define A2D_ELOAD_CMD_SOURCE_RS485	    2

//scaling
#define A2D_ELOAD_DEFAULT_V_SCALING     11.0 // (1k + 10k)/1k
#define A2D_ELOAD_DEFAULT_V_OFFSET	    0.0
#define A2D_ELOAD_24V_MIN_V             20.0
#define A2D_ELOAD_24V_MAX_V             26.0

#define A2D_ELOAD_DEFAULT_I_SCALING     4.0 //2.5V ref gives 10A output 1/(5mOhm * 50V/V Amp)
#define A2D_ELOAD_DEFAULT_I_OFFSET      0.0
#define A2D_ELOAD_MAX_CURRENT           5.0 //Amps

//default Steinhart-Hart Constants for NXRT15XV103FA1B
#define A2D_ELOAD_DEFAULT_TEMP_SHA      1.119349044e-3
#define A2D_ELOAD_DEFAULT_TEMP_SHB      2.359019498e-4
#define A2D_ELOAD_DEFAULT_TEMP_SHC      0.7926382169e-7

//Control constants
#define A2D_ELOAD_FAN_TEMP_C            40.0
#define A2D_ELOAD_FAN_TEMP_HYST         2.0
#define A2D_ELOAD_MAX_TEMP_C            80.0
#define A2D_ELOAD_WATCHDOG_TIMEOUT_S    10.0
#define A2D_ELOAD_CONTROL_S             1.0

#define A2D_ELOAD_EEPROM_INIT_VAL		0x55 //0b01010101
#define A2D_ELOAD_SERIAL_CHAR_LEN		5 //4 characters in the serial number, +1 for the null string
#define A2D_ELOAD_DEFAULT_SERIAL_NUM	"0000"

#define MANUFACTURER F("A2D Electronics")
#define MODEL F("Eload")
#define VERSION F("V1.0.0")
