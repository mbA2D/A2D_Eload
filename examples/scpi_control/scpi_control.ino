/*
AUTHOR: Micah Black, A2D Electronics
DATE: Feb 2, 2024
PURPOSE: This example implements some SCPI commands
		(that don't completely follow the susbystem style standard)
		to communicate with the A2D Eload board.
*/

//TODO - an extra UART used for debugging would be nice
//TODO - add in SCPI error queries - if the device is overtemp, bad command, bad parsing, etc.
//TODO - would be nice to move to different IDE without preprocessor bugs
//TODO - a delay 1s or more is needed between a reset command a sending the current for the current to take effect - figure out why.

#include <A2D_Eload.h>

#define SERIAL F("0001") //There seems to be a bug with Arduino IDE #ifdef, #ifndef, etc. need to hard-code the serial number

//#define DEBUG //uncomment this line to add all debug printing statements

//#define FORCE_EEPROM_REINIT //uncomment this line to force reinitialization of EEPROM (serial number, calibration, RS485 Address, etc.)
							//use this when the format of data storage in EEPROM has changed

// SERIAL Settings
#define BAUDRATE 115200
#define SER_BUF_LEN 60
#define CMD_BUF_LEN 20
#define END_CHAR '\n'
#define NO_CMD ""

// RS485 Settings
#define RS485_BAUDRATE 57600

// Macro for finding commands - PSTR to store string in PROGMEM(flash) instead of memory
#define CMDIS(i, c) (!strcmp_P(i, PSTR(c)))

// Function Prototypes for SCPI Parsing
uint8_t calc_rs485_address(uint8_t ch);
uint8_t parse_uint8_t();
uint8_t parse_channel();
bool parse_bool();
float parse_float();
bool channel_on_this_device(uint8_t channel);
void parse_command(char ser_buf[], char command[], bool from_rs485);
void pass_cmd_to_rs485(uint8_t rs485_addr);
void wait_rs485_response_send_usb(char ser_buf[]);

// Function Prototypes for SCPI handlers
void scpi_handler_idn();
void scpi_handler_rst();

void scpi_handler_send_1_bool(void (A2D_Eload::*func_to_call)(bool));
void scpi_handler_send_1_bool_ch(void (A2D_Eload::*func_to_call)(bool));

void scpi_handler_read_1_bool(bool (A2D_Eload::*func_to_call)());
void scpi_handler_read_1_bool_ch(bool (A2D_Eload::*func_to_call)());

void scpi_handler_read_1_float_ch(float (A2D_Eload::*func_to_call)());
void scpi_handler_send_1_float_ch(void (A2D_Eload::*func_to_call)(float));

void scpi_handler_read_2_float_ch(float (A2D_Eload::*func_to_call_1)(), float (A2D_Eload::*func_to_call_2)());
void scpi_handler_no_func_ch();
void scpi_handler_no_arg_no_return_ch(void (A2D_Eload::*func_to_call)());

void scpi_handler_send_4_float_ch(void (A2D_Eload::*func_to_call)(float, float, float, float));

void scpi_handler_instr_rs485_set();
void scpi_handler_instr_rs485();
void scpi_handler_instr_rs485_save();

// Function prototypes for control
void fan_temp_control();
void output_watchdog();
void output_24v_test();


//Global Variables
A2D_Eload g_a2d_eload;
HardwareSerial Serial3(PB11, PB10); // RX, TX - for RS485
uint8_t g_a2d_cmd_source;
bool g_a2d_is_query;
char ser_buf[SER_BUF_LEN];
uint8_t g_a2d_rs485_address;
unsigned long g_a2d_last_command_time;
unsigned long g_a2d_old_last_command_time;
unsigned long g_a2d_last_control_time;

void setup()
{
	// put your setup code here, to run once:
	Wire.begin();				   // I2C
	Serial.begin(BAUDRATE);		   // Communication with PC over USB
	Serial3.begin(RS485_BAUDRATE); // RS485 Communication
	//delay(2000); //delay to let serial monitor connect before printing stuff
	g_a2d_eload.init();

	g_a2d_cmd_source = 0;
	g_a2d_is_query = false;
	g_a2d_last_command_time = 0;
	g_a2d_last_control_time = 0;

#ifdef FORCE_EEPROM_REINIT
	g_a2d_eload.force_eeprom_reinit();
#endif
}

void loop()
{

	// Allocate memory for the serial buffer
	char command[CMD_BUF_LEN];
	uint8_t chars_input = 0;

	// If the command is not for this device, then we need to send it to the correct one (RS485 Address)
	// Then wait for a response (with timeout if the device does not respond).
	// RS485 address needs to be stored in EEPROM (DONE)
	// Address of 0 is reserved for the base device (the one connected with USB)
	// Commands with '0' as the channel will only return the 4 channels of the base device.

	// TODO - add a CRC check to RS485 communication.
	// The same firmware should be uploaded to the base device as any others.
	// If channel is greater than A2D_XXNUM_CHANNELS, then use [CH - 4*RS485_ADDR] as the address.

	////////////////////////////////////// CHECK NEW COMMANDS

	// CHECK COMMANDS FROM USB (priority 1)
	if (Serial.available())
	{
#ifdef DEBUG
		Serial.println(F("USB Available"));
#endif

		// Read until a full command is received
		chars_input = Serial.readBytesUntil(END_CHAR, ser_buf, SER_BUF_LEN);
		ser_buf[chars_input] = '\0'; // terminate the input string with NULL to work with strtok
		parse_command(ser_buf, command, false);

		g_a2d_cmd_source = A2D_ELOAD_CMD_SOURCE_USB;

		//If after processing the command, it is not for this device, then
		//g_a2d_last_command_time gets reset to the old time in pass_cmd_to_rs485()
		//before the time is checked against the watchdog at the end of void loop()
		g_a2d_old_last_command_time = g_a2d_last_command_time;
		g_a2d_last_command_time = millis();

		// TODO - put the RS485 command processing here - reduces functions calls in SCPI handler functions
		//      - if its not on this device, then just pass it on and wait for the response
	}

	// CHECK COMMANDS FROM RS485 (priority 2)
	else if (Serial3.available())
	{
#ifdef DEBUG
		Serial.println(F("RS485 Available"));
#endif

		chars_input = Serial3.readBytesUntil(END_CHAR, ser_buf, SER_BUF_LEN);

#ifdef DEBUG
		Serial.print(F("Chars read: "));
		Serial.println(chars_input);
#endif

		ser_buf[chars_input] = '\0'; // terminate the input string with NULL to work with strtok

#ifdef DEBUG
		Serial.println(ser_buf);
#endif

		// if from RS485, this could be a response a device or a command to a device.
		// Command out from base: rs485_addr is 1 to 32
		// Command response to base: rs485_addr is 0

		// if addr is 0 and this is the base device, then send the rest of the response out over USB.
		parse_command(ser_buf, command, true);

#ifdef DEBUG
		Serial.print(F("RS485 Address read: "));
		Serial.println(g_a2d_rs485_address);
#endif

		// if address is for this device
		if (g_a2d_rs485_address == g_a2d_eload.get_rs485_addr())
		{

// we need to process the command, but we send the response over RS485 instead of USB.
#ifdef DEBUG
			Serial.println(F("g_a2d_cmd_source = RS485"));
#endif

			g_a2d_cmd_source = A2D_ELOAD_CMD_SOURCE_RS485;
			g_a2d_last_command_time = millis(); //this is for this device, over RS485.
		}
		//else
		//{ // if address is not for this device, don't do anything
		//	strcpy(command, "NOCMD");
		//}
	}

	// if no new commands from RS485 or USB
	else if (!(CMDIS(command, NO_CMD)))
	{
		strcpy(command, NO_CMD); //clear old command
	}

	////////////////////////////////////// PROCESS COMMANDS

	//*IDN?
	if (CMDIS(command, "*IDN"))
	{
		scpi_handler_idn();
	}

	//*RST
	else if (CMDIS(command, "*RST"))
	{
		scpi_handler_rst();
	}

	// INSTR:LED CH,VAL        or      INSTR:LED CH?
	// VAL is boolean 0 or 1
	else if (CMDIS(command, "INSTR:LED"))
	{
		if(g_a2d_is_query)
		{
			scpi_handler_read_1_bool_ch(&A2D_Eload::get_led);
		}
		else
		{
			scpi_handler_send_1_bool_ch(&A2D_Eload::set_led);
		}
	}

	// INSTR:FAN CH,VAL      or      INSTR:FAN CH?
	// VAL is boolean 0 or 1
	else if (CMDIS(command, "INSTR:FAN"))
	{
		if(g_a2d_is_query)
		{
			scpi_handler_read_1_bool_ch(&A2D_Eload::get_fan);
		}
		else
		{
			scpi_handler_send_1_bool_ch(&A2D_Eload::set_fan);
		}
	}

	// INSTR:RELAY CH,VAL     or    INSTR:RELAY CH?
	// VAL is boolean 0 or 1
	else if (CMDIS(command, "INSTR:RELAY"))
	{
		if(g_a2d_is_query)
		{
			scpi_handler_read_1_bool_ch(&A2D_Eload::get_relay);
		}
		else
		{
			scpi_handler_send_1_bool_ch(&A2D_Eload::set_relay);
		}
	}

	// MEAS:VOLT CH?
	else if (CMDIS(command, "INSTR:KICK"))
	{
		scpi_handler_no_func_ch();
	}

	// MEAS:VOLT CH?
	else if (CMDIS(command, "MEAS:VOLT"))
	{
		scpi_handler_read_1_float_ch(&A2D_Eload::measure_voltage);
	}

	// MEAS:TEMP CH?
	else if (CMDIS(command, "MEAS:TEMP"))
	{
		scpi_handler_read_1_float_ch(&A2D_Eload::measure_temperature);
	}

	// MEAS:VOLT:ADC CH?
	else if (CMDIS(command, "MEAS:VOLT:ADC"))
	{
		scpi_handler_read_1_float_ch(&A2D_Eload::measure_raw_voltage);
	}

	// CAL:V:RST CH
	else if (CMDIS(command, "CAL:V:RST"))
	{
		scpi_handler_no_arg_no_return_ch(&A2D_Eload::reset_v_calibration);
	}

	// CAL:V:SAV CH
	else if (CMDIS(command, "CAL:V:SAV"))
	{
		scpi_handler_no_arg_no_return_ch(&A2D_Eload::save_v_calibration);
	}

	// CAL:I:RST CH
	else if (CMDIS(command, "CAL:I:RST"))
	{
		scpi_handler_no_arg_no_return_ch(&A2D_Eload::reset_i_calibration);
	}

	// CAL:I:SAV CH
	else if (CMDIS(command, "CAL:I:SAV"))
	{
		scpi_handler_no_arg_no_return_ch(&A2D_Eload::save_i_calibration);
	}

	// CAL:V CH,MEAS1,ACTUAL1,MEAS2,ACTUAL2  or  CAL:V CH?
	else if (CMDIS(command, "CAL:V"))
	{
		if(g_a2d_is_query)
		{
			scpi_handler_read_2_float_ch(&A2D_Eload::get_cal_v_offset, &A2D_Eload::get_cal_v_gain);
		}
		else
		{
			scpi_handler_send_4_float_ch(&A2D_Eload::calibrate_voltage);
		}
	}

	// CAL:I CH,MEAS1,ACTUAL1,MEAS2,ACTUAL2  or  CAL:I CH?
	else if (CMDIS(command, "CAL:I"))
	{
		if(g_a2d_is_query)
		{
			scpi_handler_read_2_float_ch(&A2D_Eload::get_cal_i_offset, &A2D_Eload::get_cal_i_gain);
		}
		else
		{
			scpi_handler_send_4_float_ch(&A2D_Eload::calibrate_current);
		}
	}

	// CURR CH? or CURR CH,VAL
	else if (CMDIS(command, "CURR"))
	{
		if(g_a2d_is_query)
		{
			scpi_handler_read_1_float_ch(&A2D_Eload::get_current_target);
		}
		else
		{
			scpi_handler_send_1_float_ch(&A2D_Eload::set_current_target);
		}
	}

	// CURR:CTRL CH?
	// returns the voltage output of the DAC that controls the current
	else if (CMDIS(command, "CURR:CTRL"))
	{
		scpi_handler_read_1_float_ch(&A2D_Eload::get_dac_voltage);
		//scpi_handler_curr_ctrl();
	}

	// INSTR:RS485?    or     INSTR:RS485 ADDR
	// Returns the current RS485 address
	else if (CMDIS(command, "INSTR:RS485"))
	{
		if(g_a2d_is_query)
		{
			scpi_handler_instr_rs485();
		}
		else
		{
			scpi_handler_instr_rs485_set();
		}
	}

	// INSTR:RS485:SAV
	// Save current RS485 address to EEPROM
	else if (CMDIS(command, "INSTR:RS485:SAV"))
	{
		scpi_handler_instr_rs485_save();
	}


	//////////////////////////////// CHECK CONTROL
	if(((millis() - g_a2d_last_control_time)/1000.0) > A2D_ELOAD_CONTROL_S)
	{
		g_a2d_last_control_time = millis();

		//Temperature control of fan, and temperature limit
		fan_temp_control();

		//Output Watchdog - turn off output if no commands for this device received from computer in X seconds
		output_watchdog();

		//Output control for 24V supply - turn off output command if no 24V supply present
		output_24v_test();
	}
}

///////////////////////////////////// CONTROL
void fan_temp_control()
{
	float eload_temp = g_a2d_eload.measure_temperature();
	if(eload_temp > (A2D_ELOAD_FAN_TEMP_C + A2D_ELOAD_FAN_TEMP_HYST))
	{
		g_a2d_eload.set_fan(true);
	}
	else if(eload_temp < (A2D_ELOAD_FAN_TEMP_C - A2D_ELOAD_FAN_TEMP_HYST))
	{
		g_a2d_eload.set_fan(false);
	}

	if(eload_temp > A2D_ELOAD_MAX_TEMP_C)
	{
		//Serial.println("Max Temp Tripped");
		g_a2d_eload.set_relay(false);
	}
}

void output_watchdog()
{
	if(g_a2d_eload.get_relay() && ((millis() - g_a2d_last_command_time)/1000.0 > A2D_ELOAD_WATCHDOG_TIMEOUT_S))
	{
		//Serial.println("Output Watchdog Tripped");
		g_a2d_eload.set_relay(false);
	}
}

void output_24v_test()
{
	g_a2d_eload.check_24v_supply(); //must be called often to initialize DAC when available
	//if output is on, it will be turned off by the check_24v_supply() function if there is no 24V supply
}

//////////////////////////////////// GENERIC SCPI HANDLERS

void scpi_handler_send_1_bool(void (A2D_Eload::*func_to_call)(bool))
{
	(g_a2d_eload.*func_to_call)(parse_bool());
}

void scpi_handler_send_1_bool_ch(void (A2D_Eload::*func_to_call)(bool))
{
	uint8_t ch = parse_channel();
	if (ch >= 1 && ch <= A2D_ELOAD_NUM_CHANNELS)
	{
		(g_a2d_eload.*func_to_call)(parse_bool());
	}
	else if (g_a2d_eload.get_rs485_addr() == A2D_ELOAD_BASE_RS485_ADDR && g_a2d_cmd_source == A2D_ELOAD_CMD_SOURCE_USB)
	{
		// if this is a base device, send command up over RS485.
		pass_cmd_to_rs485(calc_rs485_address(ch));
	}
}

void scpi_handler_read_1_bool(bool (A2D_Eload::*func_to_call)())
{
	Serial.println((g_a2d_eload.*func_to_call)(), DEC); //print out in ASCII decimal
	Serial.flush();
}

void scpi_handler_read_1_bool_ch(bool (A2D_Eload::*func_to_call)())
{
	uint8_t ch = parse_channel();

	// channel is on this device
	if (ch >= 1 && ch <= A2D_ELOAD_NUM_CHANNELS)
	{
		if (g_a2d_cmd_source == A2D_ELOAD_CMD_SOURCE_USB)
		{
			Serial.println((g_a2d_eload.*func_to_call)(), DEC); //print out in ASCII decimal
			Serial.flush();
		}
		else if (g_a2d_cmd_source == A2D_ELOAD_CMD_SOURCE_RS485)
		{
			g_a2d_eload.set_rs485_receive(false);
			Serial3.println((g_a2d_eload.*func_to_call)(), DEC); //print out in ASCII decimal
			Serial3.flush();
			g_a2d_eload.set_rs485_receive(true);
		}
	}

	// channel is not 0, and channel is not on this device, and this is a base device.
	else if (g_a2d_eload.get_rs485_addr() == A2D_ELOAD_BASE_RS485_ADDR && g_a2d_cmd_source == A2D_ELOAD_CMD_SOURCE_USB)
	{
		// if this is a base device, send command up over RS485.
		pass_cmd_to_rs485(calc_rs485_address(ch));
		wait_rs485_response_send_usb(ser_buf);
	}
}

void scpi_handler_read_1_float_ch(float (A2D_Eload::*func_to_call)())
{
	uint8_t ch = parse_channel();

	// channel is on this device
	if (ch >= 1 && ch <= A2D_ELOAD_NUM_CHANNELS)
	{
		if (g_a2d_cmd_source == A2D_ELOAD_CMD_SOURCE_USB)
		{
			Serial.println((g_a2d_eload.*func_to_call)(), 4);
			Serial.flush();
		}
		else if (g_a2d_cmd_source == A2D_ELOAD_CMD_SOURCE_RS485)
		{
			g_a2d_eload.set_rs485_receive(false);
			Serial3.println((g_a2d_eload.*func_to_call)(), 4);
			Serial3.flush();
			g_a2d_eload.set_rs485_receive(true);
		}
	}

	// channel is not 0, and channel is not on this device, and this is a base device.
	else if (g_a2d_eload.get_rs485_addr() == A2D_ELOAD_BASE_RS485_ADDR && g_a2d_cmd_source == A2D_ELOAD_CMD_SOURCE_USB)
	{
		// if this is a base device, send command up over RS485.
		pass_cmd_to_rs485(calc_rs485_address(ch));
		wait_rs485_response_send_usb(ser_buf);
	}
}

void scpi_handler_no_func_ch()
{
	uint8_t ch = parse_channel();
	if (ch >= 1 && ch <= A2D_ELOAD_NUM_CHANNELS)
	{
		;//do nothing - this command exists to kick the watchdog.
	}
	else if (g_a2d_eload.get_rs485_addr() == A2D_ELOAD_BASE_RS485_ADDR && g_a2d_cmd_source == A2D_ELOAD_CMD_SOURCE_USB)
	{
		// if this is a base device, send command up over RS485.
		pass_cmd_to_rs485(calc_rs485_address(ch));
	}
}

void scpi_handler_no_arg_no_return_ch(void (A2D_Eload::*func_to_call)())
{
	uint8_t ch = parse_channel();
	if (ch >= 1 && ch <= A2D_ELOAD_NUM_CHANNELS)
	{
		(g_a2d_eload.*func_to_call)();
	}
	else if (g_a2d_eload.get_rs485_addr() == A2D_ELOAD_BASE_RS485_ADDR && g_a2d_cmd_source == A2D_ELOAD_CMD_SOURCE_USB)
	{
		// if this is a base device, send command up over RS485.
		pass_cmd_to_rs485(calc_rs485_address(ch));
	}
}

void scpi_handler_read_2_float_ch(float (A2D_Eload::*func_to_call_1)(), float (A2D_Eload::*func_to_call_2)())
{
	uint8_t ch = parse_channel();
	if (ch >= 1 && ch <= A2D_ELOAD_NUM_CHANNELS)
	{
		if (g_a2d_cmd_source == A2D_ELOAD_CMD_SOURCE_USB)
		{
			Serial.print((g_a2d_eload.*func_to_call_1)(), 5);
			Serial.print(",");
			Serial.println((g_a2d_eload.*func_to_call_2)(), 5);
			Serial.flush();
		}
		else if (g_a2d_cmd_source == A2D_ELOAD_CMD_SOURCE_RS485)
		{
			g_a2d_eload.set_rs485_receive(false);
			Serial3.print((g_a2d_eload.*func_to_call_1)(), 5);
			Serial3.print(",");
			Serial3.println((g_a2d_eload.*func_to_call_2)(), 5);
			Serial3.flush();
			g_a2d_eload.set_rs485_receive(true);
		}
	}
	else if (g_a2d_eload.get_rs485_addr() == A2D_ELOAD_BASE_RS485_ADDR && g_a2d_cmd_source == A2D_ELOAD_CMD_SOURCE_USB)
	{
		// if this is a base device, send command up over RS485.
		pass_cmd_to_rs485(calc_rs485_address(ch));
		wait_rs485_response_send_usb(ser_buf);
	}
}

void scpi_handler_send_4_float_ch(void (A2D_Eload::*func_to_call)(float, float, float, float))
{
	uint8_t ch = parse_channel();
	if (ch >= 1 && ch <= A2D_ELOAD_NUM_CHANNELS)
	{
		// create memory to store all the floats
		const uint8_t num_floats = 4;
		float float_arr[num_floats];
		for (uint8_t index = 0; index < num_floats; index++)
		{
			float_arr[index] = parse_float();
		}
		(g_a2d_eload.*func_to_call)(float_arr[0], float_arr[1], float_arr[2], float_arr[3]);
	}
	else if (g_a2d_eload.get_rs485_addr() == A2D_ELOAD_BASE_RS485_ADDR && g_a2d_cmd_source == A2D_ELOAD_CMD_SOURCE_USB)
	{
		// if this is a base device, send command up over RS485.
		pass_cmd_to_rs485(calc_rs485_address(ch));
	}
}

void scpi_handler_send_1_float_ch(void (A2D_Eload::*func_to_call)(float))
{
	uint8_t ch = parse_channel();
	if (ch >= 1 && ch <= A2D_ELOAD_NUM_CHANNELS)
	{
		float curr_target = parse_float();
		(g_a2d_eload.*func_to_call)(curr_target);
	}
	else if (g_a2d_eload.get_rs485_addr() == A2D_ELOAD_BASE_RS485_ADDR && g_a2d_cmd_source == A2D_ELOAD_CMD_SOURCE_USB)
	{
		// if this is a base device, send command up over RS485.
		pass_cmd_to_rs485(calc_rs485_address(ch));
	}
}

//////////////////////////////////// SPECIFIC SCPI HANDLERS

void scpi_handler_idn()
{
	Serial.print(MANUFACTURER);
	Serial.print(F(","));
	Serial.print(MODEL);
	Serial.print(F(","));
	//Serial.print(g_a2d_eload.get_serial_num());
	Serial.print(SERIAL);
	Serial.print(F(","));
	Serial.println(VERSION);
	Serial.flush();
}

void scpi_handler_rst()
{
	g_a2d_eload.reset();
}

void scpi_handler_instr_rs485_set()
{
	uint8_t addr = parse_uint8_t();
	g_a2d_eload.set_rs485_addr(addr);
}

void scpi_handler_instr_rs485()
{
	Serial.println(g_a2d_eload.get_rs485_addr());
	Serial.flush();
}

void scpi_handler_instr_rs485_save()
{
	g_a2d_eload.save_rs485_addr();
}

//////////////////////////////////// SCPI PARSING FUNCTIONS

uint8_t calc_rs485_address(uint8_t ch)
{
	return uint8_t((ch - 1) / A2D_ELOAD_NUM_CHANNELS);
}

uint8_t parse_uint8_t()
{
	char delimeters[] = " ,?";
	char *token = strtok(NULL, delimeters);
	uint8_t val = uint8_t(atoi(token));
	return val;
}

uint8_t parse_channel()
{
	// returns channel if the channel is not for this device.

	// if command is not for this device, and this is the base device, then pass it on over RS485 (and wait for response if applicable)
	// if command is not for this device, and this is not the base device, then ignore the command.
	// if command is for this device, then process the command.

	uint8_t channel = parse_uint8_t();

	if (channel == 0)
	{
		return 0;
	}
	else if (channel_on_this_device(channel))
	{
		channel = channel % A2D_ELOAD_NUM_CHANNELS;
		if (channel == 0)
		{
			channel = A2D_ELOAD_NUM_CHANNELS;
		}
		return channel;
	}
	else
	{
		return channel;
	}
}

bool parse_bool()
{
	char delimeters[] = " ,?";
	char *token = strtok(NULL, delimeters);
	uint8_t val = uint8_t(atoi(token));
	if (val > 0)
	{
		return true;
	}
	else if (val == 0)
	{
		return false;
	}
}

float parse_float()
{
	char delimeters[] = " ,?";
	char *token = strtok(NULL, delimeters);
	float val = String(token).toFloat();
	return val;
}

bool channel_on_this_device(uint8_t channel)
{
	if ((channel >= (g_a2d_eload.get_rs485_addr()) * A2D_ELOAD_NUM_CHANNELS + 1) &&
		(channel <= (g_a2d_eload.get_rs485_addr()) * A2D_ELOAD_NUM_CHANNELS + A2D_ELOAD_NUM_CHANNELS))
	{
		return true;
	}
	return false;
}

void parse_command(char ser_buf[], char command[], bool from_rs485)
{
	// parses commands from the USB port (base device)

	// we will assume only 1 command is sent at a time
	// so we don't have to deal with SCPI's ';' to send
	// multiple commands on the same line

	// split input string on space to extract the command and the parameters
	// strtok replaces the delimeter with NULL to terminate the string
	// strtok maintains a static pointer to the original string passed to it.
	// to get the next token, pass NULL as the first argument.

	char delimeters_address_command[] = " ?";
	char *token;

	if (strchr(ser_buf, '?') != NULL)
	{
		g_a2d_is_query = true;
		//Serial.println("is_query=true");
	}
	else
	{
		g_a2d_is_query = false;
		//Serial.println("is_query=false");
	}

	if (from_rs485)
	{
		// parses commands from the RS485 port (chained devices)

		// returns the rs485 address sent (first part of the serial buffer).
		// if the address is 0, then command will hold what should be printed to USB.

		token = strtok(ser_buf, delimeters_address_command);
		g_a2d_rs485_address = uint8_t(atoi(token));
		token = strtok(NULL, delimeters_address_command);
		strcpy(command, token);
	}
	else
	{ // from USB
		token = strtok(ser_buf, delimeters_address_command);
		strcpy(command, token); // copies the token into command.
	}
}

void pass_cmd_to_rs485(uint8_t rs485_addr)
{
	g_a2d_last_command_time = g_a2d_old_last_command_time;
	g_a2d_eload.set_rs485_receive(false);
	Serial3.print(rs485_addr); // rs485 address
	Serial3.print(" ");
	Serial3.print(ser_buf);
	Serial3.flush();
	g_a2d_eload.set_rs485_receive(true);

#ifdef DEBUG
	Serial.print(F("Printed over RS485: "));
	Serial.print(rs485_addr); // destination rs485 address
	Serial.print(" ");
	Serial3.print(ser_buf);
	Serial.flush();
#endif
}

void wait_rs485_response_send_usb(char ser_buf[])
{
	// wait for a response.

#ifdef DEBUG
	Serial.println(F("Waiting for RS485"));
#endif

	while (!Serial3.available()) //TODO - add timeout
	{
		delayMicroseconds(5);
	}

#ifdef DEBUG
	Serial.println(F("RS485 Available"));
#endif

	uint8_t chars_response = Serial3.readBytesUntil(END_CHAR, ser_buf, SER_BUF_LEN);

#ifdef DEBUG
	Serial.println(F("RS485 Done Reading"));
#endif

	// send it back over USB.
	Serial.write(ser_buf, chars_response);
	Serial.println("");
	Serial.flush();
}
