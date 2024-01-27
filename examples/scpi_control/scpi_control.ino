/*
AUTHOR: Micah Black, A2D Electronics
DATE: Nov 16, 2023
PURPOSE: This example implements some SCPI commands
		(that don't completely follow the susbystem style standard)
		to communicate with the 4ch ADC board.
CHANGELOG:
	Jan 14, 2024: V1.0.1: Added RS485 communication with MEAS:VOLT command
	Jan 21, 2024: Updated SCPI parsing and modified all commands with <CH> parameter to work with RS485 connection
*/

#include <A2D_4CH_Isolated_ADC.h>

// #define DEBUG //uncomment this line to add all debug printing statements
// #define FORCE_EEPROM_REINIT //uncomment this line to force reinitialization of EEPROM (serial number, calibration, RS485 Address, etc.)

#define MANUFACTURER ("A2D Electronics")
#define MODEL ("4CH Isolated ADC")
#define VERSION ("V1.0.1")

// SERIAL DEFINES
#define BAUDRATE 115200
#define SER_BUF_LEN 256
#define CMD_BUF_LEN 32
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
void parse_command(char ser_buf[], char command[], char *token, bool *is_query, bool from_rs485, uint8_t *rs485_addr);
void pass_cmd_to_rs485(uint8_t rs485_addr);
void wait_rs485_response_send_usb(char ser_buf[]);

// Function Prototypes for SCPI handlers
void scpi_handler_idn();
void scpi_handler_rst();
void scpi_handler_trg();
void scpi_handler_instr_led();
void scpi_handler_meas_volt();
void scpi_handler_meas_volt_adc();
void scpi_handler_cal_reset();
void scpi_handler_cal_save();
void scpi_handler_cal_volt();
void scpi_handler_cal();
void scpi_handler_instr_gain();
void scpi_handler_instr_dr();
void scpi_handler_instr_mode();
void scpi_handler_instr_rs485_set();
void scpi_handler_instr_rs485();
void scpi_handler_instr_rs485_save();

//Global Variables
A2D_4CH_Isolated_ADC g_a2d_adc;
HardwareSerial Serial3(PB11, PB10); // RX, TX - for RS485
uint8_t g_a2d_cmd_source;
bool g_a2d_is_query;
char ser_buf[SER_BUF_LEN];
uint8_t rs485_address;

void setup()
{
	// put your setup code here, to run once:
	Wire.begin();				   // I2C
	Serial.begin(BAUDRATE);		   // Communication with PC over USB
	Serial3.begin(RS485_BAUDRATE); // RS485 Communication
	g_a2d_adc.init();

	g_a2d_cmd_source = 0;
	g_a2d_is_query = false;

#ifdef FORCE_EEPROM_REINIT
	g_a2d_adc.force_eeprom_reinit();
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
	// If channel is greater than 4, then use [CH - 4*RS485_ADDR] as the address.

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
		parse_command(ser_buf, command, &g_a2d_is_query, false, &rs485_address);

		g_a2d_cmd_source = A2D_4CH_ISO_ADC_CMD_SOURCE_USB;

		// TODO - put the RS485 command here - reduces functions calls in SCPI handler functions
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
		parse_command(ser_buf, command, &g_a2d_is_query, true, &rs485_address);

#ifdef DEBUG
		Serial.print(F("RS485 Address read: "));
		Serial.println(rs485_address);
#endif

		// if address is for this device
		if (rs485_address == g_a2d_adc.get_rs485_addr())
		{

// we need to process the command, but we send the response over RS485 instead of USB.
#ifdef DEBUG
			Serial.println(F("g_a2d_cmd_source = RS485"));
#endif

			g_a2d_cmd_source = A2D_4CH_ISO_ADC_CMD_SOURCE_RS485;
		}
		else
		{ // if address is not for this device
			strcpy(command, "NOCMD");
		}
	}

	// if no new commands from RS485 or USB
	else
	{
		strcpy(command, "NOCMD");
	}

	////////////////////////////////////// PROCESS COMMANDS

	// NOCMD
	if (CMDIS(command, "NOCMD"))
	{
		;
	}

	//*IDN?
	else if (CMDIS(command, "*IDN?"))
	{
		scpi_handler_idn();
	}

	//*RST
	else if (CMDIS(command, "*RST"))
	{
		scpi_handler_rst();
	}

	//*CLS
	else if (CMDIS(command, "*CLS"))
	{
		; // nothing since we don't have errors yet
	}

	//*TRG
	else if (CMDIS(command, "*TRG"))
	{
		scpi_handler_trg();
	}

	// INSTR:LED VAL
	// VAL is boolean 0 or 1
	else if (CMDIS(command, "INSTR:LED"))
	{
		scpi_handler_instr_led();
	}

	// MEAS:VOLT CH?
	// CH is integer in range of 1-4, or 0. 0 means read all 4 channels
	else if (CMDIS(command, "MEAS:VOLT"))
	{
		scpi_handler_meas_volt();
	}

	// MEAS:VOLT:ADC CH?
	// CH is integer in range of 1-4, or 0. 0 means read all 4 channels
	else if (CMDIS(command, "MEAS:VOLT:ADC"))
	{
		scpi_handler_meas_volt_adc();
	}

	// CAL:RESET CH
	// CH is integer in range of 1-4, or 0. 0 means reset all 4 channels
	else if (CMDIS(command, "CAL:RESET"))
	{
		scpi_handler_cal_reset();
	}

	// CAL:SAVE CH
	// CH is integer in range of 1-4, or 0. 0 means save all 4 channels
	else if (CMDIS(command, "CAL:SAVE"))
	{
		scpi_handler_cal_save();
	}

	// CAL:VOLT MEAS1,ACTUAL1,MEAS2,ACTUAL2,CH
	else if (CMDIS(command, "CAL:VOLT"))
	{
		scpi_handler_cal_volt();
	}

	// CAL CH?
	// CH is integer in range of 1-4, or 0. 0 means return all 4 channels
	else if (CMDIS(command, "CAL"))
	{
		scpi_handler_cal();
	}

	// INSTR:GAIN GAIN,CH
	// GAIN 1,2,4,8
	// CH is integer in range of 1-4, or 0. 0 means all 4 channels
	else if (CMDIS(command, "INSTR:GAIN"))
	{
		scpi_handler_instr_gain();
	}

	// INSTR:DR DR,CH
	// DR 15,60,240
	// CH is integer in range of 1-4, or 0. 0 means all 4 channels
	else if (CMDIS(command, "INSTR:DR"))
	{
		scpi_handler_instr_dr();
	}

	// INSTR:MODE MODE,CH
	// MODE 0 = Single Shot, 1 = Continuous
	// CH is integer in range of 1-4, or 0. 0 means all 4 channels
	else if (CMDIS(command, "INSTR:MODE"))
	{
		scpi_handler_instr_mode();
	}

	// INSTR:RS485:SET ADDR
	// Set RS485 address
	else if (CMDIS(command, "INSTR:RS485:SET"))
	{
		scpi_handler_instr_rs485_set();
	}

	// INSTR:RS485?
	// Returns the current RS485 address
	else if (CMDIS(command, "INSTR:RS485?"))
	{
		scpi_handler_instr_rs485();
	}

	// INSTR:RS485:SAVE
	// Save current RS485 address to EEPROM
	else if (CMDIS(command, "INSTR:RS485:SAVE"))
	{
		scpi_handler_instr_rs485_save();
	}
}

//////////////////////////////////// SCPI COMMAND HANDLING FUNCTIONS

void scpi_handler_idn()
{
	Serial.print(MANUFACTURER);
	Serial.print(",");
	Serial.print(MODEL);
	Serial.print(",");
	Serial.print(g_a2d_adc.get_serial_num());
	Serial.print(",");
	Serial.println(VERSION);
	Serial.flush();
}

void scpi_handler_rst()
{
	g_a2d_adc.reset();
}

void scpi_handler_trg()
{
	g_a2d_adc.trigger_all_single_shot();
}

void scpi_handler_instr_led()
{
	g_a2d_adc.set_led(parse_bool());
}

void scpi_handler_meas_volt()
{
	uint8_t ch = parse_channel();

	// channel is 0, and command from USB
	if (ch == 0 && g_a2d_cmd_source == A2D_4CH_ISO_ADC_CMD_SOURCE_USB)
	{
		for (uint8_t i = 0; i < A2D_4CH_ISO_ADC_NUM_CHANNELS; i++)
		{
			Serial.print(g_a2d_adc.measure_voltage(i), 4);
			if (i != A2D_4CH_ISO_ADC_NUM_CHANNELS - 1)
			{
				Serial.print(",");
			}
		}
		Serial.println("");
		Serial.flush();
	}

	// channel is on this device
	else if (ch >= 1 && ch <= A2D_4CH_ISO_ADC_NUM_CHANNELS)
	{
		if (g_a2d_cmd_source == A2D_4CH_ISO_ADC_CMD_SOURCE_USB)
		{
			Serial.println(g_a2d_adc.measure_voltage(ch - 1), 4);
			Serial.flush();
		}
		else if (g_a2d_cmd_source == A2D_4CH_ISO_ADC_CMD_SOURCE_RS485)
		{
#ifdef DEBUG
			Serial.println(F("Command from RS485"));
#endif

			g_a2d_adc.set_rs485_receive(false);
			Serial3.println(g_a2d_adc.measure_voltage(ch - 1), 4);
			Serial3.flush();
			g_a2d_adc.set_rs485_receive(true);

#ifdef DEBUG
			Serial.println(F("Response sent RS485"));
			Serial.flush();
#endif
		}
	}

	// channel is not 0, and channel is not on this device, and this is a base device.
	else if (g_a2d_adc.get_rs485_addr() == A2D_4CH_ISO_ADC_BASE_RS485_ADDR && g_a2d_cmd_source == A2D_4CH_ISO_ADC_CMD_SOURCE_USB)
	{
		// if this is a base device, send command up over RS485.
		pass_cmd_to_rs485(calc_rs485_address(ch));
		wait_rs485_response_send_usb(ser_buf);
	}
}

void scpi_handler_meas_volt_adc()
{
	uint8_t ch = parse_channel();
	if (ch == 0 && g_a2d_cmd_source == A2D_4CH_ISO_ADC_CMD_SOURCE_USB)
	{
		for (uint8_t i = 0; i < A2D_4CH_ISO_ADC_NUM_CHANNELS; i++)
		{
			Serial.print(g_a2d_adc.measure_raw_voltage(i), 4);
			if (i != A2D_4CH_ISO_ADC_NUM_CHANNELS - 1)
			{
				Serial.print(",");
			}
		}
		Serial.println("");
		Serial.flush();
	}
	else if (ch >= 1 && ch <= A2D_4CH_ISO_ADC_NUM_CHANNELS)
	{
		if (g_a2d_cmd_source == A2D_4CH_ISO_ADC_CMD_SOURCE_USB)
		{
			Serial.println(g_a2d_adc.measure_raw_voltage(ch - 1), 4);
			Serial.flush();
		}
		else if (g_a2d_cmd_source == A2D_4CH_ISO_ADC_CMD_SOURCE_RS485)
		{
			g_a2d_adc.set_rs485_receive(false);
			Serial3.println(g_a2d_adc.measure_raw_voltage(ch - 1), 4);
			Serial3.flush();
			g_a2d_adc.set_rs485_receive(true);
		}
	}
	else if (g_a2d_adc.get_rs485_addr() == A2D_4CH_ISO_ADC_BASE_RS485_ADDR && g_a2d_cmd_source == A2D_4CH_ISO_ADC_CMD_SOURCE_USB)
	{
		// if this is a base device, send command up over RS485.
		pass_cmd_to_rs485(calc_rs485_address(ch));
		wait_rs485_response_send_usb(ser_buf);
	}
}

void scpi_handler_cal_reset()
{
	uint8_t ch = parse_channel();
	if (ch == 0 && g_a2d_cmd_source == A2D_4CH_ISO_ADC_CMD_SOURCE_USB)
	{
		g_a2d_adc.reset_all_calibration();
	}
	else if (ch >= 1 && ch <= A2D_4CH_ISO_ADC_NUM_CHANNELS)
	{
		g_a2d_adc.reset_calibration(ch - 1);
	}
	else if (g_a2d_adc.get_rs485_addr() == A2D_4CH_ISO_ADC_BASE_RS485_ADDR && g_a2d_cmd_source == A2D_4CH_ISO_ADC_CMD_SOURCE_USB)
	{
		// if this is a base device, send command up over RS485.
		pass_cmd_to_rs485(calc_rs485_address(ch));
	}
}

void scpi_handler_cal_save()
{
	uint8_t ch = parse_channel();
	if (ch == 0 && g_a2d_cmd_source == A2D_4CH_ISO_ADC_CMD_SOURCE_USB)
	{
		g_a2d_adc.save_all_calibration();
	}
	else if (ch >= 1 && ch <= A2D_4CH_ISO_ADC_NUM_CHANNELS)
	{
		g_a2d_adc.save_calibration(ch - 1);
	}
	else if (g_a2d_adc.get_rs485_addr() == A2D_4CH_ISO_ADC_BASE_RS485_ADDR && g_a2d_cmd_source == A2D_4CH_ISO_ADC_CMD_SOURCE_USB)
	{
		// if this is a base device, send command up over RS485.
		pass_cmd_to_rs485(calc_rs485_address(ch));
	}
}

void scpi_handler_cal_volt()
{
	uint8_t ch = parse_channel();
	if (ch >= 1 && ch <= A2D_4CH_ISO_ADC_NUM_CHANNELS)
	{
		// create memory to store all the floats
		const uint8_t num_floats = 4;
		float float_arr[num_floats];
		for (uint8_t index = 0; index < num_floats; index++)
		{
			float_arr[index] = parse_float();
		}
		g_a2d_adc.calibrate_voltage(ch - 1, float_arr[0], float_arr[1], float_arr[2], float_arr[3]);
	}
	else if (g_a2d_adc.get_rs485_addr() == A2D_4CH_ISO_ADC_BASE_RS485_ADDR && g_a2d_cmd_source == A2D_4CH_ISO_ADC_CMD_SOURCE_USB)
	{
		// if this is a base device, send command up over RS485.
		pass_cmd_to_rs485(calc_rs485_address(ch));
	}
}

void scpi_handler_cal()
{
	uint8_t ch = parse_channel();
	if (ch == 0 && g_a2d_cmd_source == A2D_4CH_ISO_ADC_CMD_SOURCE_USB)
	{
		for (uint8_t i = 0; i < A2D_4CH_ISO_ADC_NUM_CHANNELS; i++)
		{
			Serial.print(g_a2d_adc.get_cal_offset(i), 5);
			Serial.print(",");
			Serial.print(g_a2d_adc.get_cal_gain(i), 5);
			if (i != A2D_4CH_ISO_ADC_NUM_CHANNELS - 1)
			{
				Serial.print(",");
			}
		}
		Serial.println("");
		Serial.flush();
	}
	else if (ch >= 1 && ch <= A2D_4CH_ISO_ADC_NUM_CHANNELS)
	{
		if (g_a2d_cmd_source == A2D_4CH_ISO_ADC_CMD_SOURCE_USB)
		{
			Serial.print(g_a2d_adc.get_cal_offset(ch - 1), 5);
			Serial.print(",");
			Serial.println(g_a2d_adc.get_cal_gain(ch - 1), 5);
			Serial.flush();
		}
		else if (g_a2d_cmd_source == A2D_4CH_ISO_ADC_CMD_SOURCE_RS485)
		{
			g_a2d_adc.set_rs485_receive(false);
			Serial3.print(g_a2d_adc.get_cal_offset(ch - 1), 5);
			Serial3.print(",");
			Serial3.println(g_a2d_adc.get_cal_gain(ch - 1), 5);
			Serial3.flush();
			g_a2d_adc.set_rs485_receive(true);
		}
	}
	else if (g_a2d_adc.get_rs485_addr() == A2D_4CH_ISO_ADC_BASE_RS485_ADDR && g_a2d_cmd_source == A2D_4CH_ISO_ADC_CMD_SOURCE_USB)
	{
		// if this is a base device, send command up over RS485.
		pass_cmd_to_rs485(calc_rs485_address(ch));
		wait_rs485_response_send_usb(ser_buf);
	}
}

void scpi_handler_instr_gain()
{
	uint8_t gain_p = parse_uint8_t();
	uint8_t gain = 0;
	uint8_t ch = parse_channel();

	if (ch >= 1 && ch <= A2D_4CH_ISO_ADC_NUM_CHANNELS)
	{
		bool valid_gain = false;
		if (gain_p == 1)
		{
			gain = MCP3425_GAIN_1;
			valid_gain = true;
		}
		else if (gain_p == 2)
		{
			gain = MCP3425_GAIN_2;
			valid_gain = true;
		}
		else if (gain_p == 4)
		{
			gain = MCP3425_GAIN_4;
			valid_gain = true;
		}
		else if (gain_p == 8)
		{
			gain = MCP3425_GAIN_8;
			valid_gain = true;
		}

		if (valid_gain)
		{
			if (ch == 0)
			{
				for (uint8_t i = 0; i < A2D_4CH_ISO_ADC_NUM_CHANNELS; i++)
				{
					g_a2d_adc.set_gain(i, gain);
				}
			}
			else
			{
				g_a2d_adc.set_gain(ch - 1, gain);
			}
		}
	}
	else if (g_a2d_adc.get_rs485_addr() == A2D_4CH_ISO_ADC_BASE_RS485_ADDR && g_a2d_cmd_source == A2D_4CH_ISO_ADC_CMD_SOURCE_USB)
	{
		// if this is a base device, send command up over RS485.
		pass_cmd_to_rs485(calc_rs485_address(ch));
	}
}

void scpi_handler_instr_dr()
{
	uint8_t data_rate_p = parse_uint8_t();
	uint8_t data_rate = 0;
	uint8_t ch = parse_channel();

	if (ch >= 1 && ch <= A2D_4CH_ISO_ADC_NUM_CHANNELS)
	{
		bool valid_data_rate = false;

		if (data_rate_p == 15)
		{
			data_rate = MCP3425_DR_15SPS;
			valid_data_rate = true;
		}
		else if (data_rate_p == 60)
		{
			data_rate = MCP3425_DR_60SPS;
			valid_data_rate = true;
		}
		else if (data_rate_p == 240)
		{
			data_rate = MCP3425_DR_240SPS;
			valid_data_rate = true;
		}

		if (valid_data_rate)
		{
			if (ch == 0)
			{
				for (uint8_t i = 0; i < A2D_4CH_ISO_ADC_NUM_CHANNELS; i++)
				{
					g_a2d_adc.set_data_rate(i, data_rate);
				}
			}
			else
			{
				g_a2d_adc.set_data_rate(ch - 1, data_rate);
			}
		}
	}
	else if (g_a2d_adc.get_rs485_addr() == A2D_4CH_ISO_ADC_BASE_RS485_ADDR && g_a2d_cmd_source == A2D_4CH_ISO_ADC_CMD_SOURCE_USB)
	{
		// if this is a base device, send command up over RS485.
		pass_cmd_to_rs485(calc_rs485_address(ch));
	}
}

void scpi_handler_instr_mode()
{
	uint8_t mode_p = parse_uint8_t();
	uint8_t mode = 0;
	uint8_t ch = parse_channel();

	if (ch >= 1 && ch <= A2D_4CH_ISO_ADC_NUM_CHANNELS)
	{

		bool valid_mode = false;

		if (mode_p == 0)
		{
			mode = MCP3425_MODE_SINGLE_SHOT;
			valid_mode = true;
		}
		else if (mode_p == 1)
		{
			mode = MCP3425_MODE_CONTINUOUS;
			valid_mode = true;
		}

		if (valid_mode)
		{
			if (ch == 0)
			{
				for (uint8_t i = 0; i < A2D_4CH_ISO_ADC_NUM_CHANNELS; i++)
				{
					g_a2d_adc.set_mode(i, mode);
				}
			}
			else
			{
				g_a2d_adc.set_mode(ch - 1, mode);
			}
		}
	}
	else if (g_a2d_adc.get_rs485_addr() == A2D_4CH_ISO_ADC_BASE_RS485_ADDR && g_a2d_cmd_source == A2D_4CH_ISO_ADC_CMD_SOURCE_USB)
	{
		// if this is a base device, send command up over RS485.
		pass_cmd_to_rs485(calc_rs485_address(ch));
	}
}

void scpi_handler_instr_rs485_set()
{
	uint8_t addr = parse_uint8_t();
	g_a2d_adc.set_rs485_addr(addr);
}

void scpi_handler_instr_rs485()
{
	Serial.println(g_a2d_adc.get_rs485_addr());
	Serial.flush();
}

void scpi_handler_instr_rs485_save()
{
	g_a2d_adc.save_rs485_addr();
}

//////////////////////////////////// SCPI PARSING FUNCTIONS

uint8_t calc_rs485_address(uint8_t ch)
{
	return uint8_t((ch - 1) / A2D_4CH_ISO_ADC_NUM_CHANNELS);
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

	char delimeters[] = " ,?";
	char *token = strtok(NULL, delimeters);
	uint8_t channel = uint8_t(atoi(token));

	if (channel == 0)
	{
		return 0;
	}
	else if (channel_on_this_device(channel))
	{
		channel = channel % A2D_4CH_ISO_ADC_NUM_CHANNELS;
		if (channel == 0)
		{
			channel = A2D_4CH_ISO_ADC_NUM_CHANNELS;
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
	if ((channel >= (g_a2d_adc.get_rs485_addr()) * A2D_4CH_ISO_ADC_NUM_CHANNELS + 1) &&
		(channel <= (g_a2d_adc.get_rs485_addr()) * A2D_4CH_ISO_ADC_NUM_CHANNELS + A2D_4CH_ISO_ADC_NUM_CHANNELS))
	{
		return true;
	}
	return false;
}

void parse_command(char ser_buf[], char command[], bool *is_query, bool from_rs485, uint8_t *rs485_addr)
{
	// parses commands from the USB port (base device)

	// we will assume only 1 command is sent at a time
	// so we don't have to deal with SCPI's ';' to send
	// multiple commands on the same line

	// split input string on space to extract the command and the parameters
	// strtok replaces the delimeter with NULL to terminate the string
	// strtok maintains a static pointer to the original string passed to it.
	// to get the next token, pass NULL as the first argument.

	char delimeters_address_command[] = " ";
	char *token;

	if (strchr(ser_buf, '?') != NULL)
	{
		*is_query = true;
	}
	else
	{
		*is_query = false;
	}

	if (from_rs485)
	{
		// parses commands from the RS485 port (chained devices)

		// returns the rs485 address sent (first part of the serial buffer).
		// if the address is 0, then command will hold what should be printed to USB.

		// command format is exactly the same as the USB-based commands, but has the RS485 address at the start.
		// ex: "<ADDR> MEAS:VOLT <CH>?"
		// where <ADDR> is the device at a particular RS485 address
		// and <CH> is the channel in the associated chain.
		//<CH> can be from 0 to 32*A2D_4CH_ISO_ADC_NUM_CHANNELS (0 to 128)

		// the base device could receive a command for a specific channel.
		// channels 0 to 4 are base device only (RS485 address 0).
		// channels 5 to 8 correspond to the device with RS485 address 1.
		//  A device with RS485 address of X has the channels
		//          (X+1) * A2D_4CH_ISO_ADC_NUM_CHANNELS + 1
		//          to
		//          (X+1) * A2D_4CH_ISO_ADC_NUM_CHANNELS + A2D_4CH_ISO_ADC_NUM_CHANNELS
		token = strtok(ser_buf, delimeters_address_command);
		uint8_t rs485_address = uint8_t(atoi(token));
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
	g_a2d_adc.set_rs485_receive(false);
	Serial3.print(rs485_addr); // rs485 address
	Serial3.print(" ");
	Serial3.print(ser_buf);
	Serial3.flush();
	g_a2d_adc.set_rs485_receive(true);

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

	while (!Serial3.available())
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
