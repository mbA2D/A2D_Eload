/* A2D Electronics A2D_Eload_V1.0 Board Library
 * Written By: Micah Black
 * Date: Jan 27, 2024
 * 
 */

#include "A2D_Eload.h"

//constructor
A2D_Eload::A2D_Eload()
{	
	_dac_i2c_addr = A2D_ELOAD_DAC_I2C_ADDR;
	
	//EEPROM Addresses
	//TODO - these could all be constants/defines instead of int
	_ee_addr_initialized = 0;
	_ee_addr_serial = _ee_addr_initialized + sizeof(_ee_initialized);
	_ee_addr_v_scale = _ee_addr_serial + sizeof(*_serial)*A2D_ELOAD_SERIAL_CHAR_LEN;
	_ee_addr_v_off = _ee_addr_v_scale + sizeof(_v_scaling);
	_ee_addr_rs485_addr = _ee_addr_v_off + sizeof(_v_offset);
	_ee_addr_t_sha = _ee_addr_rs485_addr + sizeof(_rs485_addr);
	_ee_addr_t_shb = _ee_addr_t_sha + sizeof(_temp_sha);
	_ee_addr_t_shc = _ee_addr_t_shb + sizeof(_temp_shb);
	_ee_addr_i_scale = _ee_addr_t_shc + sizeof(_temp_shc);
	_ee_addr_i_off = _ee_addr_i_scale + sizeof(_i_scaling);
	
	//DAC
	_dac = new DACX0501();

	//Serial number
	//#ifdef SERIAL
	strcpy(_serial, A2D_ELOAD_DEFAULT_SERIAL_NUM);
	//#endif
}

void A2D_Eload::init(TwoWire *i2c)
{
	pinMode(A2D_ELOAD_LED_PIN, OUTPUT);
	pinMode(A2D_ELOAD_RS485_DE_PIN, OUTPUT);
	pinMode(A2D_ELOAD_FAN_PIN, OUTPUT);
	pinMode(A2D_ELOAD_RELAY_PIN, OUTPUT);

	pinMode(A2D_ELOAD_VSENSE_PIN, INPUT);
	pinMode(A2D_ELOAD_NTC_PIN, INPUT);

	analogReadResolution(12);

	_24v_supply_state = false;
	
	_i2c = i2c;
	_init_dac();
	
	reset();
}

void A2D_Eload::_init_dac()
{
	_dac->init(_dac_i2c_addr, _i2c);
}

void A2D_Eload::_reset_dac()
{
	//setup DAC
	_dac->reset();
	_dac->set_buf_gain(DACX0501_BUFGAIN_1); //buf gain of 1 for 0-2.5 output
	_dac->shut_down_ref(false); //use internal reference
	_dac->set_dac(0.0); //set to 0-scale DAC output
	_dac->shut_down_dac(false); //enable output
}

void A2D_Eload::reset()
{
	//Serial.println("reset");

	set_led(false);
	set_rs485_receive(true);
	set_fan(false);
	set_relay(false);

	_24v_supply_state = false;
	_reset_dac();
	
	_init_from_eeprom();
}

float A2D_Eload::measure_raw_voltage()
{
	return float(analogRead(A2D_ELOAD_VSENSE_PIN))/A2D_ELOAD_ADC_FULL_SCALE*A2D_ELOAD_ADC_VREF;
}

float A2D_Eload::measure_voltage()
{
	float voltage = measure_raw_voltage();
	return _convert_adc_voltage_to_voltage(voltage);
}

bool A2D_Eload::check_24v_supply()
{
	float voltage = measure_voltage();
	if(voltage <= A2D_ELOAD_24V_MAX_V && voltage >= A2D_ELOAD_24V_MIN_V)
	{
		if(!_24v_supply_state)
		{
			//need to reset and initialize the DAC
			_init_dac();
			_reset_dac();
		}
		_24v_supply_state = true;
		return true;
	}
	else
	{
		_24v_supply_state = false;
		return false;
	}
}

float A2D_Eload::measure_temperature()
{
	float voltage = float(analogRead(A2D_ELOAD_NTC_PIN))/A2D_ELOAD_ADC_FULL_SCALE*A2D_ELOAD_ADC_VREF;
	return _convert_adc_voltage_to_temperature(voltage);
}

float A2D_Eload::get_dac_voltage()
{
	return _dac->get_voltage();
}

void A2D_Eload::calibrate_voltage(float p1_meas, float p1_act, float p2_meas, float p2_act)
{
	//calculate new offset (b) and scaling (m) in:  actual = m * measured + b
	_v_scaling = (p2_meas - p1_meas) / (p2_act - p1_act); //rise in actual / run in measured
	_v_offset = p2_act - (1/_v_scaling) * p2_meas; //b = actual - m * measured
}

void A2D_Eload::calibrate_current(float p1_meas, float p1_act, float p2_meas, float p2_act)
{
	//calculate new offset (b) and scaling (m) in:  actual = m * measured + b
	_i_scaling = (p2_meas - p1_meas) / (p2_act - p1_act); //rise in actual / run in measured
	_i_offset = p2_act - (1/_i_scaling) * p2_meas; //b = actual - m * measured
}

void A2D_Eload::set_sh_constants(float sha, float shb, float shc)
{
	_temp_sha = sha;
	_temp_shb = shb;
	_temp_shc = shc;
}

void A2D_Eload::reset_v_calibration()
{
	//resets to the default calibration - assumes all components have 0% tolerance.
	_v_scaling = A2D_ELOAD_DEFAULT_V_SCALING;
	_v_offset = A2D_ELOAD_DEFAULT_V_OFFSET;
}

void A2D_Eload::reset_t_calibration()
{
	_temp_sha = A2D_ELOAD_DEFAULT_TEMP_SHA;
	_temp_shb = A2D_ELOAD_DEFAULT_TEMP_SHB;
	_temp_shc = A2D_ELOAD_DEFAULT_TEMP_SHC;
}

void A2D_Eload::reset_i_calibration()
{
	_i_scaling = A2D_ELOAD_DEFAULT_I_SCALING;
	_i_offset = A2D_ELOAD_DEFAULT_I_OFFSET;
}

void A2D_Eload::save_v_calibration()
{
	EEPROM.put(_ee_addr_v_off, _v_offset);
	EEPROM.put(_ee_addr_v_scale, _v_scaling);
}

void A2D_Eload::save_t_calibration()
{
	EEPROM.put(_ee_addr_t_sha, _temp_sha);
	EEPROM.put(_ee_addr_t_shb, _temp_shb);
	EEPROM.put(_ee_addr_t_shc, _temp_shc);
}

void A2D_Eload::save_i_calibration()
{
	EEPROM.put(_ee_addr_i_off, _i_offset);
	EEPROM.put(_ee_addr_i_scale, _i_scaling);
}

void A2D_Eload::save_all_calibration()
{
	save_v_calibration();
	save_t_calibration();
	save_i_calibration();
}

void A2D_Eload::reset_all_calibration()
{
	reset_v_calibration();
	reset_t_calibration();
	reset_i_calibration();
}

float A2D_Eload::get_cal_v_offset()
{
	return _v_offset;
}

float A2D_Eload::get_cal_v_gain()
{
	return _v_scaling;
}

float A2D_Eload::get_cal_i_offset()
{
	return _i_offset;
}

float A2D_Eload::get_cal_i_gain()
{
	return _i_scaling;
}

void A2D_Eload::set_current_target(float current)
{
	_dac->set_dac(_convert_current_target_to_voltage(current));
}

float A2D_Eload::get_current_target()
{
	return _convert_voltage_to_current_target(get_dac_voltage());
}

void A2D_Eload::_init_eeprom()
{
	//RS485 Address
	set_rs485_addr(A2D_ELOAD_DEFAULT_RS485_ADDR);
	save_rs485_addr();
	
	//Serial Number - only put if SERIAL is defined
	//#ifdef SERIAL
	//for(int i = 0; i < (A2D_ELOAD_SERIAL_CHAR_LEN-1); i++)
	//{
	//	EEPROM.put(_ee_addr_serial + i*sizeof(*_serial), _serial[i]);
	//	Serial.print("put: ");
	//	Serial.print(_serial[i], HEX);
	//	Serial.print(" at ");
	//	Serial.println(_ee_addr_serial + i*sizeof(*_serial));
	//}
	//#endif

	//Calibration Values
	reset_all_calibration();
	save_all_calibration();

	EEPROM.put(_ee_addr_initialized, A2D_ELOAD_EEPROM_INIT_VAL);
}

void A2D_Eload::_init_from_eeprom()
{
	//Serial.println("_init_from_eeprom");
	
	//check the _ee_initialized byte
	EEPROM.get(_ee_addr_initialized, _ee_initialized);
	
	//if it is not correct, then load the default values in EEPROM
	//this should only need to happen the first time the board is programmed
	if(_ee_initialized != A2D_ELOAD_EEPROM_INIT_VAL)
	{
		//Serial.println("_ee_initialized_not_correct");
		_init_eeprom();
	}
	

	//now load the values from EEPROM to the class variables
	/*
	bool _serial_defined = false; //there seems to be a bug with #ifdef in Arduino IDE
	#ifdef SERIAL
	_serial_defined = true;
	#endif
	//#ifndef SERIAL //only load if SERIAL is not defined
	Serial.print("_serial_defined: ");
	Serial.println(_serial_defined);
	if(!_serial_defined)
	{
		Serial.println("Loading serial number");
		for(uint8_t i = 0; i < (A2D_ELOAD_SERIAL_CHAR_LEN-1); i++)
		{
			EEPROM.get(_ee_addr_serial + i*sizeof(*_serial), _serial[i]); //serial number
			Serial.print("got: ");
			Serial.print(_serial[i], HEX);
			Serial.print(" from ");
			Serial.println(_ee_addr_serial + i*sizeof(*_serial));
		}
		_serial[A2D_ELOAD_SERIAL_CHAR_LEN-1] = '\0';
	}
	//#endif
	*/

	EEPROM.get(_ee_addr_rs485_addr, _rs485_addr); //RS485 Address
	
	//voltage calibration
	EEPROM.get(_ee_addr_v_off, _v_offset);
	EEPROM.get(_ee_addr_v_scale, _v_scaling);

	//temperature calibration
	EEPROM.get(_ee_addr_t_sha, _temp_sha);
	EEPROM.get(_ee_addr_t_shb, _temp_shb);
	EEPROM.get(_ee_addr_t_shc, _temp_shc);

	//current calibration
	EEPROM.get(_ee_addr_i_off, _i_offset);
	EEPROM.get(_ee_addr_i_scale, _i_scaling);

	//Serial.println("_init_from_eeprom complete");
}

void A2D_Eload::set_rs485_addr(uint8_t rs485_addr)
{
	_rs485_addr = rs485_addr;
}

void A2D_Eload::save_rs485_addr()
{
	EEPROM.put(_ee_addr_rs485_addr, _rs485_addr);
}

uint8_t A2D_Eload::get_rs485_addr()
{
	return _rs485_addr;
}

void A2D_Eload::set_relay(bool state)
{
	set_current_target(0);
	if(state)
	{
		digitalWrite(A2D_ELOAD_RELAY_PIN, A2D_ELOAD_RELAY_ON);
		_relay_state = true;
	}
	else
	{
		digitalWrite(A2D_ELOAD_RELAY_PIN, A2D_ELOAD_RELAY_OFF);
		_relay_state = false;
	}
}

bool A2D_Eload::get_relay()
{
	return _relay_state;
}

void A2D_Eload::set_fan(bool state)
{
	if(state)
	{
		digitalWrite(A2D_ELOAD_FAN_PIN, A2D_ELOAD_FAN_ON);
		_fan_state = true;
	}
	else
	{
		digitalWrite(A2D_ELOAD_FAN_PIN, A2D_ELOAD_FAN_OFF);
		_fan_state = false;
	}
}

bool A2D_Eload::get_fan()
{
	return _fan_state;
}

/*
void A2D_Eload::set_fan_speed(float speed)
{
	//speed is 0-1.
	//This DOES change the speed, but anything below 1 makes a squealing sound
	if(speed >= 0.0 && speed <= 1.0)
	{
		analogWrite(A2D_ELOAD_FAN_PIN, uint8_t(speed*255));
	}
}
*/

void A2D_Eload::set_led(bool state)
{
	if(state)
	{
		digitalWrite(A2D_ELOAD_LED_PIN, A2D_ELOAD_LED_ON);
		_led_state = true;
	}
	else
	{
		digitalWrite(A2D_ELOAD_LED_PIN, A2D_ELOAD_LED_OFF);
		_led_state = false;
	}
}

bool A2D_Eload::get_led()
{
	return _led_state;
}

float A2D_Eload::_convert_adc_voltage_to_voltage(float voltage)
{
	return (voltage - _v_offset) * _v_scaling;
}

float A2D_Eload::_convert_current_target_to_voltage(float current_target)
{
	if(current_target <= 0)
	{
		current_target = 0;
	}
	else if (current_target >= A2D_ELOAD_MAX_CURRENT)
	{
		current_target = A2D_ELOAD_MAX_CURRENT;
	}

	return (current_target / _i_scaling) + _i_offset;
}

float A2D_Eload::_convert_voltage_to_current_target(float voltage)
{
	return (voltage - _i_offset) * _i_scaling;
}

float A2D_Eload::_convert_adc_voltage_to_temperature(float voltage)
{
	//calculate resistance = voltage / current - series resistance
	float ntc_resistance = voltage / ((A2D_ELOAD_ADC_VREF - voltage)/A2D_ELOAD_NTC_TOP_RES) - A2D_ELOAD_NTC_SERIES_RES;

	//calculate temperature with steinhart-hart constants
	return ((1/(_temp_sha + _temp_shb * log(ntc_resistance) + _temp_shc * (pow(log(ntc_resistance),3)))) - 273.15);
}

void A2D_Eload::set_rs485_receive(bool state)
{
	if(state)
	{
		digitalWrite(A2D_ELOAD_RS485_DE_PIN, A2D_ELOAD_RS485_RECEIVE);
	}
	else
	{
		digitalWrite(A2D_ELOAD_RS485_DE_PIN, A2D_ELOAD_RS485_TRANSMIT);
	}
}

char* A2D_Eload::get_serial_num()
{
	return _serial;
}

void A2D_Eload::force_eeprom_reinit()
{
	_init_eeprom();
	reset();
}
