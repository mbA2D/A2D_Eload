/* A2D Electronics A2D_4CH_Isolated_ADC Board Library
 * Written By: Micah Black
 * Date: Nov 15, 2023
 *
 */

#include "A2D_4CH_Isolated_ADC.h"
#include "MCP3425.h"

//constructor
A2D_4CH_Isolated_ADC::A2D_4CH_Isolated_ADC()
{	
	_adc_i2c_addrs[0] = A2D_4CH_ISO_ADC_CH1_I2C_ADDR;
	_adc_i2c_addrs[1] = A2D_4CH_ISO_ADC_CH2_I2C_ADDR;
	_adc_i2c_addrs[2] = A2D_4CH_ISO_ADC_CH3_I2C_ADDR;
	_adc_i2c_addrs[3] = A2D_4CH_ISO_ADC_CH4_I2C_ADDR;
	
	//EEPROM Addresses
	_ee_addr_initialized = 0;
	_ee_addr_serial = _ee_addr_initialized + sizeof(_ee_initialized);
	_ee_addr_v_scale[0] = _ee_addr_serial + sizeof(_serial);
	_ee_addr_v_off[0] = _ee_addr_v_scale[0] + sizeof(_v_scaling[0]);
	for(uint8_t i = 1; i < A2D_4CH_ISO_ADC_NUM_CHANNELS; i++)
	{
		_ee_addr_v_scale[i] = _ee_addr_v_off[i-1] + sizeof(_v_offset[i-1]);
		_ee_addr_v_off[i] = _ee_addr_v_scale[i] + sizeof(_v_scaling[i]);
	}
	_ee_addr_rs485_addr = _ee_addr_v_off[A2D_4CH_ISO_ADC_NUM_CHANNELS-1] + sizeof(_v_offset[A2D_4CH_ISO_ADC_NUM_CHANNELS-1]);
	
	//ADC 
	for(uint8_t i = 0; i < A2D_4CH_ISO_ADC_NUM_CHANNELS; i++)
	{
		_adc[i] = new MCP3425();
	}
	
	//Serial number
	#ifdef SERIAL_NUM
	strcpy(_serial, SERIAL_NUM);
	#else
	strcpy(_serial, A2D_4CH_ISO_ADC_DEFAULT_SERIAL_NUM);
	#endif
}

void A2D_4CH_Isolated_ADC::init(TwoWire *i2c)
{
	pinMode(A2D_4CH_ISO_ADC_LED_PIN, OUTPUT);
	pinMode(A2D_4CH_ISO_ADC_RS485_DE_PIN, OUTPUT);
	_i2c = i2c;
	reset();
}

void A2D_4CH_Isolated_ADC::reset()
{
	set_led(false);
	set_rs485_receive(true);
	
	for(uint8_t i = 0; i < A2D_4CH_ISO_ADC_NUM_CHANNELS; i++)
	{
		_adc[i]->init(_adc_i2c_addrs[i]);
		_adc[i]->reset();
		_adc[i]->set_gain(MCP3425_GAIN_1);
		_adc[i]->set_rate(MCP3425_DR_15SPS);
		_adc[i]->set_mode(MCP3425_MODE_CONTINUOUS);
	}
	_init_from_eeprom();
	
}

float A2D_4CH_Isolated_ADC::measure_raw_voltage(uint8_t ch)
{
	if(_validate_ch(ch))
	{
		return _adc[ch]->measure_voltage_continuous();
	}
	return 0.0;
}

float A2D_4CH_Isolated_ADC::measure_voltage(uint8_t ch)
{
	if(_validate_ch(ch))
	{
		float voltage = _adc[ch]->measure_voltage_continuous();
		return _convert_adc_voltage_to_voltage(ch, voltage);
	}
	return 0.0;
}

void A2D_4CH_Isolated_ADC::calibrate_voltage(uint8_t ch, float p1_meas, float p1_act, float p2_meas, float p2_act)
{
	//calculate new offset (b) and scaling (m) in:  actual = m * measured + b
	if(_validate_ch(ch))
	{
		_v_scaling[ch] = (p2_meas - p1_meas) / (p2_act - p1_act); //rise in actual / run in measured
		_v_offset[ch] = p2_act - (1/_v_scaling[ch]) * p2_meas; //b = actual - m * measured
	}
}

void A2D_4CH_Isolated_ADC::reset_calibration(uint8_t ch)
{
	//resets to the default calibration - assumes all components have 0% tolerance.
	if(_validate_ch(ch))
	{
		_v_scaling[ch] = A2D_4CH_ISO_ADC_DEFAULT_V_SCALING;
		_v_offset[ch] = A2D_4CH_ISO_ADC_DEFAULT_V_OFFSET;
	}
}

void A2D_4CH_Isolated_ADC::reset_all_calibration()
{
	for(uint8_t i = 0; i < A2D_4CH_ISO_ADC_NUM_CHANNELS; i++)
	{
		reset_calibration(i);
	}
}

void A2D_4CH_Isolated_ADC::save_calibration(uint8_t ch)
{
	if(_validate_ch(ch))
	{
		EEPROM.put(_ee_addr_v_off[ch], _v_offset[ch]);
		EEPROM.put(_ee_addr_v_scale[ch], _v_scaling[ch]);
	}
}

void A2D_4CH_Isolated_ADC::save_all_calibration()
{
	for(uint8_t i = 0; i < A2D_4CH_ISO_ADC_NUM_CHANNELS; i++)
	{
		save_calibration(i);
	}
	EEPROM.put(_ee_addr_initialized, A2D_4CH_ISO_ADC_EEPROM_INIT_VAL);
}

float A2D_4CH_Isolated_ADC::get_cal_offset(uint8_t ch)
{
	if(_validate_ch(ch))
	{
		return _v_offset[ch];
	}
	return 0.0;
}

float A2D_4CH_Isolated_ADC::get_cal_gain(uint8_t ch)
{
	if(_validate_ch(ch))
	{
		return _v_scaling[ch];
	}
	return 0.0;
}

void A2D_4CH_Isolated_ADC::_init_eeprom()
{
	//RS485 Address
	set_rs485_addr(A2D_4CH_ISO_ADC_DEFAULT_RS485_ADDR);
	save_rs485_addr();
	
	//Serial Number
	EEPROM.put(_ee_addr_serial, _serial);
	
	//Calibration Values
	reset_all_calibration();
	save_all_calibration();
}

void A2D_4CH_Isolated_ADC::_init_from_eeprom()
{
	//check the _ee_initialized byte
	EEPROM.get(_ee_addr_initialized, _ee_initialized);
	
	//if it is not correct, then load the default values in EEPROM
	if(_ee_initialized != A2D_4CH_ISO_ADC_EEPROM_INIT_VAL)
	{
		_init_eeprom();
	}
	
	//now load the values from EEPROM to the class variables
	EEPROM.get(_ee_addr_serial, _serial);
	EEPROM.get(_ee_addr_rs485_addr, _rs485_addr); //RS485 Address
	
	for(uint8_t i = 0; i < A2D_4CH_ISO_ADC_NUM_CHANNELS; i++)
	{
		EEPROM.get(_ee_addr_v_off[i], _v_offset[i]);
		EEPROM.get(_ee_addr_v_scale[i], _v_scaling[i]);
	}
}

void A2D_4CH_Isolated_ADC::set_rs485_addr(uint8_t rs485_addr)
{
	_rs485_addr = rs485_addr;
}

void A2D_4CH_Isolated_ADC::save_rs485_addr()
{
	EEPROM.put(_ee_addr_rs485_addr, _rs485_addr);
}

uint8_t A2D_4CH_Isolated_ADC::get_rs485_addr()
{
	return _rs485_addr;
}

void A2D_4CH_Isolated_ADC::set_led(bool state)
{
	if(state)
	{
		digitalWrite(A2D_4CH_ISO_ADC_LED_PIN, A2D_4CH_ISO_ADC_LED_ON);
	}
	else
	{
		digitalWrite(A2D_4CH_ISO_ADC_LED_PIN, A2D_4CH_ISO_ADC_LED_OFF);
	}
	
}

void A2D_4CH_Isolated_ADC::set_gain(uint8_t ch, uint8_t gain)
{
	if(_validate_ch(ch))
	{
		if(_validate_gain(gain))
		{
			_adc[ch]->set_gain(gain);
		}
	}
}

void A2D_4CH_Isolated_ADC::set_data_rate(uint8_t ch, uint8_t data_rate)
{
	if(_validate_ch(ch))
	{
		if(_validate_data_rate(data_rate))
		{
			_adc[ch]->set_rate(data_rate);
		}
	}
}

void A2D_4CH_Isolated_ADC::set_mode(uint8_t ch, uint8_t mode)
{
	if(_validate_ch(ch))
	{
		if(_validate_mode(mode))
		{
			_adc[ch]->set_mode(mode);
		}
	}
}

float A2D_4CH_Isolated_ADC::_convert_adc_voltage_to_voltage(uint8_t ch, float voltage)
{
	if(_validate_ch(ch))
	{
		return (voltage - _v_offset[ch]) * _v_scaling[ch];
	}
	return 0;
}

void A2D_4CH_Isolated_ADC::trigger_all_single_shot()
{
	//changes the device mode to single shot and initiates a conversion for all devices on the bus
	_i2c->beginTransmission(I2C_GENERAL_CALL_MCP3425_ADDRESS);
	_i2c->write(I2C_GENERAL_CALL_MCP3425_CONV);
	_i2c->endTransmission(true);
}

bool A2D_4CH_Isolated_ADC::_validate_ch(uint8_t ch)
{
	return ((ch < A2D_4CH_ISO_ADC_NUM_CHANNELS) && (ch >= 0));
}

bool A2D_4CH_Isolated_ADC::_validate_gain(uint8_t gain)
{
	if(gain==MCP3425_GAIN_1 || gain==MCP3425_GAIN_2 || gain==MCP3425_GAIN_4 || gain==MCP3425_GAIN_8)
	{
		return true;
	}
	return false;
}

bool A2D_4CH_Isolated_ADC::_validate_data_rate(uint8_t data_rate)
{
	if(data_rate==MCP3425_DR_15SPS || data_rate==MCP3425_DR_60SPS || data_rate==MCP3425_DR_240SPS)
	{	
		return true;
	}
	return false;
}

bool A2D_4CH_Isolated_ADC::_validate_mode(uint8_t mode)
{
	if(mode==MCP3425_MODE_CONTINUOUS || mode == MCP3425_MODE_SINGLE_SHOT)
	{
		return true;
	}
	return false;
}

void A2D_4CH_Isolated_ADC::set_rs485_receive(bool state)
{
	if(state)
	{
		digitalWrite(A2D_4CH_ISO_ADC_RS485_DE_PIN, false);
	}
	else
	{
		digitalWrite(A2D_4CH_ISO_ADC_RS485_DE_PIN, true);
	}
}

char* A2D_4CH_Isolated_ADC::get_serial_num()
{
	return _serial;
}

void A2D_4CH_Isolated_ADC::force_eeprom_reinit()
{
	_init_eeprom();
	reset();
}
