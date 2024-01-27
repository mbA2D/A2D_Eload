/* A2D Electronics A2D_4CH_Isolated_ADC Board Library
 * Written By: Micah Black
 * Date: Nov 15, 2023
 * 
 */


#ifndef A2D_4CH_Isolated_ADC_h
#define A2D_4CH_Isolated_ADC_h

#include <Arduino.h>
#include <Wire.h>
#include "A2D_4CH_Isolated_ADC_V1.0.h" //header file with pins, etc
#include "MCP3425.h"
#include <EEPROM.h>

class A2D_4CH_Isolated_ADC
{
	public:
		A2D_4CH_Isolated_ADC(); //constructor
		
		//scaling for voltages
		float _v_scaling[A2D_4CH_ISO_ADC_NUM_CHANNELS]; // V/V
		float _v_offset[A2D_4CH_ISO_ADC_NUM_CHANNELS]; //V
		
		//Configuration
		void init(TwoWire *i2c = &Wire);
		void reset();
		
		//Interface
		float measure_voltage(uint8_t ch);
		float measure_raw_voltage(uint8_t ch);
		void calibrate_voltage(uint8_t ch, float p1_meas, float p1_act, float p2_meas, float p2_act);
		void reset_calibration(uint8_t ch);
		void reset_all_calibration();
		void save_calibration(uint8_t ch);
		void save_all_calibration();
		void set_led(bool state);
		void set_rs485_receive(bool state);
		
		void set_gain(uint8_t ch, uint8_t gain);
		void set_data_rate(uint8_t ch, uint8_t data_rate);
		void set_mode(uint8_t ch, uint8_t mode);
		void trigger_all_single_shot(); //triggers all 4 channels simultaneously
		
		void set_rs485_addr(uint8_t rs485_addr);
		void save_rs485_addr();
		uint8_t get_rs485_addr();
		
		float get_cal_offset(uint8_t ch);
		float get_cal_gain(uint8_t ch);
		
		char* get_serial_num();
		void force_eeprom_reinit();

	private:
		//************METHODS****************
		float _convert_adc_voltage_to_voltage(uint8_t ch, float voltage);
		void _init_eeprom();
		void _init_from_eeprom();
		bool _validate_ch(uint8_t ch);
		bool _validate_gain(uint8_t gain);
		bool _validate_data_rate(uint8_t data_rate);
		bool _validate_mode(uint8_t mode);
		
		
		//*********VARIABLES/CLASSES*********
		uint8_t _ee_initialized;
		char _serial[A2D_4CH_ISO_ADC_SERIAL_CHAR_LEN];
		uint8_t _rs485_addr;
		TwoWire *_i2c;
		
		//EEPROM Addresses
		int _ee_addr_initialized;
		int _ee_addr_serial;
		int _ee_addr_rs485_addr;
		int _ee_addr_v_off[A2D_4CH_ISO_ADC_NUM_CHANNELS];
		int _ee_addr_v_scale[A2D_4CH_ISO_ADC_NUM_CHANNELS];
		
		//ADC
		uint8_t _adc_i2c_addrs[A2D_4CH_ISO_ADC_NUM_CHANNELS];
		MCP3425* _adc[A2D_4CH_ISO_ADC_NUM_CHANNELS];
};

#endif