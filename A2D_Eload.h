/* A2D Electronics A2D_Eload_V1.0 Board Library
 * Written By: Micah Black
 * Date: Jan 27, 2024
 * 
 */


#ifndef A2D_Eload_h
#define A2D_Eload_h

#include <Arduino.h>
#include <Wire.h>
#include "A2D_Eload_V1.0.h" //header file with pins, etc
#include <EEPROM.h>
#include "DACX0501.h"

class A2D_Eload
{
	public:
		A2D_Eload(); //constructor
		
		//Configuration
		void init(TwoWire *i2c = &Wire);
		void reset();
		
		//Voltage sense for 24V Input
		float measure_voltage();
		float measure_raw_voltage();
		void calibrate_voltage(float p1_meas, float p1_act, float p2_meas, float p2_act);
		void reset_v_calibration();
		void save_v_calibration();
		float get_cal_v_offset();
		float get_cal_v_gain();


		//Temperature of heatsink
		float measure_temperature();
		void set_sh_constants(float sha, float shb, float shc);
		void reset_t_calibration();
		void save_t_calibration();

		//current targets
		void reset_i_calibration();
		void save_i_calibration();
		uint16_t get_dac_voltage();
		void set_current_target(float current);
		float get_current_target();
		float get_cal_i_offset();
		float get_cal_i_gain();

		//general calibration
		void reset_all_calibration();
		void save_all_calibration();

		void set_led(bool state);

		void set_fan(bool state);
		void set_fan_speed(float speed);

		void set_relay(bool state);

		//RS485
		void set_rs485_receive(bool state);
		void set_rs485_addr(uint8_t rs485_addr);
		void save_rs485_addr();
		uint8_t get_rs485_addr();
		
		char* get_serial_num();
		void force_eeprom_reinit();

	private:
		//************METHODS****************
		float _convert_adc_voltage_to_voltage(float voltage);
		float _convert_adc_voltage_to_temperature(float voltage);
		float _convert_current_target_to_voltage(float current_target);
		float _convert_voltage_to_current_target(float voltage);
		void _init_eeprom();
		void _init_from_eeprom();
		
		//*********VARIABLES/CLASSES*********
		uint8_t _ee_initialized;
		char _serial[A2D_ELOAD_SERIAL_CHAR_LEN];
		uint8_t _rs485_addr;
		TwoWire *_i2c;
		
		//scaling for voltages
		float _v_scaling; // V/V
		float _v_offset; //V

		float _temp_sha;
		float _temp_shb;
		float _temp_shc;

		float _i_scaling;
		float _i_offset;

		//EEPROM Addresses
		int _ee_addr_initialized;
		int _ee_addr_serial;
		int _ee_addr_rs485_addr;
		int _ee_addr_v_off;
		int _ee_addr_v_scale;
		int _ee_addr_t_sha;
		int _ee_addr_t_shb;
		int _ee_addr_t_shc;
		int _ee_addr_i_scale;
		int _ee_addr_i_off;
		
		//DAC
		uint8_t _dac_i2c_addr;
		DACX0501* _dac;
};

#endif