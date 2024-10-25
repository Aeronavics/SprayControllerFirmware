/*
 * spray_driver.hpp
 *
 *  Created on: Oct 21, 2024
 *      Author: jmorritt
 */

#ifndef INC_SPRAY_DRIVER_HPP_
#define INC_SPRAY_DRIVER_HPP_

#include <stdint.h>

#include "driver_module.hpp"
#include "can_params.hpp"

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_adc_ex.h"

#include "adc.h"

#include "main.h"
#include "tim.h"

#include <com.aeronavics.SprayInfo.h>
#include <com.aeronavics.SprayCtrl.h>
#include <com.aeronavics.LoadcellInfo.h>

#define MIN_SPRAY_WEIGHT 								0.0
#define MAX_SPRAY_WEIGHT 								15.0
#define MIN_SPRAY_PERCENT 							0
#define MAX_SPRAY_PERCENT								100

#define NUM_FLOWRATE_SENSORS						4
#define TELEM_MOVING_AVERAGE_BINS 			10

#define FLOWRATE_SENSOR_SINGLE_ROTATION 0.454
#define CLOCK_SPEED 										170000000
#define UINT12_MAX											4095
#define PRESSURE_SENSOR_VOLTAGE					3.3
#define PRESSURE_SENSOR_MULT						261.17
#define PRESSURE_SENSOR_OFFSET					86.185

#define MAX_PRESSURE										689.47
#define MIN_PRESSURE										68.95

#define FLOWRATE_CHANGE_TIMEOUT					10000

typedef struct
{
		uint8_t array_index;		//current index in array for moving average
		//uint16_t value;			//calculated value
		uint32_t raw_values[TELEM_MOVING_AVERAGE_BINS];	//array of raw values used to calculate a moving average
} telemBase_t;

class Spray_driver : public Driver_module
{
	public:
		void sync_update_unthrottled(void);
		void sync_update_100Hz(void);
		void sync_update_10Hz(void);
		void sync_update_1Hz(void);
		void handle_rx_can(const CanardRxTransfer *transfer,
		                   uint64_t data_type_signature, uint16_t data_type_id,
		                   uint8_t *inout_transfer_id, uint8_t priority,
		                   const void *payload, uint16_t payload_len);

		void timer_capture_callback(TIM_HandleTypeDef *htim);
		void adc_callback(ADC_HandleTypeDef *AdcHandle);

		static Spray_driver& get_driver(void);
		~Spray_driver(void);

	private:

		Driver_state prev_state;

		float spray_right_weight;
		float spray_left_weight;
		float fuel_weight;

		uint8_t spray_level;

		uint16_t desired_flowrate;
		uint16_t desired_pressure;

		uint16_t measured_flowrate;
		uint16_t measured_pressure;

		volatile uint32_t adc_reading;
		volatile telemBase_t pressure_sensor;
		volatile telemBase_t flowrate_sensor[NUM_FLOWRATE_SENSORS];

		uint16_t flowrate_integral;
		uint32_t previous_PID_time;
		uint16_t previous_error;

		uint8_t spray_fault;
		uint32_t flowrate_change_ms;

		Spray_driver(void);
		Spray_driver(Spray_driver const&);		// Poisoned.
		void operator =(Spray_driver const&);	// Poisoned.

		void transmit_telemetry(void);
		void calculate_flowrate(void);
		void calculate_pressure(void);

		uint16_t calculate_spray_ppm(void);

		void check_for_faults(void);

		uint8_t map_spray_level(float weight);
		uint32_t calculate_average(volatile uint32_t *array, uint8_t size);

};

#endif /* INC_SPRAY_DRIVER_HPP_ */
