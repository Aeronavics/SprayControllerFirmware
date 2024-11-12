/*
 * generator.hpp
 *
 *  Created on: Oct 21, 2024
 *      Author: jmorritt
 */

#ifndef INC_GENERATOR_DRIVER_HPP_
#define INC_GENERATOR_DRIVER_HPP_

#include <stdint.h>

#include "driver_module.hpp"
#include "can_params.hpp"

#include "gpio.h"

#include <com.aeronavics.ExtenderInfo.h>
#include <com.aeronavics.ExtenderCtrl.h>
#include <com.aeronavics.LoadcellInfo.h>

#define MIN_FUEL_WEIGHT 		0.0
#define MAX_FUEL_WEIGHT 		5.5
#define MIN_FUEL_PERCENT 		0
#define MAX_FUEL_PERCENT		100

class Generator_driver : public Driver_module
{
	public:

		void sync_update_unthrottled();
		void sync_update_100Hz();
		void sync_update_10Hz();
		void sync_update_1Hz();
		void handle_rx_can(const CanardRxTransfer *transfer,
		                   uint64_t data_type_signature, uint16_t data_type_id,
		                   uint8_t *inout_transfer_id, uint8_t priority,
		                   const void *payload, uint16_t payload_len);

		static Generator_driver& get_driver(void);
		~Generator_driver(void);

	private:

		Driver_state prev_state;
		Generator_driver(void);
		Generator_driver(Generator_driver const&); 	// Poisoned.
		void operator =(Generator_driver const&);		// Poisoned.

		float fuel_weight;
		uint8_t fuel_percent;

		uint16_t generator_engine_speed;
		uint16_t generator_throttle_pos;
		uint8_t generator_motor_temp;
		uint8_t generator_cylinder_temp;
		uint16_t generator_output_voltage;
		uint16_t generator_output_current;
		uint8_t generator_working_state;
		uint16_t generator_alarm;
		uint16_t generator_run_time;

		uint8_t map_fuel_level(float weight);

};

#endif /* INC_GENERATOR_DRIVER_HPP_ */
