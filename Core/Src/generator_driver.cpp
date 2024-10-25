/*
 * generator.cpp
 *
 *  Created on: Oct 21, 2024
 *      Author: jmorritt
 */

#include "generator_driver.hpp"

Generator_driver& Generator_driver::get_driver(void)
{
	// Create a singleton for the driver.
	static Generator_driver singleton;

	// Return the driver.
	return singleton;
}

Generator_driver::~Generator_driver(void)
{
	return;
}

void Generator_driver::sync_update_unthrottled(void)
{
	// If we're not in STATE_NORMAL, then we don't do anything yet.
	if(this_state != DRIVER_STATE_NORMAL)
	{
		return;
	}
	return;
}

void Generator_driver::sync_update_100Hz(void)
{
	// If we're not in STATE_NORMAL, then we don't do anything yet.
	if(this_state != DRIVER_STATE_NORMAL)
	{
		return;
	}
	return;
}

void Generator_driver::sync_update_10Hz(void)
{
	// If we're not in STATE_NORMAL, then we don't do anything yet.
	if(this_state != DRIVER_STATE_NORMAL)
	{
		return;
	}
	return;
}

void Generator_driver::sync_update_1Hz(void)
{
	// Run the driver housekeeping state machine.

	// If we don't specify a new state, assume we will just remain in the current state.
	Driver_state next_state = this_state;

	// Select behaviour depending on the current state.
	switch(this_state)
	{
		case DRIVER_STATE_UNKNOWN:
		{
			// If we don't know which state we are in, then we probably want to initialise.
			next_state = DRIVER_STATE_INIT;
			break;
		}

		case DRIVER_STATE_INIT:
		{
			Libcanard_module::get_driver().sendLog(impl_::LogLevel::Debug,
			                                       "Generator Driver initialized");
			next_state = DRIVER_STATE_NORMAL;
			break;
		}

		case DRIVER_STATE_NORMAL:
		{
			break;
		}

		case DRIVER_STATE_ERROR:
		{
			// We'll attempt to reinitialise; that's about all we can hope for.
			next_state = DRIVER_STATE_INIT;
			break;
		}

		default:
		{
			// We shouldn't ever end up here.
			this_state = DRIVER_STATE_UNKNOWN;
			next_state = DRIVER_STATE_UNKNOWN;
			break;
		}
	}

	// Advance to the next state.
	prev_state = this_state;
	this_state = next_state;

	// All done.

	return;
}

void Generator_driver::handle_rx_can(const CanardRxTransfer *transfer,
                                     uint64_t data_type_signature,
                                     uint16_t data_type_id,
                                     uint8_t *inout_transfer_id,
                                     uint8_t priority, const void *payload,
                                     uint16_t payload_len)
{
	//Let us subscribe to the ArrayCommand message.
	if(transfer->data_type_id == COM_AERONAVICS_LOADCELLINFO_ID)
	{
		com_aeronavics_LoadcellInfo loadcell_info;
		//decode the actual transfer.
		com_aeronavics_LoadcellInfo_decode(transfer, &loadcell_info);

		if(loadcell_info.sensor_id == COM_AERONAVICS_LOADCELLINFO_FUEL_TANK)
		{
			fuel_weight = loadcell_info.weight;
			fuel_percent = map_fuel_level(fuel_weight);
		}
	}
	else if(transfer->data_type_id == COM_AERONAVICS_EXTENDERINFO_ID)
	{
		com_aeronavics_ExtenderInfo generator_info;
		com_aeronavics_ExtenderInfo_decode(transfer, &generator_info);

		generator_info.FuelPosition = fuel_percent;

		uint8_t generator_info_buffer[COM_AERONAVICS_EXTENDERINFO_MAX_SIZE];

		uint8_t transfer_id = 0;
		uint32_t length = 0;

		length = com_aeronavics_ExtenderInfo_encode(&generator_info,
		                                            generator_info_buffer);
		driverhost_broadcast_can(nullptr,
		COM_AERONAVICS_EXTENDERINFO_SIGNATURE,
		                         COM_AERONAVICS_EXTENDERINFO_ID, &transfer_id,
		                         CAN_TRANSFER_PRIORITY_MEDIUM,
		                         &generator_info_buffer, length, this);
	}
}

uint8_t Generator_driver::map_fuel_level(float weight)
{
	return (uint8_t) ((weight - MIN_FUEL_WEIGHT)
	    * (MIN_FUEL_PERCENT - MAX_FUEL_PERCENT)
	    / (MAX_FUEL_WEIGHT - MIN_FUEL_WEIGHT) + MIN_FUEL_PERCENT);
}

Generator_driver::Generator_driver(void)
{
	// Initialise the driver state machine.
	prev_state = DRIVER_STATE_UNKNOWN;
	// All done.
	return;
}

