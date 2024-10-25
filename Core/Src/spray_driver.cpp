/*
 * spray_driver.cpp
 *
 *  Created on: Oct 21, 2024
 *      Author: jmorritt
 */

#include "spray_driver.hpp"

/**
 * @brief  Creates and returns a singleton instance of the Spray driver
 * @retval Singleton instance of the Spray driver
 */
Spray_driver& Spray_driver::get_driver(void)
{
	// Create a singleton for the driver.
	static Spray_driver singleton;

	// Return the driver.
	return singleton;
}

/**
 * @brief  Spray driver deconstructor
 * @retval None
 */
Spray_driver::~Spray_driver(void)
{
	return;
}

/**
 * @brief  Handles spray drivers unthrottled loop
 * @retval None
 */
void Spray_driver::sync_update_unthrottled(void)
{
	// If we're not in STATE_NORMAL, then we don't do anything yet.
	if(this_state != DRIVER_STATE_NORMAL)
	{
		return;
	}
	return;
}

/**
 * @brief  Handles spray drivers 100Hz loop
 * @retval None
 */
void Spray_driver::sync_update_100Hz(void)
{
	// If we're not in STATE_NORMAL, then we don't do anything yet.
	if(this_state != DRIVER_STATE_NORMAL)
	{
		return;
	}

	// Calculate pressure and flowrate moving averages
	calculate_pressure();
	calculate_flowrate();

	// Run PID controller
	uint16_t target_pwm = calculate_spray_ppm();
	// Update PWM out to spray Pump
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, target_pwm);

	return;
}

/**
 * @brief  Handles spray drivers 10Hz loop
 * @note   Transmits spray status telemetry in this loop.
 * @retval None
 */
void Spray_driver::sync_update_10Hz(void)
{
	// If we're not in STATE_NORMAL, then we don't do anything yet.
	if(this_state != DRIVER_STATE_NORMAL)
	{
		return;
	}

	spray_level = map_spray_level(spray_right_weight + spray_left_weight);

	check_for_faults();

	// Transmit Spray telemetry
	transmit_telemetry();
	return;
}

/**
 * @brief  Handles spray drivers 1Hz loop
 * @note   1Hz loop handles all housekeeping.
 * @retval None
 */
void Spray_driver::sync_update_1Hz(void)
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
			/* Intialise ADCs*/
			if(HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
			{
				/* Calibration Error */
				Error_Handler();
			}

			if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_reading, 1) != HAL_OK)
			{
				/* Start Error */
				Error_Handler();
			}

			/* Initialise Timers */
			MX_TIM2_Init();

			HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
			HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
			HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
			HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);

			MX_TIM2_Init();

			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

			Libcanard_module::get_driver().sendLog(impl_::LogLevel::Debug,
			                                       "Spray Driver initialized");
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

/**
 * @brief  Handles incoming CAN messages
 * @param  transfer : The CAN message
 * @param  data_type_signature : CAN signature of the transfer
 * @param	data_type_id : CAN ID of the transfer
 * @param	inout_transfer_id : transfer id of the transfer
 * @param	priority : Priority of the transfer
 * @param  payload : CAN payload of the transfer
 * @param	payload_len : Length of the CAN payload
 * @note   Listens to the incoming can messages and processes them if required.
 * @retval None
 */
void Spray_driver::handle_rx_can(const CanardRxTransfer *transfer,
                                 uint64_t data_type_signature,
                                 uint16_t data_type_id,
                                 uint8_t *inout_transfer_id, uint8_t priority,
                                 const void *payload, uint16_t payload_len)
{
	//Let us subscribe to the LoadcellInfo message.
	if(transfer->data_type_id == COM_AERONAVICS_LOADCELLINFO_ID)
	{
		com_aeronavics_LoadcellInfo loadcell_info;
		//decode the actual transfer.
		com_aeronavics_LoadcellInfo_decode(transfer, &loadcell_info);

		if(loadcell_info.sensor_id == COM_AERONAVICS_LOADCELLINFO_FUEL_TANK)
		{
			fuel_weight = loadcell_info.weight;
		}
		else if(loadcell_info.sensor_id
		    == COM_AERONAVICS_LOADCELLINFO_SPRAY_TANK_RIGHT)
		{
			spray_right_weight = loadcell_info.weight;
		}
		else if(loadcell_info.sensor_id
		    == COM_AERONAVICS_LOADCELLINFO_SPRAY_TANK_LEFT)
		{
			spray_left_weight = loadcell_info.weight;
		}
	}
	//Let us subscribe to the SprayCtrl message.
	else if(transfer->data_type_id == COM_AERONAVICS_SPRAYCTRL_ID)
	{
		com_aeronavics_SprayCtrl Spray_control;
		com_aeronavics_SprayCtrl_decode(transfer, &Spray_control);

		if(desired_flowrate > 0 && Spray_control.flow_rate == 0)
		{
			spray_fault = 0;
		}
		else if(desired_flowrate != Spray_control.flow_rate)
		{
			flowrate_change_ms = HAL_GetTick();
		}

		desired_flowrate = Spray_control.flow_rate;
		desired_pressure = Spray_control.pressure;
	}
}

/**
 * @brief  Transmits spray driver telemetry
 * @note   Tramsmits a com_aronavics_SprayInfo msg over CAN
 * 				after checking for errors and calculating values
 * @retval None
 */
void Spray_driver::transmit_telemetry(void)
{
	uint8_t transfer_id = 0;
	uint32_t length = 0;

	com_aeronavics_SprayInfo spray_info;

	float total_weight = spray_right_weight + spray_left_weight + fuel_weight;

	spray_info.spray_remaining = spray_level;
	spray_info.tank_weight = total_weight;
	spray_info.flow_rate = measured_flowrate;
	spray_info.pressure = measured_pressure;
	spray_info.error_flags = spray_fault;

	uint8_t spray_info_buffer[COM_AERONAVICS_SPRAYINFO_MAX_SIZE];
	length = com_aeronavics_SprayInfo_encode(&spray_info, spray_info_buffer);
	driverhost_broadcast_can(nullptr,
	COM_AERONAVICS_SPRAYINFO_SIGNATURE,
	                         COM_AERONAVICS_SPRAYINFO_ID, &transfer_id,
	                         CAN_TRANSFER_PRIORITY_MEDIUM,
	                         &spray_info_buffer, length, this);

}

/**
 * @brief  Checks for faults in the spray system.
 * @note   spray_fault is updated based on any faults found.
 * @retval None
 */
void Spray_driver::check_for_faults(void)
{
	// Report an error if there is no spray remaining
	if(spray_level == 0 && (spray_fault >> 6) & 0x01 == 0)
	{
		spray_fault += COM_AERONAVICS_SPRAYINFO_ERROR_NO_SPRAY;
	}

	// Report an error if the pressure is greater or equal to the max pressure
	if(measured_pressure >= MAX_PRESSURE && (spray_fault >> 5) & 0x01 == 0)
	{
		spray_fault += COM_AERONAVICS_SPRAYINFO_ERROR_OVER_PRESSURE;
	}

	// Report a low pressure error if flow rate is correct but pressure is unreasonably low
	if((measured_flowrate < desired_flowrate * 1.1
	    || measured_flowrate > desired_flowrate * 0.9)
	    && (measured_pressure < MIN_PRESSURE))
	{
		spray_fault += COM_AERONAVICS_SPRAYINFO_ERROR_LOW_PRESSURE;
	}

	// Report an error if the flow rate is not within 10% of the desired within 10 seconds
	if((measured_flowrate > desired_flowrate * 1.1
	    || measured_flowrate < desired_flowrate * 0.9)
	    && (flowrate_change_ms - HAL_GetTick() > FLOWRATE_CHANGE_TIMEOUT)
	    && ((spray_fault >> 0) & 0x01 == 0))
	{
		spray_fault += COM_AERONAVICS_SPRAYINFO_ERROR_FLOW_RATE_1;
	}
}

/**
 * @brief  Calculates the pressure of the system
 * @note   Calculates the moving average of the pressure sensor.
 *					measured_pressure is then updated to the calculated value.
 * @retval None
 */
void Spray_driver::calculate_pressure(void)
{
	// get moving average of ADC readings
	uint32_t average_adc = calculate_average(pressure_sensor.raw_values,
	                                         TELEM_MOVING_AVERAGE_BINS);
	// Convert average ADC reading to voltage
	float measured_voltage = ((float) average_adc) / UINT12_MAX
	    * PRESSURE_SENSOR_VOLTAGE;
	// Convert voltage to pressure (kpa) with equation from datasheet
	float calculated_pressure = (PRESSURE_SENSOR_MULT * measured_voltage)
	    - PRESSURE_SENSOR_OFFSET;

	if(calculated_pressure < 0.0)
	{
		measured_pressure = 0;
	}
	else
	{
		measured_pressure = (uint16_t) calculated_pressure;
	}
}

/**
 * @brief  Calculates the total flowrate of the system
 * @note   Calculates the moving average of the periods recorded
 * 				for each flowrate sensor and converts that into ml/s.
 *					All flowrates are summed together to get the total flowrate of the system.
 *					measured_flowrate is then updated to the total flowrate.
 * @retval None
 */
void Spray_driver::calculate_flowrate(void)
{
	uint16_t total_flowrate = 0;
	for(uint8_t i = 0; i < NUM_FLOWRATE_SENSORS; i++)
	{
		uint32_t average_period = calculate_average(flowrate_sensor[i].raw_values,
		                                            TELEM_MOVING_AVERAGE_BINS);
		if(average_period != 0)
		{
			total_flowrate += (uint16_t) (FLOWRATE_SENSOR_SINGLE_ROTATION
			    * (CLOCK_SPEED / average_period));
		}
	}
	measured_flowrate = total_flowrate;
}

/**
 * @brief  Maps spray weight to percentage
 * @param  weight : Measured weight of the spray
 * @note   Calculates a percentage of spray remaining based on a given weight.
 *					The weight of the spray is assumed to be within MIN_SPRAY_WEGHT & MIN_SPRAY_WEGHT.
 *					The output percentage will be between MIN_SPRAY_PERCENT & MAX_SPRAY_PERCENT.
 * @retval A percentage of spray remaining
 */
uint8_t Spray_driver::map_spray_level(float weight)
{
	return (uint8_t) ((weight - MIN_SPRAY_WEIGHT)
	    * (MAX_SPRAY_PERCENT - MIN_SPRAY_PERCENT)
	    / (MAX_SPRAY_WEIGHT - MIN_SPRAY_WEIGHT) + MIN_SPRAY_PERCENT);
}

/**
 * @brief  Calculates the average of a given array
 * @param  array : Array to calculate average over
 * @param  size	: Size of the array
 * @retval uint16_t average of the array
 */
uint32_t Spray_driver::calculate_average(volatile uint32_t *array, uint8_t size)
{
	uint64_t temp_value = 0;
	for(uint8_t i = 0; i < size; i++)
	{
		temp_value += array[i];
	}
	return (uint32_t) (temp_value / size);
}

/**
 * @brief  Calculates the PWM to send to the Spray pump
 * @note		A PID controller used to calculate the PWM value to
 * 				send to the spray pump to achieve the desired flowrate.
 * @retval uint16_t Spray pump PWM value
 */
uint16_t Spray_driver::calculate_spray_ppm(void)
{
	// PID calculations
	float error = desired_flowrate - measured_flowrate;

	float p = get_can_param_by_id(CAN_PARAM_IDX_KD)->value.real_value * error;

	flowrate_integral = flowrate_integral
	    + get_can_param_by_id(CAN_PARAM_IDX_KI)->value.real_value * error
	        * (HAL_GetTick() - previous_PID_time);

	float d = get_can_param_by_id(CAN_PARAM_IDX_KD)->value.real_value
	    * (error - previous_error) / (HAL_GetTick() - previous_PID_time);

	uint16_t output = (uint16_t) (p + flowrate_integral + d);

	previous_error = error;
	previous_PID_time = HAL_GetTick();

	return output;
}

/**
 * @brief  Conversion complete callback in non blocking mode
 * @param  AdcHandle : ADC handle
 * @note   Adds the ADC readings into their respective sensor structures
 *					and calls the functions to calculate the moving averages for each sensor.
 * @retval None
 */
void Spray_driver::adc_callback(ADC_HandleTypeDef *AdcHandle)
{
	pressure_sensor.raw_values[pressure_sensor.array_index
	    % TELEM_MOVING_AVERAGE_BINS] = adc_reading;
	pressure_sensor.array_index = ++pressure_sensor.array_index
	    % TELEM_MOVING_AVERAGE_BINS;
}

/**
 * @brief  Timer interrupt callback
 * @param  htim : Timer handle
 * @note   Read the period between two rising edges from the flowrate timers
 * @retval None
 */
void Spray_driver::timer_capture_callback(TIM_HandleTypeDef *htim)
{
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		flowrate_sensor[0].raw_values[flowrate_sensor[0].array_index
		    % TELEM_MOVING_AVERAGE_BINS] = HAL_TIM_ReadCapturedValue(htim,
		                                                             TIM_CHANNEL_1);
		flowrate_sensor[0].array_index = ++flowrate_sensor[0].array_index
		    % TELEM_MOVING_AVERAGE_BINS;
	}
	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		flowrate_sensor[1].raw_values[flowrate_sensor[1].array_index
		    % TELEM_MOVING_AVERAGE_BINS] = HAL_TIM_ReadCapturedValue(htim,
		                                                             TIM_CHANNEL_2);
		flowrate_sensor[1].array_index = ++flowrate_sensor[1].array_index
		    % TELEM_MOVING_AVERAGE_BINS;
	}
	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	{
		flowrate_sensor[2].raw_values[flowrate_sensor[2].array_index
		    % TELEM_MOVING_AVERAGE_BINS] = HAL_TIM_ReadCapturedValue(htim,
		                                                             TIM_CHANNEL_3);
		flowrate_sensor[2].array_index = ++flowrate_sensor[2].array_index
		    % TELEM_MOVING_AVERAGE_BINS;
	}
	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
	{
		flowrate_sensor[3].raw_values[flowrate_sensor[3].array_index
		    % TELEM_MOVING_AVERAGE_BINS] = HAL_TIM_ReadCapturedValue(htim,
		                                                             TIM_CHANNEL_4);
		flowrate_sensor[3].array_index = ++flowrate_sensor[3].array_index
		    % TELEM_MOVING_AVERAGE_BINS;
	}
}

Spray_driver::Spray_driver(void)
{
	// Initialise the driver state machine.
	prev_state = DRIVER_STATE_UNKNOWN;
	// All done.
	return;
}

