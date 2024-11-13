/*
 * SprayController.cpp
 *
 *  Created on: Oct 18, 2024
 *      Author: jmorritt
 */

#include "SprayController.hpp"

#define EVENT_LED_TIMEOUT 10 // Time between flashing sequences for the event LED (in 100s of milliseconds).

/* Private variables ---------------------------------------------------------*/
static volatile bool run_sloppy_100hz = false;
static volatile bool run_sloppy_10hz = false;
static volatile bool run_sloppy_1hz = true;

static Uart_module uart1_driver;

static uint8_t base_mode = 0;
static uint32_t custom_mode = 0;
static uint32_t system_status = MAV_STATE_UNINIT;

// Flashing event (error) status.
static Driver_event_severity current_event_severity;
static Driver_event_code current_event_code;
static uint8_t event_led_flashes;
static uint8_t event_led_timeout;
//uart buffer
static uint64_t systick_counter;
uint8_t counter;

//driver list
static Driver_module *const drivers[] =
{
		static_cast<Driver_module*>(&Libcanard_module::get_driver()),
//    static_cast<Driver_module*>(&Mavlink_params::get_driver()),
//    static_cast<Driver_module*>(&uart1_driver),
    static_cast<Driver_module*>(&Spray_driver::get_driver()),
    static_cast<Driver_module*>(&Generator_driver::get_driver()),
};

static const size_t NUM_DRIVERS = sizeof(drivers) / sizeof(Driver_module*);

/* Private function prototypes -----------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void isr_system_timer_tick(void);
void driverhost_broadcast_rx_raw(unsigned char, Uart_module_t)
{
}

void runDriversUnthrottled(void);
void runDrivers100Hz(bool run);
void runDrivers10Hz(bool run);
void runDrivers1Hz(bool run);

void readDriverStatus(void);

void sendHeartbeat(void);

int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM6_Init();
//	MX_USART1_UART_Init();
	MX_IWDG_Init();

	HAL_TIM_Base_Start_IT(&htim6);


	// Turn off LEDS
	HAL_GPIO_WritePin(ERROR0_GPIO_Port, ERROR0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ERROR1_GPIO_Port, ERROR1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(STATUS1_GPIO_Port, STATUS1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(STATUS2_GPIO_Port, STATUS2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(STATUS3_GPIO_Port, STATUS3_Pin, GPIO_PIN_SET);

	// Turn off Fans
	HAL_GPIO_WritePin(FAN_1_GPIO_Port, FAN_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FAN_2_GPIO_Port, FAN_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FAN_3_GPIO_Port, FAN_3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FAN_4_GPIO_Port, FAN_4_Pin, GPIO_PIN_RESET);


	/* USER CODE BEGIN 2 */
//	uart1_driver.set_uart_module(&huart1, UART_MODULE_UART_1, false, MAVLINK_COMM_0);

	Libcanard_module::get_driver().set_name(UAVCAN_MODULE_NAME);
	/* USER CODE END 2 */

	// FUCKING REQUIRED DO NOT TOUCH
	HAL_Delay(500);

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while(1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		// Clear the watchdog, and reset it to 1 second (1/(40K/16/2500))
		HAL_IWDG_Refresh(&hiwdg);

		// Iterate over each of the drivers and run the relevant handler.
		runDriversUnthrottled();
		runDrivers100Hz(run_sloppy_100hz);
		runDrivers10Hz(run_sloppy_10hz);
		runDrivers1Hz(run_sloppy_1hz);

		// Check if we need to run the sloppy 100Hz tasks.
		if(run_sloppy_100hz)
		{
			// Clear the 100Hz task flag now that the tasks have been run.
			run_sloppy_100hz = false;
		}

		// Check if we need to run the 10Hz tasks.
		if(run_sloppy_10hz)
		{
			// If the event LED timeout is non-zero, then we need to wait a little, so that there are actually gaps between event codes being flashed.
			if(event_led_timeout == 0)
			{
				// If required, we toggle the event LED to flash error codes.
				if(event_led_flashes > 0)
				{
					// One less edge to go.
					event_led_flashes--;

					// Toggle the status of the LED.
					HAL_GPIO_TogglePin(ERROR0_GPIO_Port, ERROR0_Pin);
					// If we're done, start a timeout.
					if(event_led_flashes == 0)
					{
						event_led_timeout = EVENT_LED_TIMEOUT;
					}
				}
				// Else turn the LED off: this prevents weirdness if a sequence gets preempted by a higher priority one when there is an odd number of flashes remaining.
				else
				{
					HAL_GPIO_WritePin(ERROR0_GPIO_Port, ERROR0_Pin, GPIO_PIN_SET);
				}
			}
			else
			{
				event_led_timeout--;
			}
			// Clear the 10Hz task flag now that the tasks have been run.
			run_sloppy_10hz = false;
		}

		// Check if we need to run the 1Hz tasks.
		if(run_sloppy_1hz)
		{
			// // Blink the alive LED, to indicate we are still alive.
			HAL_GPIO_TogglePin(STATUS1_GPIO_Port, STATUS1_Pin);
			// // Update the system status indicators.

			base_mode = ((is_system_armed() ? MAV_MODE_FLAG_SAFETY_ARMED : 0) | 0);

			readDriverStatus();

			// Sending a heart beat message to indicate we are still alive.
			sendHeartbeat();

			// Clear the 1Hz task flag now that the tasks have been run.
			run_sloppy_1hz = false;
		}
	}
	/* USER CODE END 3 */
}

void sendHeartbeat()
{
	// Send a heartbeat message to indicate we are still alive.
	mavlink_message_t msg;
	mavlink_msg_heartbeat_pack(get_sID(), get_cID(), &msg,
	                           MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID,
	                           0, custom_mode, system_status);
	driverhost_broadcast_mavlink(msg, nullptr);
}
void runDriversUnthrottled()
{
	static uint8_t driverIndex_to_run = 0;

	if(drivers[driverIndex_to_run] != nullptr)
	{
		drivers[driverIndex_to_run]->sync_update_unthrottled();
	}
	if(++driverIndex_to_run == NUM_DRIVERS)
	{
		driverIndex_to_run = 0;
	}
}
void runDrivers100Hz(bool run)
{
	static bool isRunning = false;
	static uint8_t driverIndex_to_run = 0;

	if(run)
		isRunning = true;

	if(isRunning)
	{
		if(drivers[driverIndex_to_run] != nullptr)
		{
			drivers[driverIndex_to_run]->sync_update_100Hz();
		}
		if(++driverIndex_to_run == NUM_DRIVERS)
		{
			driverIndex_to_run = 0;
			isRunning = false;
		}
	}
}
void runDrivers10Hz(bool run)
{
	static bool isRunning = false;
	static uint8_t driverIndex_to_run = 0;

	if(run)
		isRunning = true;
	if(isRunning)
	{
		if(drivers[driverIndex_to_run] != nullptr)
		{
			drivers[driverIndex_to_run]->sync_update_10Hz();
		}
		if(++driverIndex_to_run == NUM_DRIVERS)
		{
			driverIndex_to_run = 0;
			isRunning = false;
		}
	}
}
void runDrivers1Hz(bool run)
{
	static bool isRunning = false;
	static uint8_t driverIndex_to_run = 0;

	if(run)
		isRunning = true;

	if(isRunning)
	{
		if(drivers[driverIndex_to_run] != nullptr)
		{
			drivers[driverIndex_to_run]->sync_update_1Hz();
		}
		if(++driverIndex_to_run == NUM_DRIVERS)
		{
			driverIndex_to_run = 0;
			isRunning = false;
		}
	}
}
void readDriverStatus()
{
	bool are_drivers_init = false;
	bool are_drivers_normal = false;
	bool are_drivers_error = false;
	for(size_t i = 0; i < NUM_DRIVERS; i++)
	{
		if(drivers[i] != nullptr)
		{
			if(drivers[i]->get_state() == DRIVER_STATE_INIT)
				are_drivers_init = true;
			if(drivers[i]->get_state() == DRIVER_STATE_NORMAL)
				are_drivers_normal = true;
			if(drivers[i]->get_state() == DRIVER_STATE_ERROR)
				are_drivers_error = true;
		}
	}
	system_status = MAV_STATE_UNINIT;
	if(are_drivers_error)
		system_status = MAV_STATE_CRITICAL;
	else if(are_drivers_init)
		system_status = MAV_STATE_BOOT;
	else if(are_drivers_normal)
		system_status = (is_system_armed()) ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;
}
void driverhost_broadcast_can(
		const CanardRxTransfer *transfer,
		uint64_t data_type_signature,
		uint16_t data_type_id,
		uint8_t *inout_transfer_id,
		uint8_t priority,
		const void *payload,
		uint16_t payload_len,
#ifdef CANARD_MULTI_IFACE
		uint8_t iface_mask,
#endif
		Driver_module *const except
		)
{
	for(size_t i = 0; i < NUM_DRIVERS; i++)
	{
		// Check we're not supposed to be skipping this module.
		if(drivers[i] != except)
		{
			// Else, pass the message to this driver's message handler.
			if(drivers[i] != nullptr)
				drivers[i]->handle_rx_can(
						transfer,
						data_type_signature,
						data_type_id,
						inout_transfer_id,
						priority,
						payload,
						payload_len
#ifdef CANARD_MULTI_IFACE
						, iface_mask
#endif
						);
		}
	}

	if(transfer->data_type_id
	    == UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_REQUEST_ID)
	{
		bootloader_info_location = RAM_DO_BOOTLOADER_BYTE;
		NVIC_SystemReset();
	}

	return;
}
Driver_module* driverhost_get_can_module(void)
{
	return nullptr;
}

void driverhost_broadcast_mavlink(const mavlink_message_t &message,
                                  Driver_module *const except)
{
	mavlink_message_t msg = message;

	// Iterate over each of the drivers.
	for(size_t i = 0; i < NUM_DRIVERS; i++)
	{
		// Check we're not supposed to be skipping this module.
		if(drivers[i] != except)
		{
			// Else, pass the message to this driver's message handler.
			if(drivers[i] != nullptr)
				drivers[i]->handle_rx_mavlink(msg);
		}
	}

	/*if(message.compid == 1 && message.sysid != get_sID()){  //if our sysID doesn't match the flight controller, lets update
	 set_sID(message.sysid);
	 Mavlink_params::get_driver().request_data_store();
	 }*/

	// All done
	return;
}
void driverhost_report_event(Driver_event_severity severity,
                             Driver_event_code event_code)
{
	// char buffer[100];
	// snprintf(buffer, 100, "Error Occurred, code:%d", event_code);
	// Libcanard_module::get_driver().sendLog(impl_::LogLevel::Error,buffer);

	// Arrange for the event/error LED to flash the required number of times.

	// If we're currently in the process of reporting an event of greater severity, then we just have to mask this one.
	if(event_led_flashes == 0 || severity > current_event_severity)
	{
		//reset the event flashes. if the code is not the same.
		if(event_code != current_event_code)
		{
			event_led_flashes = 0;
			HAL_GPIO_WritePin(ERROR0_GPIO_Port, ERROR0_Pin, GPIO_PIN_SET);
		}
		current_event_severity = severity;
		current_event_code = event_code;

		//queue up flashes so that we can get an idea of how many times this gets calle

		event_led_flashes += event_code * 2; // Queue up twice the number of edges, since you need to toggle between on and off states.
	}

	// All done.
	return;
}

void driverhost_uart_tx_raw(const uint8_t *message, uint16_t size,
                            Uart_module_t target_module)
{
	switch(target_module)
	{
		case UART_MODULE_UART_1:
		{
			uart1_driver.queue_tx_raw(message, size);
			break;
		}
		default:
		{
			break;
		}
	}
	// All done
	return;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		uart1_driver.callback_tx_complete();
	}
}

void HAL_UART_RxIdleCallback(UART_HandleTypeDef *huart)
{

	if(huart->Instance == USART1)
	{
		uart1_driver.callback_rx_idle(false);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		uart1_driver.callback_rx_complete();
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		uart1_driver.reset_uart();
	}
}

// Timer period elapsed callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{
		systick_counter++;
		isr_system_timer_tick();
	}
	if(htim->Instance == TIM1)
	{
		Spray_driver::get_driver().timer_period_callback(htim);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == FLOW_1_Pin || GPIO_Pin == FLOW_2_Pin || GPIO_Pin == FLOW_3_Pin || GPIO_Pin == FLOW_4_Pin)
	{
		Spray_driver::get_driver().gpio_callback(GPIO_Pin);
	}
}

// ADC callback
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{
	Spray_driver::get_driver().adc_callback(AdcHandle);
}

void isr_system_timer_tick()
{
	// Counters to keep track of the number of calls to this ISR.
	static uint8_t tick_count_10hz = 0;
	static uint8_t tick_count_1hz = 0;

	// NOTE - Don't need a counter for 100Hz, since the ISR fires at 100Hz already.

	run_sloppy_100hz = true;

	// Increment the counter for 10hz and check for counts required.
	if(++tick_count_10hz == 10) // Interval for 10hz is 10 times 100hz.
	{
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		run_sloppy_10hz = true;
		tick_count_10hz = 0;

		// Increment the counter for 10hz and check for counts required.
		if(++tick_count_1hz == 10) // Interval for 1hz is 10 times 10hz, placed here to reduce overhead.
		{
			run_sloppy_1hz = true;
			tick_count_1hz = 0;
		}
	}

	// All done.
	return;
}

uint64_t driverhost_get_monotonic_time_us(void)
{
	/*
	 *     the counter interrupts every 10000 microseconds, so we can extrapolate this and add where we currently are up to in the timer
	 */
	return (uint64_t) ((systick_counter * 10000)
	    + (uint64_t) __HAL_TIM_GET_COUNTER(&htim6));
}

