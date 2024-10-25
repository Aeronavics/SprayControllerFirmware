/*
 * SprayController.hpp
 *
 *  Created on: Oct 18, 2024
 *      Author: jmorritt
 */

#ifndef INC_SPRAYCONTROLLER_HPP_
#define INC_SPRAYCONTROLLER_HPP_

//include all our modules
#include "mavlink.hpp"
// Include driver modules for peripherals.
#include "driver_module.hpp"
#include "libcanard_module.hpp"
#include "params.hpp"
#include "uart_module.hpp"

#include "spray_driver.hpp"
#include "generator_driver.hpp"

#include <uavcan.protocol.file.BeginFirmwareUpdate_req.h>

#include "main.h"
#include "config.h"

#include "adc.h"
#include "fdcan.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "dma.h"

#include "stm32g4xx_hal.h"
#include "system_stm32g4xx.h"
#include "stm32g4xx_hal_conf.h"
#include "stm32g4xx_it.h"

//details shown in CAN Node info
#define UAVCAN_MODULE_NAME "com.aeronavics.sprayctrl"

#endif /* INC_SPRAYCONTROLLER_HPP_ */
