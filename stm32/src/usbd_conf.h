/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_conf.h
  * @version        : v2.0_Cube
  * @brief          : Header for usbd_conf.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CONF__H__
#define __USBD_CONF__H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"

#define USBD_MAX_NUM_INTERFACES 1
#define USBD_MAX_NUM_CONFIGURATION 1
#define USBD_MAX_STR_DESC_SIZ 512
#define USBD_DEBUG_LEVEL 0
#define USBD_SELF_POWERED 1
#define HID_FS_BINTERVAL 0xA

/****************************************/
/* #define for FS and HS identification */
#define DEVICE_FS 0

/* Memory management macros */

/** Alias for memory allocation. */
#define USBD_malloc (uint32_t *)USBD_static_malloc

/** Alias for memory release. */
#define USBD_free USBD_static_free

/** Alias for memory set. */
#define USBD_memset /* Not used */

/** Alias for memory copy. */
#define USBD_memcpy /* Not used */

/** Alias for delay. */
#define USBD_Delay HAL_Delay

/* For footprint reasons and since only one allocation is handled in the HID class
   driver, the malloc/free is changed into a static allocation method */
void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *p);

#endif /* __USBD_CONF__H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
