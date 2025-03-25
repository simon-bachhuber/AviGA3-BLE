/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ble_custom.h"

#include "meas.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* AVIGA */
  uint8_t               Data_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[247];
uint8_t NotifyCharData[247];

/* USER CODE BEGIN PV */

//Sensor
extern imu_sensor sensor;

//BLE-Buffer
extern uint8_t ble_buffer[];
extern uint8_t ble_flag;

//BATTERY
extern uint8_t battery_level;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* AVIGA */
static void Custom_Data_Update_Char(void);
static void Custom_Data_Send_Notification(void);

/* USER CODE BEGIN PFP */

//BLE: Update BLE characteristics with custom data
void ble_updateChar(uint8_t char_identifier)
{


	//Check for idetifier to update
	if(char_identifier == BLE_CHAR_DATA)
	{
		if(ble_flag == ON)
		{
			Custom_STM_App_Update_Char(CUSTOM_STM_DATA, ble_buffer);

			ble_flag = OFF;
		}
	}

	//Check for idetifier to update
		if(char_identifier == BLE_CHAR_BATTERY)
		{
			if(ble_flag == ON)
			{

				Custom_STM_App_Update_Char(CUSTOM_STM_BATTERY, &battery_level);


				ble_flag = OFF;
			}
		}
}

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* AVIGA */
    case CUSTOM_STM_DATA_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DATA_READ_EVT */

      /* USER CODE END CUSTOM_STM_DATA_READ_EVT */
      break;

    case CUSTOM_STM_DATA_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DATA_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_DATA_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_DATA_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DATA_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_DATA_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_BATTERY_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_BATTERY_READ_EVT */

      /* USER CODE END CUSTOM_STM_BATTERY_READ_EVT */
      break;

    case CUSTOM_STM_SETTINGS_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SETTINGS_READ_EVT */

      /* USER CODE END CUSTOM_STM_SETTINGS_READ_EVT */
      break;

    case CUSTOM_STM_SETTINGS_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SETTINGS_WRITE_EVT */

      /* USER CODE END CUSTOM_STM_SETTINGS_WRITE_EVT */
      break;

    case CUSTOM_STM_SYNC_TIMER_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SYNC_TIMER_READ_EVT */

      /* USER CODE END CUSTOM_STM_SYNC_TIMER_READ_EVT */
      break;

    case CUSTOM_STM_SYNC_TIMER_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SYNC_TIMER_WRITE_EVT */

      /* USER CODE END CUSTOM_STM_SYNC_TIMER_WRITE_EVT */
      break;

    case CUSTOM_STM_START_STOP_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_START_STOP_READ_EVT */

      /* USER CODE END CUSTOM_STM_START_STOP_READ_EVT */
      break;

    case CUSTOM_STM_START_STOP_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_START_STOP_WRITE_EVT */

      /* USER CODE END CUSTOM_STM_START_STOP_WRITE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */

  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* AVIGA */
void Custom_Data_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Data_UC_1*/

  /* USER CODE END Data_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_DATA, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Data_UC_Last*/

  /* USER CODE END Data_UC_Last*/
  return;
}

void Custom_Data_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Data_NS_1*/

  /* USER CODE END Data_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_DATA, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Data_NS_Last*/

  /* USER CODE END Data_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
