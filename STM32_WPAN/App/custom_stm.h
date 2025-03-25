/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_stm.h
  * @author  MCD Application Team
  * @brief   Header for custom_stm.c module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CUSTOM_STM_H
#define CUSTOM_STM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  /* AVIGA */
  CUSTOM_STM_DATA,
  CUSTOM_STM_BATTERY,
  CUSTOM_STM_SETTINGS,
  CUSTOM_STM_SYNC_TIMER,
  CUSTOM_STM_START_STOP,
} Custom_STM_Char_Opcode_t;

typedef enum
{
  /* DATA */
  CUSTOM_STM_DATA_READ_EVT,
  CUSTOM_STM_DATA_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_DATA_NOTIFY_DISABLED_EVT,
  /* BATTERY */
  CUSTOM_STM_BATTERY_READ_EVT,
  /* SETTINGS */
  CUSTOM_STM_SETTINGS_READ_EVT,
  CUSTOM_STM_SETTINGS_WRITE_EVT,
  /* SYNC_TIMER */
  CUSTOM_STM_SYNC_TIMER_READ_EVT,
  CUSTOM_STM_SYNC_TIMER_WRITE_EVT,
  /* START_STOP */
  CUSTOM_STM_START_STOP_READ_EVT,
  CUSTOM_STM_START_STOP_WRITE_EVT,

  CUSTOM_STM_BOOT_REQUEST_EVT
} Custom_STM_Opcode_evt_t;

typedef struct
{
  uint8_t * pPayload;
  uint8_t   Length;
} Custom_STM_Data_t;

typedef struct
{
  Custom_STM_Opcode_evt_t       Custom_Evt_Opcode;
  Custom_STM_Data_t             DataTransfered;
  uint16_t                      ConnectionHandle;
  uint8_t                       ServiceInstance;
} Custom_STM_App_Notification_evt_t;

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
extern uint8_t SizeData;
extern uint8_t SizeBattery;
extern uint8_t SizeSettings;
extern uint8_t SizeSync_Timer;
extern uint8_t SizeStart_Stop;

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ------------------------------------------------------- */
void SVCCTL_InitCustomSvc(void);
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification);
tBleStatus Custom_STM_App_Update_Char(Custom_STM_Char_Opcode_t CharOpcode,  uint8_t *pPayload);
/* USER CODE BEGIN EF */

/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*CUSTOM_STM_H */
