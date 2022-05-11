/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOE
#define LED4_Pin GPIO_PIN_4
#define LED4_GPIO_Port GPIOE
#define SIM_CHANNEL_Pin GPIO_PIN_4
#define SIM_CHANNEL_GPIO_Port GPIOA
#define CS_PIN_Pin GPIO_PIN_4
#define CS_PIN_GPIO_Port GPIOC
#define POWER_KEY_Pin GPIO_PIN_8
#define POWER_KEY_GPIO_Port GPIOC
#define LED0_Pin GPIO_PIN_3
#define LED0_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

#define GIT										0
#define MAIL_SIZE 								(u32) 50					// Размер очереди
#define DEBUG 									0							// Отладка
#if DEBUG
#define SCHEDULER_STATUS						1							// Статус планировщика. 0 - диспетчер задач, 1 - время работы задач
#endif
#define	RoutePoints_COUNT						10							// Количество точек для сохранения маршрута
#define	SaveZonePoints_COUNT					10							// Количество точек для сохранения зоны безопасности

#define CAN_EXT_ID_MASK							0x18000003
#define CAN_STD_ID								0x000						// Стандартный идентификатор
#define CAN_EXT_ID								0x00000000					// Расширенный идентификатор
#define CAN_LAT_ID								0x174
#define CAN_LON_ID								0x175
#define CAN_ALT_ROVER_ID						0x176
#define CAN_ALT_ID								0x18051003					// пдентификатор высоты для Контроллера ГСТ// �?дентификатор высоты для Контроллера ГСТ

#define CAN_LAT1_EXT_ID							0x174						// �?дентификатор широты от второго ГНСС модема
#define CAN_LON1_EXT_ID							0x175						// �?дентификатор долготы от второго ГНСС модема
#define CAN_Remote_Control_A_ID					0x195						// Посылка от джойстика движения
#define CAN_Remote_Control_B_ID					0x196						// Посылка от джойстика навесного оборудования
#define CAN_Remote_Control_C_ID					0x198						// Посылка с оборотами ДВС
#define CAN_Remote_Control_sButton_ID			0x18030003

#define RPM_ENGINE								750							// Обороты 750...2200

#define COM1_GPS_BUF									1000
#define PRINT_BUF								255
#define STATUS_BUF								255
#define GPS_BUF									100
#define	SERVER_BUF								500
#define NTRIP_BUF								1024
#define	CAN_BUF									8
#define RADIO_BUF								255
#define COMMAND_BUF								13


#define DEG_TO_RAD 								0.01745329252
#define RAD_TO_DEG								57.29577951308

#define FULL_DRIVE								100
#define STOP_DRIVE								125
#define SLOW_DRIVE								50

#define	FULL_DRIVE_CRC							255 - FULL_DRIVE
#define STOP_DRIVE_CRC							255 - STOP_DRIVE
#define SLOW_DRIVE_CRC							255 - SLOW_DRIVE

#define NTRIP_SERVER_CON_ID						0
#define REMOTE_SERVER_CON_ID					1

#define PACK_ID_tNum							0x01
#define PACK_ID_Coords							0x02
#define PACK_ID_Mode							0x05
#define PACK_ID_RC								0x06
#define PACK_ID_ALIVE							0xA1
#define PACK_ID_Err								0xFF
#define PACK_ID_Confirm							0xFE
#define PACK_ID_Request							0x07

#define PACK_ID_myCoords						0x12
#define PACK_ID_dozerParams						0xA0
#define PACK_ID_CompletePoint					0x13

#define PACK_LEN_tNum							4
#define PACK_LEN_Coords							14
#define PACK_LEN_Mode							4
#define PACK_LEN_RC								12
#define PACK_LEN_Err							5
#define PACK_LEN_Confirm						6
#define PACK_LEN_myCoords						18
#define PACK_LEN_dozerParams					13
#define PACK_LEN_CompletePoint					4
#define PACK_LEN_ALIVE							5
#define PACK_LEN_Request						4

#define PARCEL_ID_tNumSave						0x01
#define PARCEL_ID_tNumCoord						0x02
#define PARCEL_ID_SaveCoord						0x01
#define PARCEL_ID_PointCoord					0x02
#define PARCEL_ID_Mode							0x01
#define PARCEL_ID_RC							0x00
#define PARCEL_ID_Err							0xFF
#define PARCEL_ID_myCoord						0x01
#define PARCEL_ID_Request_Coords				0x12
#define PARCEL_ID_Request_CAN					0xA0

#define CAN_Tx_Modem_201_ID						0x0201
#define CAN_Tx_Modem_205_ID						0x0205
#define CAN_Tx_Modem_207_ID						0x0207
#define CAN_Tx_Modem_209_ID						0x0209
#define CAN_Tx_Modem_210_ID						0x0210
#define CAN_Tx_Modem_215_ID						0x0215
#define CAN_Tx_Modem_216_ID						0x0216
#define CAN_Tx_Modem_221_ID						0x0221
#define CAN_Tx_Modem_510_ID						0x0510
#define CAN_Tx_Modem_506_ID						0x0506
#define CAN_Tx_Modem_507_ID						0x0507

#define CAN_Tx_Modem_303_ID						0x0303
#define CAN_Tx_Modem_308_ID						0x0308
//#define CAN_Tx_Modem_311_ID						0x0311
#define CAN_Tx_Modem_313_ID						0x0313
#define CAN_Tx_Modem_314_ID						0x0314

#define CAN_Tx_Modem_350_ID						0x0350
#define CAN_Tx_Modem_353_ID						0x0353
#define CAN_Tx_Modem_354_ID						0x0354
#define CAN_Tx_Modem_355_ID						0x0355

#define CAN_EE_F004_ID							0xF004
#define CAN_EE_FEEE_ID							0xFEEE
#define CAN_EE_FEEF_ID							0xFEEF
//#define CAN_EE_FEFC_ID						0xFEFC
#define CAN_EE_FFA0_ID							0xFFA0

#define CAN_Tx_DM1_ALT							0x0510
#define CAN_Tx_DM1_Angles						0x0511
#define CAN_Tx_DM1_Heights						0x0512

enum List_of_Points_Type
{
	SAVE_ZONE = 1,				
	ROUTE									
};

enum List_of_MOVING
{
	NO_MOVEMENT,         					
	LEFT_DRIVE,          					
	RIGHT_DRIVE,         					
	FORWARD_DRIVE,       					
	BACKWARD_DRIVE      					
};

enum List_of_MODES_of_CONTROL
{
	IDLE_MODE,
	MANUAL_CONTROL_ON,
	MANUAL_CONTROL_OFF,
	START_AUTO,
	PAUSE_AUTO,
	RESUME_AUTO,
	STOP_AUTO,
	DISCONNECT_RC = 0xFF
};

enum List_of_CAN_places
{
	CAN_Tx_Modem_201_place,					
	CAN_Tx_Modem_205_place,					
	CAN_Tx_Modem_207_place,					
	CAN_Tx_Modem_209_place,					
	CAN_Tx_Modem_210_place,					
	CAN_Tx_Modem_215_place,				
	CAN_Tx_Modem_216_place,					
	CAN_Tx_Modem_221_place,					
	CAN_Tx_Modem_510_place,					
	CAN_Tx_Modem_303_place,					
	CAN_Tx_Modem_308_place,					
	//CAN_Tx_Modem_311_place,					
	CAN_Tx_Modem_313_place,					
	CAN_Tx_Modem_314_place,	

	

	CAN_EE_F004_place,						
	CAN_EE_FEEE_place,						
	CAN_EE_FEEF_place,						
	//CAN_EE_FEFC_place,						
	CAN_EE_FFA0_place,						

	CAN_Tx_DM1_Angles_place,				
	CAN_Tx_DM1_Heights_place,

//	CAN_Tx_Modem_506_place,
//	CAN_Tx_Modem_507_place
};



typedef int64_t time_t;
typedef int64_t i64;
typedef uint8_t u8;
typedef int32_t i32;
typedef int16_t i16;
typedef int8_t i8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef struct          						// Структура с сервисными данными
{
	u8 data [18];           					// Данные
} QUEUE_t;

typedef struct          						// Структура с сервисными данными
{
	u8 data [1];           						// Данные
} USART_q;



// ********************************************************************************************************************************************
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
