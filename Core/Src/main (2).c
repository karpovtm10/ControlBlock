/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "queue.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "base64.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 550 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myLedTask */
osThreadId_t myLedTaskHandle;
const osThreadAttr_t myLedTask_attributes = {
  .name = "myLedTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myGsmTask */
osThreadId_t myGsmTaskHandle;
const osThreadAttr_t myGsmTask_attributes = {
  .name = "myGsmTask",
  .stack_size = 550 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myGpsTask */
osThreadId_t myGpsTaskHandle;
const osThreadAttr_t myGpsTask_attributes = {
  .name = "myGpsTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mySysTickTask */
osThreadId_t mySysTickTaskHandle;
const osThreadAttr_t mySysTickTask_attributes = {
  .name = "mySysTickTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for myMobileApplica */
osThreadId_t myMobileApplicaHandle;
const osThreadAttr_t myMobileApplica_attributes = {
  .name = "myMobileApplica",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mySendingServer */
osThreadId_t mySendingServerHandle;
const osThreadAttr_t mySendingServer_attributes = {
  .name = "mySendingServer",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myControl */
osThreadId_t myControlHandle;
const osThreadAttr_t myControl_attributes = {
  .name = "myControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myUsartTask */
osThreadId_t myUsartTaskHandle;
const osThreadAttr_t myUsartTask_attributes = {
  .name = "myUsartTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for SendQueue */
osMessageQueueId_t SendQueueHandle;
const osMessageQueueAttr_t SendQueue_attributes = {
  .name = "SendQueue"
};
/* Definitions for UsartQueue */
osMessageQueueId_t UsartQueueHandle;
const osMessageQueueAttr_t UsartQueue_attributes = {
  .name = "UsartQueue"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
void StartDefaultTask(void *argument);
void StartLedTask(void *argument);
void StartGsmTask(void *argument);
void StartGpsTask(void *argument);
void StartmySysTickTask(void *argument);
void StartmyMobileApplicationTask(void *argument);
void StartmySendingServerDataTask(void *argument);
void StartmyControlTask(void *argument);
void StartUsartTask(void *argument);

/* USER CODE BEGIN PFP */
CAN_TxHeaderTypeDef TxHeader_1;
CAN_RxHeaderTypeDef RxHeader_1;
CAN_TxHeaderTypeDef TxHeader_2;
CAN_RxHeaderTypeDef RxHeader_2;

u8 				global_buf[PACK_LEN_dozerParams * 21];
char 			print_buffer					[PRINT_BUF];

const char char_map[10] =																		// Карта
{
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'
};

enum List_of_MODES_of_CONTROL				MODE_CONTROL = DISCONNECT_RC;

struct GPGGA_Struct         					// Структура с данными GPS-ГЛОНАСС-модуля
{
	char _time [15];           					// Время
	char Latitude_int [10];     				// Широта целая часть
	char Latitude_float [12];   				// Широта десятичная часть
	char NS;                   					// Север-Юг
	char Longitude_int[10];     				// Долгота целая часть
	char Longitude_float[12];   				// Долгота десятичная часть
	char EW;                   					// Запад-Восток
	char ReceiverMode;         					// Режим работы приемника
	char SatelliteNum [10];     				// Количество спутников в решении
	char Altitude_int [12];     				// Высота целая часть
	char Altitude_float [10];   				// Высота десятичная часть
	char alt_full[20];
};
struct GPGGA_Struct GPSFixData; 				// С этой структурой и будем работать

volatile i64 number1;
u8 negative_flag;
u8 ies;
u8 char_cnt;
u8 sizeofc = 0;

volatile u8 	GPS_DATA						[GPS_BUF];  		// Буффер на GPS данные
char										size_of_RTK_parcel_arr			[4];
char										RECEIVE_SERVER_BUF				[SERVER_BUF];
 
 volatile time_t							timeout_USART_wait;
u8 Items_num_Queue = 0;

 u32 check_ext_id = 0;
u32 def_ext_id = 0;
enum List_of_CAN_places place;

i64 										GPS_al  = 251156;									// Высота с GPS модуля после парсинга NMEA
double 										GPS_lat;											// Широта с GPS модуля после парсинга NMEA
double 										GPS_lon;											// Долгота с GPS модуля после парсинга NMEA

double 										coord_GPS1_latitude = 0;							// �?тоговая широта первого ГНСС модема
double 										coord_GPS2_latitude = 0;							// �?тоговая широта второго ГНСС модема
double 										coord_TARGET_latitude = 0;							// �?тоговая широта цели					
double 										coord_GPS1_longitude = 0;							// �?тоговая долгота первого ГНСС модема	
double 										coord_GPS2_longitude = 0;							// �?тоговая долгота второго ГНСС модема	
double 										coord_TARGET_longitude = 0;							// �?тоговая долгота цели
u64 										u64_lat1 = 0;										// uint64 переменная для широты от второго ГНСС модема					
double	 									lat1_multiplier = 0;								// множитель широты
u64 										u64_lon1 = 0;										// uint64 переменная для долготы от второго ГНСС модема
double	 									lon1_multiplier = 0;								// множитель долготы

float 										HEADING = 0;										// Азимут
float 										ANGLE_btw_TARGET_N_DOZER = 0;						// Угол между машиной и целью
float 										DELTA_ANGLE_btw_CURSE_N_HEADING = 0;				// Разниаца между Азимутом и углом к цели
float										SPEED_ENGINE = 0;

double 										fi;
double 										pi = 3.14159265359;

double 										shirota_lau_rad;       // Пересчет координат цели в радианы
double 										dolgota_lau_rad;
double 										shirota_tar_rad;
double 										dolgota_tar_rad;

double myLat = 55.216422;
double myLon = 61.441115;

u32 GPS_count = 0;
u32 									   	k = 0;                        						// Переменные для счетчиков

u8 											n_lat_float = 0; 									// Количество цифр после запятой в широте с GPS приемника(парсинг NMEA)
u8											n_lon_float = 0; 									// Количество цифр после запятой в долготе с GPS приемника(парсинг NMEA)
u8											n_alt_float = 0;									// Количество цифр после запятой в высоте с GPS приемника(парсинг NMEA)

i32 	lat_int;
i32 	lon_int;
u32 	lat_float;
u32 	lon_float;

char str1[60]={0};
typedef struct USART_prop{

  uint8_t usart_buf[60];

  uint8_t usart_cnt;

  uint8_t is_tcp_connect;//статус попытки создать соединение TCP с сервером

  uint8_t is_text;//статус попытки передать текст серверу

} USART_prop_ptr;

USART_prop_ptr usartprop;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
u8 crc8(u8 *pcBlock, u8 len)    
{
	u8 crc = 0x00;
	u8 i,j;
	u8 b;

	for(j = 0; j < len; j++)
	{
	   b = *pcBlock++;

		   i = 8;
		   do
			   {
				   if((b ^ crc) & 0x01)
					   {
						   crc =((crc ^ 0x18) >> 1) | 0x80;
					   }
				   else
					   {
						   crc >>= 1;
					   }
				   b >>= 1;
			   }
		   while(--i);
	}                
	return crc;
}

i64 char_to_int (char *cti)													// Функция перевода из строки в число
{
	negative_flag = 0;
	ies = 0;
	number1 = 0;
	char_cnt = 0;

	while (cti[char_cnt] != 0)
	{
		for (ies = 0; ies < 10; ies++)
		{
			if (cti[0] == '-') negative_flag = 1;
			if (cti[char_cnt] == char_map[ies]) 
			{
				number1 = number1 * 10 + ies ;
				break;
			}
		}
		char_cnt++;
	}
	
	if (negative_flag) number1 *= -1;
	
	return number1;
}

void mask_parse (char *result, volatile char *_TEMP, char *left_mask, char *right_mask) 	// *GSM_TEMP = наша команда
{	
	u8 i = 0, j = 0, del = 0;								                            	// *left_mask = символы перед нужной нам частью строки
	char *startstring; 											            	// *right_mask = символы после нужной нам части строки
	char *endstring;
	u8 sofleft = 0;
	u16 sofstart = 0;
	u16 sofend = 0;
	sofleft = strlen(left_mask);
    startstring = strstr((const char *)_TEMP, left_mask); // ищем left_mask
    startstring = startstring + sofleft; 

	endstring = strstr(startstring, right_mask);
	sofstart = strlen(startstring);
	sofend = strlen(right_mask);
	for (; i < sofstart; i++)
	{
		if (startstring[i] == endstring[j])
		{			
			if (j < sofend) j++;
			else 
			{
				for(del = i; del >= i - j; del--)
				{
					result[del] = 0;
				}
				break;
			}
		}
		
		else 
		{
			j = 0;
			if (startstring[i] == endstring[j]) j++;
			
		}
		result[i] = startstring[i];
	}
}

void clear_RXBuffer(u8 *RX_BUFER, u16 size)           				// Функция очистки буфера
{
    u16 i = 0;
    for (i = 0; i < size; i++) RX_BUFER[i] = 0;
}

void string_parse(char* buf_str)
{

  HAL_UART_Transmit(&huart6,(uint8_t*)buf_str,strlen(buf_str),0x1000);

}

void HAL_UARTExGSM_ReceiveToIdle_DMA(void)
{
//	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (u8 *)GSM_DATA, GSM_BUF);
//	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

//-----------------------------------------------

//-----------------------------------------------

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart6)
	{
		CDC_Transmit_FS(str1, 1);
		HAL_UART_Receive_IT(&huart6,(uint8_t*)str1,1);	
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	u8 CAN1_DATA[8];
	u8 CAN2_DATA[8];
	
	QUEUE_t msg_CAN1;
	QUEUE_t msg_CAN2;
	
	BaseType_t xHigherPriorityTaskWoken;
	
	//************************** CAN1 *********************************************
    if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader_1, CAN1_DATA) == HAL_OK)
    {
		msg_CAN1.data[0] = '$';
		msg_CAN1.data[1] = PACK_ID_dozerParams;
		
        for(u8 i = 0; i < 8; i++)
		{
			msg_CAN1.data[i + 4] = CAN1_DATA[i];
		}
	
		if (RxHeader_1.StdId == CAN_LAT1_EXT_ID)
		{

			coord_GPS2_latitude = 0;
			u64_lat1 = 0;
			lat1_multiplier = 100000000;
			
			for(u8 i = 0; i <= 4; i++)
			{
				u64_lat1 = u64_lat1 + ((u64)CAN1_DATA[i] << (i * 8));
			}
						
			coord_GPS2_latitude = (double)u64_lat1 / lat1_multiplier - 90;			
		}
		
		if (RxHeader_1.StdId == CAN_LON1_EXT_ID)
		{	
	
			coord_GPS2_longitude = 0;
			u64_lon1 = 0;
			lon1_multiplier = 100000000;
			
			for(u8 i = 0; i <= 4; i++)
			{
				u64_lon1 = u64_lon1 + ((u64)CAN1_DATA[i] << (i * 8));
			}
			
			coord_GPS2_longitude = (double)u64_lon1 / lon1_multiplier - 180;	
		}
		
		switch(RxHeader_1.ExtId)
		{
			case CAN_Tx_DM1_Angles:
										msg_CAN1.data[2] = 0x11;
										msg_CAN1.data[3] = 0x05;
										msg_CAN1.data[12] = crc8(&msg_CAN1.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg_CAN1.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_DM1_Heights:
										msg_CAN1.data[2] = 0x12;
										msg_CAN1.data[3] = 0x05;
										msg_CAN1.data[12] = crc8(&msg_CAN1.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg_CAN1.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_201_ID:
										msg_CAN1.data[2] = 0x01;
										msg_CAN1.data[3] = 0x02;
										msg_CAN1.data[12] = crc8(&msg_CAN1.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg_CAN1.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_205_ID:
										msg_CAN1.data[2] = 0x05;
										msg_CAN1.data[3] = 0x02;
										msg_CAN1.data[12] = crc8(&msg_CAN1.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg_CAN1.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_207_ID:
										msg_CAN1.data[2] = 0x07;
										msg_CAN1.data[3] = 0x02;
										msg_CAN1.data[12] = crc8(&msg_CAN1.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg_CAN1.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_209_ID:
										msg_CAN1.data[2] = 0x09;
										msg_CAN1.data[3] = 0x02;
										msg_CAN1.data[12] = crc8(&msg_CAN1.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg_CAN1.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_210_ID:
										msg_CAN1.data[2] = 0x10;
										msg_CAN1.data[3] = 0x02;
										msg_CAN1.data[12] = crc8(&msg_CAN1.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg_CAN1.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_215_ID:
										msg_CAN1.data[2] = 0x15;
										msg_CAN1.data[3] = 0x02;
										msg_CAN1.data[12] = crc8(&msg_CAN1.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg_CAN1.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_216_ID:
										msg_CAN1.data[2] = 0x16;
										msg_CAN1.data[3] = 0x02;
										msg_CAN1.data[12] = crc8(&msg_CAN1.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg_CAN1.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_221_ID:
										msg_CAN1.data[2] = 0x21;
										msg_CAN1.data[3] = 0x02;
										msg_CAN1.data[12] = crc8(&msg_CAN1.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg_CAN1.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_2BD_ID:
										msg_CAN1.data[2] = 0xBD;
										msg_CAN1.data[3] = 0x02;
										msg_CAN1.data[12] = crc8(&msg_CAN1.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg_CAN1.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_303_ID:
										msg_CAN1.data[2] = 0x03;
										msg_CAN1.data[3] = 0x03;
										msg_CAN1.data[12] = crc8(&msg_CAN1.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg_CAN1.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_308_ID:
										msg_CAN1.data[2] = 0x08;
										msg_CAN1.data[3] = 0x03;
										msg_CAN1.data[12] = crc8(&msg_CAN1.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg_CAN1.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_311_ID:
										msg_CAN1.data[2] = 0x11;
										msg_CAN1.data[3] = 0x03;
										msg_CAN1.data[12] = crc8(&msg_CAN1.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg_CAN1.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_313_ID:
										msg_CAN1.data[2] = 0x13;
										msg_CAN1.data[3] = 0x03;
										msg_CAN1.data[12] = crc8(&msg_CAN1.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg_CAN1.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_314_ID:
										msg_CAN1.data[2] = 0x14;
										msg_CAN1.data[3] = 0x03;
										msg_CAN1.data[12] = crc8(&msg_CAN1.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg_CAN1.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_EE_F004_ID:
										msg_CAN1.data[2] = 0x04;
										msg_CAN1.data[3] = 0xF0;
										msg_CAN1.data[12] = crc8(&msg_CAN1.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg_CAN1.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_EE_FEEE_ID:
										msg_CAN1.data[2] = 0xEE;
										msg_CAN1.data[3] = 0xFE;
										msg_CAN1.data[12] = crc8(&msg_CAN1.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg_CAN1.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_EE_FEEF_ID:
										msg_CAN1.data[2] = 0xEF;
										msg_CAN1.data[3] = 0xFE;
										msg_CAN1.data[12] = crc8(&msg_CAN1.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg_CAN1.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_EE_FEFC_ID:
										msg_CAN1.data[2] = 0xFC;
										msg_CAN1.data[3] = 0xFE;
										msg_CAN1.data[12] = crc8(&msg_CAN1.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg_CAN1.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_EE_FFA0_ID:
										msg_CAN1.data[2] = 0xA0;
										msg_CAN1.data[3] = 0xFF;
										msg_CAN1.data[12] = crc8(&msg_CAN1.data[1], 11);
							
			xQueueSendFromISR(SendQueueHandle, &msg_CAN1.data, &xHigherPriorityTaskWoken);
			break;
		}
    }
	
	//************************** CAN2 *********************************************
    if(HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader_2, CAN2_DATA) == HAL_OK)
    {
		
	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
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
//  MX_CAN1_Init();
//  MX_CAN2_Init();
//  MX_SPI1_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	//	HAL_UARTExGSM_ReceiveToIdle_DMA();
	//	HAL_UARTEx_ReceiveToIdle_IT(&huart1, (u8 *)GSM_DATA, GSM_BUF);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of SendQueue */
  SendQueueHandle = osMessageQueueNew (50, sizeof(QUEUE_t), &SendQueue_attributes);

  /* creation of UsartQueue */
  UsartQueueHandle = osMessageQueueNew (200, sizeof(USART_q), &UsartQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myLedTask */
  myLedTaskHandle = osThreadNew(StartLedTask, NULL, &myLedTask_attributes);

  /* creation of myGsmTask */
  myGsmTaskHandle = osThreadNew(StartGsmTask, NULL, &myGsmTask_attributes);

  /* creation of myGpsTask */
  myGpsTaskHandle = osThreadNew(StartGpsTask, NULL, &myGpsTask_attributes);

  /* creation of mySysTickTask */
  mySysTickTaskHandle = osThreadNew(StartmySysTickTask, NULL, &mySysTickTask_attributes);

  /* creation of myMobileApplica */
  myMobileApplicaHandle = osThreadNew(StartmyMobileApplicationTask, NULL, &myMobileApplica_attributes);

  /* creation of mySendingServer */
  mySendingServerHandle = osThreadNew(StartmySendingServerDataTask, NULL, &mySendingServer_attributes);

  /* creation of myControl */
  myControlHandle = osThreadNew(StartmyControlTask, NULL, &myControl_attributes);

  /* creation of myUsartTask */
  myUsartTaskHandle = osThreadNew(StartUsartTask, NULL, &myUsartTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 10;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 10;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED2_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SIM_CHANNEL_GPIO_Port, SIM_CHANNEL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SPI1_CS_Pin|GSM_POWER_KEY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED0_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED2_Pin LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : SIM_CHANNEL_Pin */
  GPIO_InitStruct.Pin = SIM_CHANNEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SIM_CHANNEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CS_Pin GSM_POWER_KEY_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin|GSM_POWER_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED0_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void SENDING_COORDS(void)
{
	QUEUE_t msg;
	u64 mulLat = (myLat + 90) * pow(10, 8);
	u64 mulLon = (myLon + 180) * pow(10, 8);
	
	msg.data[0] = '$';
	msg.data[1] = PACK_ID_myCoords;
	for(u8 i = 0; i <= 4; i++)
	{
		msg.data[i + 2] = mulLat >> (8 * i) & 0xFF;
	}
	
	for(u8 i = 0; i <= 4; i++)
	{
		msg.data[i + 7] = mulLon >> (8 * i) & 0xFF;
	}
	
	msg.data[12] = (u16)roundf(HEADING) & 0xFF;
	msg.data[13] = (u16)roundf(HEADING) >> 8;
	msg.data[14] = char_to_int(&GPSFixData.ReceiverMode);
	msg.data[15] = char_to_int(GPSFixData.SatelliteNum);
	msg.data[16] = MODE_CONTROL;
	msg.data[17] = crc8(&msg.data[1], 16);
	xQueueSend(SendQueueHandle, &msg.data, osWaitForever);
}

void Parse_Nmea_Messages (void)													// Парсинг NMEA сообщений от GPS модуля
{

	u32    	j = 0; 
	u32    	g = 0;
	u32    	gh = 0;
	u8 		statComma = 0;
	u8 		float_flag = 0;
	u32		mul = 1;
	
	for (j = 0; j <= k; j++)
	{
		if (GPS_DATA[j] == 0x0A) 												// �?щем конец сообщения
		{

			statComma = 0;														// Обнуляем счетчик запятых
			while (GPSFixData.Latitude_float[n_lat_float] != 0 && GPSFixData.Latitude_float[n_lat_float] != 0x2E) 
			{
				n_lat_float++;	// Считаем количество знаков после запятой у широты
				mul *= 10;
			}
			while (GPSFixData.Longitude_float[n_lon_float] != 0 && GPSFixData.Longitude_float[n_lon_float] != 0x2E) n_lon_float++;	// Считаем количество знаков после запятой у долготы
			while (GPSFixData.Altitude_float[n_alt_float] != 0 && GPSFixData.Altitude_float[n_alt_float] != 0x2E) n_alt_float++;	// Считаем количество знаков после запятой у высоты
			lat_int =  char_to_int(GPSFixData.Latitude_int);					// Переводим в число целую часть широты
			lat_float = char_to_int(GPSFixData.Latitude_float);					// Переводим в число десятичную часть широты
			lon_int =  char_to_int(GPSFixData.Longitude_int);					// Переводим в число целую часть долготы
			lon_float = char_to_int(GPSFixData.Longitude_float);				// Переводим в число десятичну часть долготы
			GPS_al  = char_to_int(GPSFixData.alt_full);							// Получаем конечное INT значение высоты
			if (GPS_al < 0) GPS_al = 0;
			if (GPS_al > 9999999) GPS_al = 0;
			GPS_lat = (double)((u32)lat_int / 100) + (((double)((u32)lat_int % 100) + (double)lat_float / mul) / 60);			// Получаем конечное INT значение широты
			GPS_lon = (double)((u32)lon_int / 100) + (((double)((u32)lon_int % 100) + (double)lon_float / mul) / 60); 			// Получаем конечное INT значение долготы
		}
		
		if (GPS_DATA[j] != 0x2C)												// �?щим запятую
		{
			switch(statComma)													// По количеству запятых определяем нужный параметр в сообщении
			{
				case 1: GPSFixData._time[g++] = GPS_DATA[j]; break;				// Записывам время

				case 2:  														// Записываем широту
					  if (float_flag == 0) GPSFixData.Latitude_int[g++] = GPS_DATA[j]; 
					  else GPSFixData.Latitude_float[g++] = GPS_DATA[j];
					  
					  if (GPS_DATA[j] == 0x2E) { float_flag = 1; g = 0; }
				break;

				case 3: GPSFixData.NS = GPS_DATA[j]; break;						// Записываем Север/Юг

				case 4: 														// Записываем долготу
					  if (float_flag == 0) GPSFixData.Longitude_int[g++] = GPS_DATA[j]; 
					  else GPSFixData.Longitude_float[g++] = GPS_DATA[j];
					  
					  if (GPS_DATA[j] == 0x2E) { float_flag = 1; g = 0; }
				break;

				case 5: GPSFixData.EW = GPS_DATA[j]; break;						// Записываем Запад/Восток

				case 6: GPSFixData.ReceiverMode = GPS_DATA[j]; break;			// Записываем код качества сигнала

				case 7: GPSFixData.SatelliteNum[g++] = GPS_DATA[j]; break;		// Записываем количество видимых спутников

				case 9: 														// Записываем высоту
					GPSFixData.alt_full[gh++] = GPS_DATA[j];

					  if (GPS_DATA[j] == 0x2E) { float_flag = 1; g = 0; }
				break;
			}
		}
		else
		{
			statComma++;														// Счетчик запятых
			float_flag = 0;
			g = 0;
			gh = 0;
		}
	}			
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
	HAL_UART_Receive_IT(&huart6,(uint8_t*)str1,1);
	/* Infinite loop */
	for(;;)
	{

	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
* @brief Function implementing the myLedTask thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_StartLedTask */
void StartLedTask(void *argument)
{
  /* USER CODE BEGIN StartLedTask */
	/* Infinite loop */
	for(;;)
	{
		HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
		osDelay(500);
	}
  /* USER CODE END StartLedTask */
}

/* USER CODE BEGIN Header_StartGsmTask */
/**
* @brief Function implementing the myGsmTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGsmTask */
void StartGsmTask(void *argument)
{
  /* USER CODE BEGIN StartGsmTask */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
  /* USER CODE END StartGsmTask */
}

/* USER CODE BEGIN Header_StartGpsTask */
/**
* @brief Function implementing the myGpsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGpsTask */
void StartGpsTask(void *argument)
{
  /* USER CODE BEGIN StartGpsTask */
	/* Infinite loop */
	for(;;)
	{
		if (GPS_DATA[k-1] == 0x0A && GPS_DATA[k-2] == 0x0D) 
		{
			Parse_Nmea_Messages();
			coord_GPS1_latitude = GPS_lat;
			coord_GPS1_longitude = GPS_lon;
			GPS_count++;	
			k = 0;

			clear_RXBuffer((u8 *)GPSFixData.alt_full, sizeof(GPSFixData.alt_full));
		}
		coord_GPS1_latitude = myLat;
		coord_GPS1_longitude = myLon;
		osDelay(1);
	}
  /* USER CODE END StartGpsTask */
}

/* USER CODE BEGIN Header_StartmySysTickTask */
/**
* @brief Function implementing the mySysTickTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartmySysTickTask */
void StartmySysTickTask(void *argument)
{
  /* USER CODE BEGIN StartmySysTickTask */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
  /* USER CODE END StartmySysTickTask */
}

/* USER CODE BEGIN Header_StartmyMobileApplicationTask */
/**
* @brief Function implementing the myMobileApplica thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartmyMobileApplicationTask */
void StartmyMobileApplicationTask(void *argument)
{
  /* USER CODE BEGIN StartmyMobileApplicationTask */

	/* Infinite loop */
	for(;;)
	{
		SENDING_COORDS();
		osDelay(1000);
	}
  /* USER CODE END StartmyMobileApplicationTask */
}

/* USER CODE BEGIN Header_StartmySendingServerDataTask */
/**
* @brief Function implementing the mySendingServer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartmySendingServerDataTask */
void StartmySendingServerDataTask(void *argument)
{
  /* USER CODE BEGIN StartmySendingServerDataTask */
	QUEUE_t msg;
	time_t can_send_timer;
	clear_RXBuffer(global_buf, PACK_LEN_dozerParams * 21);
	/* Infinite loop */
	for(;;)
	{
		Items_num_Queue = uxQueueMessagesWaiting(SendQueueHandle);
		if (xQueueReceive(SendQueueHandle, &msg, 0) == pdTRUE)
		{
			if (msg.data[1] == PACK_ID_dozerParams)
			{
				check_ext_id = 0x18000003 | (msg.data[2] << 8) | (msg.data[3] << 16);
				
				switch(check_ext_id)
				{
					case CAN_Tx_Modem_201_ID:
						place = CAN_Tx_Modem_201_place;
					break;
					
					case CAN_Tx_Modem_205_ID:
						place = CAN_Tx_Modem_205_place;
					break;
					
					case CAN_Tx_Modem_207_ID:
						place = CAN_Tx_Modem_207_place;
					break;
				
					case CAN_Tx_Modem_209_ID:
						place = CAN_Tx_Modem_209_place;
					break;
					
					case CAN_Tx_Modem_210_ID:
						place = CAN_Tx_Modem_210_place;
					break;
					
					case CAN_Tx_Modem_215_ID:
						place = CAN_Tx_Modem_215_place;
					break;
					
					case CAN_Tx_Modem_216_ID:
						place = CAN_Tx_Modem_216_place;
					break;
					
					case CAN_Tx_Modem_221_ID:
						place = CAN_Tx_Modem_221_place;
					break;
					
					case CAN_Tx_Modem_2BD_ID:
						place = CAN_Tx_Modem_2BD_place;
					break;
					
					case CAN_Tx_Modem_303_ID:
						place = CAN_Tx_Modem_303_place;
					break;
					
					case CAN_Tx_Modem_308_ID:
						place = CAN_Tx_Modem_308_place;
					break;
					
					case CAN_Tx_Modem_311_ID:
						place = CAN_Tx_Modem_311_place;
					break;
					
					case CAN_Tx_Modem_313_ID:
						place = CAN_Tx_Modem_313_place;
					break;
					
					case CAN_Tx_Modem_314_ID:
						place = CAN_Tx_Modem_314_place;
					break;
					
					case CAN_Tx_DM1_Angles:
						place = CAN_Tx_DM1_Angles_place;
					break;
					
					case CAN_Tx_DM1_Heights:
						place = CAN_Tx_DM1_Heights_place;
					break;
						
					default: check_ext_id = (msg.data[2]) | (msg.data[3] << 8);
				}
				
				switch(check_ext_id)
				{
					case CAN_EE_F004_ID:
						place = CAN_EE_F004_place;
					break;
					
					case CAN_EE_FEEE_ID:
						place = CAN_EE_FEEE_place;
					break;
					
					case CAN_EE_FEEF_ID:
						place = CAN_EE_FEEF_place;
					break;
					
					case CAN_EE_FEFC_ID:
						place = CAN_EE_FEFC_place;
					break;
					
					case CAN_EE_FFA0_ID:
						place = CAN_EE_FFA0_place;
					break;
				}
				
				for(u8 i = 0; i < PACK_LEN_dozerParams; i++)
				{
					global_buf[PACK_LEN_dozerParams * place + i] = msg.data[i];
				}
				
				if (xTaskGetTickCount() - can_send_timer > 200)
				{
					can_send_timer = xTaskGetTickCount();
					HAL_UART_Transmit(&huart3, global_buf, PACK_LEN_dozerParams * 21, 0xFFFF);
				}
			}
			
			if (msg.data[1] == PACK_ID_myCoords)
			HAL_UART_Transmit(&huart3, msg.data, PACK_LEN_myCoords, 0xFFFF);
			
			if (msg.data[1] == PACK_ID_Confirm || msg.data[1] == PACK_ID_Err)
			HAL_UART_Transmit(&huart3, msg.data, PACK_LEN_Confirm, 0xFFFF);
			
			if (msg.data[1] == PACK_ID_CompletePoint)
			HAL_UART_Transmit(&huart3, msg.data, PACK_LEN_CompletePoint, 0xFFFF);
			
		}
		else
		{	
			osDelay(1);
		}
	}
  /* USER CODE END StartmySendingServerDataTask */
}

/* USER CODE BEGIN Header_StartmyControlTask */
/**
* @brief Function implementing the myControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartmyControlTask */
void StartmyControlTask(void *argument)
{
  /* USER CODE BEGIN StartmyControlTask */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
  /* USER CODE END StartmyControlTask */
}

/* USER CODE BEGIN Header_StartUsartTask */
/**
* @brief Function implementing the myUsartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsartTask */
void StartUsartTask(void *argument)
{
  /* USER CODE BEGIN StartUsartTask */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
  /* USER CODE END StartUsartTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{

	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
