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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "base64.h"
#include "usbd_cdc_if.h"
#include "math.h"
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_RADIO_HANDLE &huart1
#define UART_COM1_GPS_HANDLE &huart6
#define UART_COM2_GPS_HANDLE &huart2
#define UART_DEBUG_HANDLE &huart3
/******************************************************************************/
/******************************************************************************/
#define CONTROL_BLOCK
//#define ROVER_BLOCK
/******************************************************************************/
/******************************************************************************/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;															// CAN1 handle Structure definition
CAN_HandleTypeDef hcan2;															// CAN2 handle Structure definition

CAN_FilterTypeDef  sFilterConfig;													// CAN Filters handle Structure definition

SPI_HandleTypeDef hspi1;															// SPI1 handle Structure definition

UART_HandleTypeDef huart1;															// UART1 handle Structure definition
UART_HandleTypeDef huart2;															// UART2 handle Structure definition
UART_HandleTypeDef huart3;															// UART3 handle Structure definition
UART_HandleTypeDef huart4;															// UART4 handle Structure definition
UART_HandleTypeDef huart6;															// UART6 handle Structure definition
DMA_HandleTypeDef hdma_usart1_rx;													// DMA UART1 handle Structure definition
DMA_HandleTypeDef hdma_usart2_rx;													// DMA UART2 handle Structure definition
DMA_HandleTypeDef hdma_usart6_rx;													// DMA UART6 handle Structure definition

																					/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
	.name = "defaultTask",
	.stack_size = 550 * 4,
	.priority = (osPriority_t) osPriorityHigh,
};
																					/* Definitions for myLedTask */
osThreadId_t myLedTaskHandle;
const osThreadAttr_t myLedTask_attributes = {
	.name = "myLedTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal,
};
																					/* Definitions for myGsmTask */
osThreadId_t myRadioTaskHandle;
const osThreadAttr_t myRadioTask_attributes = {
	.name = "myRadioTask",
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
CAN_TxHeaderTypeDef CAN1_TxHeader;													// CAN-1 Tx message header structure definition
CAN_RxHeaderTypeDef CAN1_RxHeader;													// CAN-1 Rx message header structure definition
uint8_t CAN1_TxData[8] = {0,};														// Буффер для отправки сообщений в CAN-1
uint8_t CAN1_RxData[8] = {0,};														// Буффер для приема сообщений в CAN-1
uint32_t CAN1_TxMailbox = 0;														// Хранилище для посылок CAN-1	

CAN_TxHeaderTypeDef CAN2_TxHeader;													// CAN-2 Tx message header structure definition
CAN_RxHeaderTypeDef CAN2_RxHeader;													// CAN-2 Rx message header structure definition
uint8_t CAN2_TxData[8] = {0,};														// Буффер для отправки сообщений в CAN-2
uint8_t CAN2_RxData[8] = {0,};														// Буффер для приема сообщений в CAN-2
uint32_t CAN2_TxMailbox = 0;														// Хранилище для посылок CAN-2

struct GPGGA_Struct         														// Структура с данными ГНСС модуля
{
	char _time [15];           														// Время
	char Latitude_int [10];     													// Широта целая часть
	char Latitude_float [12];   													// Широта десятичная часть
	char NS;                   														// Север-Юг
	char Longitude_int[10];     													// Долгота целая часть
	char Longitude_float[12];   													// Долгота десятичная часть
	char EW;                   														// Запад-Восток
	char ReceiverMode;         														// Режим работы приемника Блока управления
	u8 ReceiverMode_2;																// Режим работы Ровера	
	char SatelliteNum [10];     													// Количество спутников в решении
	u8 SatelliteNum_2;     															// Количество спутников в решении
	char Altitude_int [12];     													// Высота целая часть
	char Altitude_float [10];   													// Высота десятичная часть
	char alt_full[20];
};
struct GPGGA_Struct GPSFixData; 													// Объявление структуры с данными ГНСС модуля

struct Coords																		// Структура координат
{
	double latitude;																// Широта
	double longitude;																// Долгота
};
struct Coords coordSaveZone[SaveZonePoints_COUNT];									// Массив точек зоны безопасности
struct Coords coordRoute[RoutePoints_COUNT];										// Массив точек маршрута

u8 global_buf[PACK_LEN_dozerParams * 19];											// Глобальный буфер параметров для передачи приложению
u8 Items_num_Queue = 0;																// Переменная для отслеживания количества данных в очереди
	
enum List_of_CAN_places place;														// Объявление перечисления CAN посылок

time_t can_send_timer;																// Переменная для контроля задержки передачи данных на сервер							
u8 isCanSendingAllowed = 0;															// Флаг принятой посылки для формирования глобального буфера
char			RADIO_DATA1						[30];
extern u8 										USB_Buf[500];						// Буффер для храниния данных (USB), полученных через виртуальнй СОМ порт
extern u8 										USB_flag;							// Флаг принятых данных
extern u32 										USB_len;							// Длинна посылки с (USB)
u8												virtual_gps;						// Флаг тоннеля USB <--> GPS
u8												virtual_bt;							// Флаг тоннеля USB <--> Bluetooth
u8												virtual_surv = 0;
extern time_t 									USB_timer;							// Таймер для определения конца посылки через USB
extern u16 										USB_cnt;							// Счетчик заполнения USB буффера
u8 												USB_Cplt = 0;						// Флаг завершения передачи по USB

volatile char	COM1_GPS_DATA					[COM1_GPS_BUF];						// Буффер данных для COM1 порта ГНСС модуля
char			COM2_GPS_DATA					[GPS_BUF];							// Буффер данных для COM2 порта ГНСС модуля
char			RADIO_DATA						[RADIO_BUF];						// Буффер данных для Радио

char			RECEIVE_SERVER_BUF				[SERVER_BUF];

u8				CAN_BASKET_LAT					[CAN_BUF];							// Корзина для широты в CAN
u8				CAN_BASKET_LON					[CAN_BUF];							// Корзина для долготы в CAN
u8				CAN_BASKET_ALT					[CAN_BUF];							// Корзина для высоты в CAN
u8				CAN_BASKET_RC_DRIVING			[CAN_BUF];							// Корзина для дистанционки в CAN				
u8				CAN_BASKET_RC_DIR				[CAN_BUF];							// Корзина для дистанционки в CAN		
u8				CAN_BASKET_RC_SPEED_ENGINE		[CAN_BUF];							// Корзина для дистанционки в CAN
u8				CAN_BASKET_RC_START_BUTTON		[CAN_BUF];							// Корзина для дистанционки в CAN

u8 												SERVER_BUF_CNT = 0;
u8 												parcel_count = 0;

enum List_of_MODES_of_CONTROL					MODE_CONTROL = IDLE_MODE;
u8												total_NUM_Points_SaveZone = 0;		// Общее количество точек зоны безопасности
u8												total_NUM_Points_Route = 0;			// Общее количество точек маршрута

double 											MASTER_coord_latitude = 0;			// Итоговая широта с антенны блока управления
double 											MASTER_coord_longitude = 0;			// Итоговая долгота с антенны блока управления	

double 											SLAVE_coord_latitude = 0;			// Итоговая широта с антенны ровера
double 											SLAVE_coord_longitude = 0;			// Итоговая долгота с антенны ровера

double 											TARGET_coord_latitude = 0;			// Итоговая широта цели					
double 											TARGET_coord_longitude = 0;			// Итоговая долгота цели	


double 											HEADING = 0;						// Азимут
double 											ANGLE_btw_TARGET_N_DOZER = 0;		// Угол между машиной и целью
double 											DELTA_ANGLE_btw_CURSE_N_HEADING = 0;// Разниаца между Азимутом и углом к цели
float											SPEED_ENGINE = 0;

double const 									pi = 3.14159265359;					// Число ПИ			

u8 												RC_OFF_cnt = 5;						// Счетчик для отправки окончания дистанционного управления
u8												RC_SEND = 0;						// Флаг принятой посылки с дистанционным управлением


u8												REMOTE_CONNECT_OK; 					// Флаг успешного подключения с Планшетом

u8												incData;							// Флаг полученной посылки от Планшета


u8												Current_Route_Point = 0;			// Номер текущей проходимой точки
u8												Num_Route_Point_DONE = 0;			// Номер пройденной точки	
extern u8 next;
u8 												turn_in_place = 0;					// Флаг поворота на месте



u64 											u64_lat1 = 0;						// uint64 переменная для широты от второго ГНСС модема					
double	 										lat1_multiplier = 0;				// множитель широты

u64 											u64_lon1 = 0;						// uint64 переменная для долготы от второго ГНСС модема
double	 										lon1_multiplier = 0;				// множитель долготы

enum 											List_of_MOVING _curse;				// Объявление перечисления направлений движения

u8 												GPGGA = 0;							// Флаг получений NMEA координат от ГНСС модуля

volatile time_t									timer_CAN_coords = 0;				// Таймер для отслеживания получения координат от Ровера
volatile time_t									timer_CAN_DM1_Angles = 0;			// Таймер для отслеживания получения углов
volatile time_t									timer_CAN_DM1_Heights = 0;			// Таймер для отслеживания получения высот
volatile time_t									delta_timeout_Can_coords = 0;		// Вычисление таймаута получения координат от Ровера
volatile time_t 								Application_get_timer = 0;			// Таймер проверки связи с Планшетом
volatile time_t 								time_gps_parcel = 0;				// Таймер получений посылок от ГНСС модуля
volatile time_t 								D_time_gps_parcel = 0;				// Таймаут получения посылок от ГНСС модуля

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
static void MX_DMA_Init(void);
static void MX_USART6_UART_Init(void);
void StartDefaultTask(void *argument);
void StartLedTask(void *argument);
void StartRadioTask(void *argument);
void StartGpsTask(void *argument);
void StartmySysTickTask(void *argument);
void StartmyMobileApplicationTask(void *argument);
void StartmySendingServerDataTask(void *argument);
void StartmyControlTask(void *argument);
void StartUsartTask(void *argument);

/* USER CODE BEGIN PFP */
double filter(double val);
u8 crc8(u8 *pcBlock, u8 len);
i64 char_to_int (char *cti);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void CAN_Send(CAN_HandleTypeDef *hcan, u8 *data, u32 can_id);
void gsm_at_parse (char *result, volatile char *GSM_TEMP, char *left_mask, char *right_mask);
void clear_RXBuffer(u8 *RX_BUFER, u16 size);
void HAL_UARTExDebug_ReceiveToIdle_IT(void);
void HAL_UARTExRadio_ReceiveToIdle_IT(void);
void HAL_UARTExCOM1_GPS_ReceiveToIdle_DMA(void);
void HAL_UARTExCOM2_GPS_ReceiveToIdle_IT(void);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void SENDING_COORDS(void);
void Parse_To_RC_Command(u8 *ArrayData);
void Parse_To_GPS_Points(u8 *ArrayData);
void Parse_To_Total_Num_Coords(u8 *ArrayData);
void Parse_To_Mode(u8 *ArrayData);
void Parse_To_Error(u8* ArrayData);
void Parse_To_Confirm(u8 *ArrayData);
void Parce_To_CAN_Command(u8* ArrayData);
void GET_CONFIRM_MSG (u8 *ArrayData, u8 *OutputArray);
void Parse_To_NMEA(void);
void run_RC_drive(void);
double Azimuth_Calculating (double latitude1, double longitude1, double latitude2, double longitude2);
void Set_Left_Joystick_poti(i8 joy_y, i8 joy_x);
void run_gps_drive(void);
void getting_data(void);
u8 GET_DATA_PARCEL(char *xBuf, u8 *result_mas1);
void STOP_MOVING(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#define Z 0.01
double Value_old;				// Значение на предыдущей итерации фильтра
double Value = 0;				// Выходное значение фильтра

double filter(double val)		// Функция фитльтрации
{
	if (fabs(val - Value) < 1) 
		Value = Value_old + (val - Value_old) * Z;
	else
	{
		Value = val;
	}
	Value_old = Value;
	return Value;	
}

u8 crc8(u8 *pcBlock, u8 len)	// Функция вычисления контрольной суммы  	  
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


volatile time_t number1;
u8 negative_flag;
u8 ies;
u8 char_cnt;
u8 sizeofc = 0;
const char char_map[10] =																	// Карта символов для функции CHAR TO INT
{
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'
};

i64 char_to_int (char *cti)																	// Функция перевода из строки в число
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

uint8_t char_to_int_symbol (char cti)														// Функция перевода из строки в число
{
	uint8_t i_cnt = 0;

		for (i_cnt = 0; i_cnt < 10; i_cnt++)
		{
			if (cti == char_map[i_cnt]) 
			{
				break;
			}
		}
		
	return i_cnt;
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)											// Обработчик ошибок CAN
{
    uint32_t er = HAL_CAN_GetError(hcan);
	
}

void ReTransmitPacket(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef Header, u8 *data)		// Функция пересыла посылок из CAN2 в CAN1
{
	if (Header.IDE == CAN_ID_STD)
	CAN_Send(hcan, data, Header.StdId);
	
	if (Header.IDE == CAN_ID_EXT)
	CAN_Send(hcan, data, Header.ExtId);
}

u8 queue_1 = 0, queue_2 = 1, queue_init = 1;
u8 queue_cnt = 0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)								// Обработчик прерываний CAN
{
    if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &CAN1_RxHeader, CAN1_RxData) == HAL_OK) 	// CAN1
    {
		QUEUE_t msg;																		// Объаявление структуры очереди
		//BaseType_t xHigherPriorityTaskWoken;
		u16 IDx = 0;
		
		msg.data[0] = '$';																	// Формирование CAN пакета
		msg.data[1] = PACK_ID_dozerParams;													// Идентификатор CAN пакета
		
		
		for(u8 i = 0; i < 8; i++)
		{
			msg.data[i + 4] = CAN1_RxData[i];												// Сохраняем CAN пакет для отправки в очередь
		}	
	
		
		IDx = (CAN1_RxHeader.ExtId >> 8) & 0xFFFF;											// Отсекаем ненужную часть CAN ID для формирования пакета по протоколу
		
		switch(IDx)																			// Поиск CAN ID в списке
		{

				
//			case CAN_Tx_Modem_506_ID:
//										msg.data[2] = 0x06;
//										msg.data[3] = 0x05;
//										msg.data[12] = crc8(&msg.data[1], 11);
//										place = CAN_Tx_Modem_506_place;
//			isCanSendingAllowed = 1;
//			
//			break;
//			
//			case CAN_Tx_Modem_507_ID:
//										msg.data[2] = 0x07;
//										msg.data[3] = 0x05;
//										msg.data[12] = crc8(&msg.data[1], 11);
//										place = CAN_Tx_Modem_507_place;
//			isCanSendingAllowed = 1;
//			
//			break;
			
			case CAN_Tx_DM1_Angles:
										msg.data[2] = 0x11;
										msg.data[3] = 0x05;
										msg.data[12] = crc8(&msg.data[1], 11);
										place = CAN_Tx_DM1_Angles_place;
			timer_CAN_DM1_Angles = xTaskGetTickCount();
			isCanSendingAllowed = 1;
			
			break;
			
			case CAN_Tx_DM1_Heights:
										msg.data[2] = 0x12;
										msg.data[3] = 0x05;
										msg.data[12] = crc8(&msg.data[1], 11);
										place = CAN_Tx_DM1_Heights_place;
			timer_CAN_DM1_Heights = xTaskGetTickCount();
			isCanSendingAllowed = 1;
			
			break;
			
			case CAN_Tx_Modem_201_ID:
										msg.data[2] = 0x01;
										msg.data[3] = 0x02;
										msg.data[12] = crc8(&msg.data[1], 11);
										place = CAN_Tx_Modem_201_place;
			isCanSendingAllowed = 1;
			
			break;
			
			case CAN_Tx_Modem_205_ID:
										msg.data[2] = 0x05;
										msg.data[3] = 0x02;
										msg.data[12] = crc8(&msg.data[1], 11);
										place = CAN_Tx_Modem_205_place;
			isCanSendingAllowed = 1;
			
			break;
			
			case CAN_Tx_Modem_207_ID:
										msg.data[2] = 0x07;
										msg.data[3] = 0x02;
										msg.data[12] = crc8(&msg.data[1], 11);
										place = CAN_Tx_Modem_207_place;
			isCanSendingAllowed = 1;
			
			break;
			
			case CAN_Tx_Modem_209_ID:
										msg.data[2] = 0x09;
										msg.data[3] = 0x02;
										msg.data[12] = crc8(&msg.data[1], 11);
										place = CAN_Tx_Modem_209_place;
			isCanSendingAllowed = 1;
			
			break;
			
			case CAN_Tx_Modem_210_ID:
										msg.data[2] = 0x10;
										msg.data[3] = 0x02;
										msg.data[12] = crc8(&msg.data[1], 11);
										place = CAN_Tx_Modem_210_place;
			isCanSendingAllowed = 1;
			
			break;
			
			case CAN_Tx_Modem_215_ID:
										msg.data[2] = 0x15;
										msg.data[3] = 0x02;
										msg.data[12] = crc8(&msg.data[1], 11);
										place = CAN_Tx_Modem_215_place;
			isCanSendingAllowed = 1;
			
			break;
			
			case CAN_Tx_Modem_216_ID:
										msg.data[2] = 0x16;
										msg.data[3] = 0x02;
										msg.data[12] = crc8(&msg.data[1], 11);
										place = CAN_Tx_Modem_216_place;
			isCanSendingAllowed = 1;
			
			break;
			
			case CAN_Tx_Modem_221_ID:
										msg.data[2] = 0x21;
										msg.data[3] = 0x02;
										msg.data[12] = crc8(&msg.data[1], 11);
										place = CAN_Tx_Modem_221_place;
			isCanSendingAllowed = 1;
			
			break;
			
			case CAN_Tx_Modem_510_ID:
										msg.data[2] = 0x10;
										msg.data[3] = 0x05;
										msg.data[12] = crc8(&msg.data[1], 11);
										place = CAN_Tx_Modem_510_place;
//										GPSFixData.ReceiverMode_ = CAN_DATA[5];
			isCanSendingAllowed = 1;
			
			break;
			
			case CAN_Tx_Modem_303_ID:
										msg.data[2] = 0x03;
										msg.data[3] = 0x03;
										msg.data[12] = crc8(&msg.data[1], 11);
										place = CAN_Tx_Modem_303_place;
			isCanSendingAllowed = 1;
			
			break;
			
			case CAN_Tx_Modem_308_ID:
										msg.data[2] = 0x08;
										msg.data[3] = 0x03;
										msg.data[12] = crc8(&msg.data[1], 11);
										place = CAN_Tx_Modem_308_place;
			isCanSendingAllowed = 1;
			
			break;
			
//			case CAN_Tx_Modem_311_ID:
//										msg.data[2] = 0x11;
//										msg.data[3] = 0x03;
//										msg.data[12] = crc8(&msg.data[1], 11);
//										place = CAN_Tx_Modem_311_place;
//			isCanSendingAllowed = 1;
//			
//			break;
			
			case CAN_Tx_Modem_313_ID:
										msg.data[2] = 0x13;
										msg.data[3] = 0x03;
										msg.data[12] = crc8(&msg.data[1], 11);
										place = CAN_Tx_Modem_313_place;
			isCanSendingAllowed = 1;
			
			break;
			
			case CAN_Tx_Modem_314_ID:
										msg.data[2] = 0x14;
										msg.data[3] = 0x03;
										msg.data[12] = crc8(&msg.data[1], 11);
										place = CAN_Tx_Modem_314_place;
			isCanSendingAllowed = 1;
			
			break;
			
			case CAN_EE_F004_ID:
										msg.data[2] = 0x04;
										msg.data[3] = 0xF0;
										msg.data[12] = crc8(&msg.data[1], 11);
										place = CAN_EE_F004_place;
			isCanSendingAllowed = 1;
			
			break;
			
			case CAN_EE_FEEE_ID:
										msg.data[2] = 0xEE;
										msg.data[3] = 0xFE;
										msg.data[12] = crc8(&msg.data[1], 11);
										place = CAN_EE_FEEE_place;
			isCanSendingAllowed = 1;
			
			break;
			
			case CAN_EE_FEEF_ID:
										msg.data[2] = 0xEF;
										msg.data[3] = 0xFE;
										msg.data[12] = crc8(&msg.data[1], 11);
										place = CAN_EE_FEEF_place;
			isCanSendingAllowed = 1;
			
			break;
			
//			case CAN_EE_FEFC_ID:
//										msg.data[2] = 0xFC;
//										msg.data[3] = 0xFE;
//										msg.data[12] = crc8(&msg.data[1], 11);
//										place = CAN_EE_FEFC_place;
//			isCanSendingAllowed = 1;

//			break;
			
			case CAN_EE_FFA0_ID:
										msg.data[2] = 0xA0;
										msg.data[3] = 0xFF;
										msg.data[12] = crc8(&msg.data[1], 11);
										place = CAN_EE_FFA0_place;
			isCanSendingAllowed = 1;
						
						
			break;
		}
		if (isCanSendingAllowed)																// Если пришла посылка из списка
		{
			for(u8 i = 0; i < PACK_LEN_dozerParams; i++)										// Сохранение CAN посылок в единый массив
			{
				global_buf[PACK_LEN_dozerParams * place + i] = msg.data[i];
			}
			isCanSendingAllowed = 0;															// Обнуление флага принятой посылки из списка
		}
			
//		if (xTaskGetTickCount() - can_send_timer > 1000)
//		{
////		if (queue_cnt == 2)
////		{
//			can_send_timer = xTaskGetTickCount();
//			xQueueSendFromISR(SendQueueHandle, &msg.data, &xHigherPriorityTaskWoken);
////			queue_cnt++;
//			
//		}
         
    }
	
	if(HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &CAN2_RxHeader, CAN2_RxData) == HAL_OK)		// CAN2
    {
		if (CAN2_RxHeader.StdId == CAN_LAT1_EXT_ID)												// Широта
		{
			timer_CAN_coords = xTaskGetTickCount();												// Перезапуск таймера таймаута
			SLAVE_coord_latitude = 0;
			u64_lat1 = 0;
			lat1_multiplier = 100000000;														// Делитель широты
			
			for(u8 i = 0; i <= 4; i++)
			{
				u64_lat1 = u64_lat1 + ((u64)CAN2_RxData[i] << (i * 8));							// Собираем широту из CAN посылки
			}
						
			SLAVE_coord_latitude = (double)u64_lat1 / lat1_multiplier - 90;						// Преобразование координаты в десятичный вид
			GPSFixData.ReceiverMode_2 = CAN2_RxData[5];											// Качество решения
			GPSFixData.SatelliteNum_2 = CAN2_RxData[6];											// Количество спутников
//			NTRIP_CONNECT_OK_2 = CAN2_RxData[7];
		}	
		else if (CAN2_RxHeader.StdId == CAN_LON1_EXT_ID)										// Долгота
		{	
			timer_CAN_coords = xTaskGetTickCount();												// Перезапуск таймера таймаута
			SLAVE_coord_longitude = 0;
			u64_lon1 = 0;
			lon1_multiplier = 100000000;														// Делитель долготы
			
			for(u8 i = 0; i <= 4; i++)
			{
				u64_lon1 = u64_lon1 + ((u64)CAN2_RxData[i] << (i * 8));							// Собираем долготу из CAN посылки
			}
			
			SLAVE_coord_longitude = (double)u64_lon1 / lon1_multiplier - 180;	
		}
		else if (CAN2_RxHeader.StdId == CAN_ALT_ROVER_ID)										// Высота
		{
			
		}
		else
			ReTransmitPacket(&hcan1, CAN2_RxHeader, CAN2_RxData);								// Все остальные посылки пересылаем в CAN1
    }
}

void CAN_Send(CAN_HandleTypeDef *hcan, u8 *data, u32 can_id)                                   	// Функция отправки CAN сообщений
{
	if (hcan == &hcan1)																			// Если CAN1
	{
		if (can_id <= 0x7FF)																	// Если ID в диапазоне стандартной посылки
		{	
			CAN1_TxHeader.StdId = can_id;
			CAN1_TxHeader.ExtId = CAN_EXT_ID;
			CAN1_TxHeader.IDE = CAN_ID_STD;
		}
		else																					// Иначе ID - расширенный
		{
			CAN1_TxHeader.StdId = CAN_STD_ID;		
			CAN1_TxHeader.ExtId = can_id;
			CAN1_TxHeader.IDE = CAN_ID_EXT;
		}
						
		CAN1_TxHeader.RTR = CAN_RTR_DATA;					
		CAN1_TxHeader.DLC = 8;
		CAN1_TxHeader.TransmitGlobalTime = DISABLE;	

		CAN1_TxData[0] = data[0];			
		CAN1_TxData[1] = data[1];
		CAN1_TxData[2] = data[2];
		CAN1_TxData[3] = data[3];
		CAN1_TxData[4] = data[4];		
		CAN1_TxData[5] = data[5];
		CAN1_TxData[6] = data[6];
		CAN1_TxData[7] = data[7];

		HAL_CAN_AddTxMessage(hcan, &CAN1_TxHeader, CAN1_TxData, &CAN1_TxMailbox);
	}
	
	if (hcan == &hcan2)																			// Если CAN2
	{
		if (can_id <= 0x7FF)																	// Если ID в диапазоне стандартной посылки
		{	
			CAN2_TxHeader.StdId = can_id;
			CAN2_TxHeader.ExtId = CAN_EXT_ID;
			CAN2_TxHeader.IDE = CAN_ID_STD;
		}
		else																					// Иначе ID - расширенный
		{
			CAN2_TxHeader.StdId = CAN_STD_ID;		
			CAN2_TxHeader.ExtId = can_id;
			CAN2_TxHeader.IDE = CAN_ID_EXT;
		}
						
		CAN2_TxHeader.RTR = CAN_RTR_DATA;					
		CAN2_TxHeader.DLC = 8;
		CAN2_TxHeader.TransmitGlobalTime = DISABLE;	

		CAN2_TxData[0] = data[0];			
		CAN2_TxData[1] = data[1];
		CAN2_TxData[2] = data[2];
		CAN2_TxData[3] = data[3];
		CAN2_TxData[4] = data[4];		
		CAN2_TxData[5] = data[5];
		CAN2_TxData[6] = data[6];
		CAN2_TxData[7] = data[7];

		HAL_CAN_AddTxMessage(hcan, &CAN2_TxHeader, CAN2_TxData, &CAN2_TxMailbox);
	}
	osDelay(1);
}

void gsm_at_parse (char *result, volatile char *GSM_TEMP, char *left_mask, char *right_mask) 	// *GSM_TEMP = наша команда
{	
	u8 i = 0, j = 0, del = 0;								                            		// *left_mask = символы перед нужной нам частью строки
	char *startstring; 											            					// *right_mask = символы после нужной нам части строки
	char *endstring;
	u8 sofleft = 0;
	u16 sofstart = 0;
	u16 sofend = 0;
	sofleft = strlen(left_mask);
    startstring = strstr((const char *)GSM_TEMP, left_mask); 									// ищем left_mask
    startstring = startstring + sofleft;														// Получаем указатель на символ следующий за left mask 

	endstring = strstr(startstring, right_mask);												// Получаем указатель на строку до right mask
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

void clear_RXBuffer(u8 *RX_BUFER, u16 size)           											// Функция очистки буфера
{
    u16 i = 0;
    for (i = 0; i < size; i++) RX_BUFER[i] = 0;
}

void HAL_UARTExDebug_ReceiveToIdle_IT(void)													
{
	HAL_UARTEx_ReceiveToIdle_IT(UART_DEBUG_HANDLE, (u8 *)RADIO_DATA1, 30);
}

void HAL_UARTExRadio_ReceiveToIdle_IT(void)
{
	HAL_UARTEx_ReceiveToIdle_IT(UART_RADIO_HANDLE, (u8 *)RADIO_DATA1, 30);
}

void HAL_UARTExCOM1_GPS_ReceiveToIdle_DMA(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(UART_COM1_GPS_HANDLE, (u8 *)COM1_GPS_DATA, COM1_GPS_BUF);
	__HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
}

void HAL_UARTExCOM2_GPS_ReceiveToIdle_IT(void)
{
	HAL_UARTEx_ReceiveToIdle_IT(UART_COM2_GPS_HANDLE, (u8 *)COM2_GPS_DATA, GPS_BUF);
	//__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

u8 	pack_length = 0;
u8 dollar_flag = 0;
u8 size_inc = 0;
time_t usart_time = 0;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart == UART_COM2_GPS_HANDLE)
	{
		GPGGA = 1;
		time_gps_parcel = xTaskGetTickCount();
		HAL_UARTExCOM2_GPS_ReceiveToIdle_IT();
	}
	
	if (huart == UART_RADIO_HANDLE)
	{
		usart_time = xTaskGetTickCount();
		size_inc += Size;
			if (SERVER_BUF_CNT == 2)
		{
			__nop();
		}
		if (virtual_surv)
		{
			HAL_UART_Transmit_IT(UART_COM1_GPS_HANDLE, (u8 *)RADIO_DATA1,Size);
		}
		else
		{
			if (RADIO_DATA1[0] == '$') dollar_flag = 1;
			
			if (dollar_flag)
			{
				if (pack_length == 0)
				{
					switch (RADIO_DATA1[1])
					{
						case (u8)PACK_ID_tNum: pack_length = PACK_LEN_tNum;
						break;
						case (u8)PACK_ID_Coords: pack_length = PACK_LEN_Coords;
						break;
						case (u8)PACK_ID_Mode: pack_length = PACK_LEN_Mode;
						break;
						case (u8)PACK_ID_RC: pack_length = PACK_LEN_RC;
						break;
						case (u8)PACK_ID_Err: pack_length = PACK_LEN_Err;
						break;
						case (u8)PACK_ID_Confirm: pack_length = PACK_LEN_ALIVE;
						break;	
						case (u8)PACK_ID_Request: pack_length = PACK_LEN_Request;
						break;						
						
	//					default: pack_length = 0;
					}
				}
				if (pack_length == 0)
				{					
					dollar_flag = 0;
					SERVER_BUF_CNT = 0;
					return;
				}
				
				for(u8 i = 0; i < Size; i++)
				{
					RECEIVE_SERVER_BUF[SERVER_BUF_CNT++] = RADIO_DATA1[i];
				}
				
				if (SERVER_BUF_CNT == pack_length + 1) 
				{
					incData = 1;
					parcel_count++;
					pack_length = 0;
					dollar_flag = 0;
					
				}
				
				if (SERVER_BUF_CNT > pack_length + 1)
				{
					pack_length = 0;
					dollar_flag = 0;
					SERVER_BUF_CNT = 0;
				}
			}
			
			
			
			if (virtual_bt)
			CDC_Transmit_FS((u8 *)RADIO_DATA1, Size);
			
			if (RADIO_DATA1[17] != 0)
			{
				__nop();
			}
//			if (RADIO_DATA[0] == '$')
			
			
			
		}
		
	}
	
	if (huart == UART_COM1_GPS_HANDLE)
	{
		if (virtual_gps)
		{
			CDC_Transmit_FS((u8 *)COM1_GPS_DATA, Size);
			//clear_RXBuffer(GSM_DATA, Size);
		}
		if (virtual_surv)
		{
			HAL_UART_Transmit_IT(UART_RADIO_HANDLE, (u8 *)COM1_GPS_DATA,Size);
		}
		
		HAL_UARTExCOM1_GPS_ReceiveToIdle_DMA();
	}
	HAL_UARTExRadio_ReceiveToIdle_IT();
}
double myLat = 55.216422;
double myLon = 61.441115;
u8 rec_m = 0;
void SENDING_CAN(void)
{
	QUEUE_t msg;
	msg.data[1] = PACK_ID_dozerParams;
	xQueueSend(SendQueueHandle, &msg.data, osWaitForever);
}

void SENDING_COORDS(void)
{
	QUEUE_t msg;
	//myLat += 0.00001;
	//myLon += 0.00001;
	u64 mulLat = (MASTER_coord_latitude + 90) * pow(10, 8);
	u64 mulLon = (MASTER_coord_longitude + 180) * pow(10, 8);

	
	msg.data[0] = '$';
	msg.data[1] = 0x12;
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
	msg.data[14] = char_to_int_symbol(GPSFixData.ReceiverMode);
	msg.data[15] = char_to_int(GPSFixData.SatelliteNum);
	msg.data[16] = MODE_CONTROL;
	msg.data[17] = crc8(&msg.data[1], 16);
	
	rec_m++;
	
	if (rec_m == 3) rec_m++;
	
	if (rec_m > 5) rec_m = 0;
	xQueueSend(SendQueueHandle, &msg.data, osWaitForever);
}

u8 left_y1 = 0;
u8 left_y2 = 0;
u8 left_x1 = 0;
u8 left_x2 = 0;
u8 right_y1 = 0;
u8 right_y2 = 0;
u8 right_x1 = 0;
u8 right_x2 = 0;
u8 left_btn = 0;
u8 right_btn = 0;
u8 speed_level = 0;
u8 start_bit;
u8 RC_BTN_SEND = 0;
volatile time_t	RC_timer = 0;
volatile time_t	delta_timeout_RC = 0;

void Parse_To_RC_Command(u8 *ArrayData)
{
	switch (ArrayData[4]) 
	{
		case 0: left_btn = 0x00;	
				right_btn = 0x00;    
		break;
		case 1: left_btn = 0x08;	
				right_btn = 0x00;    
		break;
		case 2: left_btn = 0x10;	
				right_btn = 0x00; 	
		break;
		case 4: left_btn = 0x00;	
				right_btn = 0x08; 	
		break;
		case 8: left_btn = 0x00;	
				right_btn = 0x10; 	
		break;
		default: left_btn = 0x00; 	
				right_btn = 0x00;    
		break;
	}
	
	if (ArrayData[6] == 100) 
	{
		left_y1 = 0;
		left_y2 = 0;
	}
	else if(ArrayData[6] > 100) 
	{
		left_y1 = ArrayData[6] - 100;
		left_y2 = 255 - left_y1;
	}
	else 
	{
		left_y2 = 100 - ArrayData[6];
		left_y1 = 255 - left_y2;
	}
/************************************************************/
	if (ArrayData[5] == 100) 
	{
		left_x1 = 0;
		left_x2 = 0;
	}
	else if(ArrayData[5] > 100) 
	{
		left_x1 = ArrayData[5] - 100;
		left_x2 = 255 - left_x1;
	}
	else 
	{
		left_x2 = 100 - ArrayData[5];
		left_x1 = 255 - left_x2;
	}
/************************************************************/
	if (ArrayData[8] == 100) 
	{
		right_y1 = 0;
		right_y2 = 0;
	}
	else if(ArrayData[8] > 100) 
	{
		right_y1 = ArrayData[8] - 100;
		right_y2 = 255 - right_y1;
	}
	else 
	{
		right_y2 = 100 - ArrayData[8];
		right_y1 = 255 - right_y2;
	}
/************************************************************/
	if (ArrayData[7] == 100) 
	{
		right_x1 = 0;
		right_x2 = 0;
	}
	else if(ArrayData[7] > 100) 
	{
		right_x1 = ArrayData[7] - 100;
		right_x2 = 255 - right_x1;
	}
	else 
	{
		right_x2 = 100 - ArrayData[7];
		right_x1 = 255 - right_x2;
	}
/************************************************************/
   
	speed_level = ArrayData[9];                    
	if(speed_level >= 255) speed_level = 255;
	if (MODE_CONTROL == MANUAL_CONTROL_ON)
	{
 
	}		
	
	CAN_BASKET_RC_SPEED_ENGINE[0] = 0xFF; 			
	CAN_BASKET_RC_SPEED_ENGINE[1] = 0xFF; 			
	CAN_BASKET_RC_SPEED_ENGINE[2] = 0xFF;     		
	CAN_BASKET_RC_SPEED_ENGINE[3] = speed_level;	
	CAN_BASKET_RC_SPEED_ENGINE[4] = 0xFF;        	
//	CAN_BASKET_RC_SPEED_ENGINE[5] = RC_MANUAL << CONTROL_TYPE | SICK_IGNORING << SICK_DEFINITION; 	// Идентификатор автономного управления 0 bit [0 - ручное, 1 - автономное]
	CAN_BASKET_RC_SPEED_ENGINE[5]=0xff;																								// Игнорирования препятствия			1 bit [0 - блокировавка, 1 - игнор]
	CAN_BASKET_RC_SPEED_ENGINE[6] = 0x00;        	
	CAN_BASKET_RC_SPEED_ENGINE[7] = 0x01;        	
	
	CAN_BASKET_RC_START_BUTTON[0] = 0x00;
	CAN_BASKET_RC_START_BUTTON[1] = 0x00;
	CAN_BASKET_RC_START_BUTTON[2] = 0x00;
	CAN_BASKET_RC_START_BUTTON[3] = 0x00;
	CAN_BASKET_RC_START_BUTTON[4] = 0x00;
	CAN_BASKET_RC_START_BUTTON[5] = 0x00;
	CAN_BASKET_RC_START_BUTTON[6] = 0x00;
	
	if(ArrayData[10] == 0xF8)
	{
		start_bit = 0x01;
		CAN_BASKET_RC_START_BUTTON[7] = 0xF1;
		RC_BTN_SEND = 1;
	}                        
	if(ArrayData[10] == 0x00) 
	{
		start_bit = 0x00;
		CAN_BASKET_RC_START_BUTTON[7] = 0xF0;
		RC_BTN_SEND = 1;
	}
	
	RC_timer = xTaskGetTickCount();
	delta_timeout_RC = 0;
	
	RC_SEND = 1;
}

void Parse_To_GPS_Points(u8 *ArrayData)
{
	enum List_of_Points_Type _type;
	u8 i = 0;
	u8 index = ArrayData[3] % RoutePoints_COUNT;
	double latitude = 0;
	double longitude = 0;

	u64 coor_lat[5];
	u64 coor_lon[5];

	coor_lat[0] = ArrayData[4];
	coor_lat[1] = ArrayData[5];
	coor_lat[2] = ArrayData[6];
	coor_lat[3] = ArrayData[7];
	coor_lat[4] = ArrayData[8];
	
	coor_lon[0] = ArrayData[9];
	coor_lon[1] = ArrayData[10];
	coor_lon[2] = ArrayData[11];
	coor_lon[3] = ArrayData[12];
	coor_lon[4] = ArrayData[13];
	
	
	if (ArrayData[2] == PARCEL_ID_SaveCoord) _type = SAVE_ZONE;
	if (ArrayData[2] == PARCEL_ID_PointCoord) _type = ROUTE;
	
	if (_type == SAVE_ZONE)
	{
		for(i = 0; i <= 4; i++)
		{
			latitude = latitude + (coor_lat[i] << (i * 8));
			longitude = longitude + (coor_lon[i] << (i * 8));
		}
		coordSaveZone[index].latitude = latitude / 100000000 - 90;
		coordSaveZone[index].longitude = longitude / 100000000 - 180;
	}	
	
	if (_type == ROUTE)
	{
		for(i = 0; i <= 4; i++)
		{
			latitude = latitude + (coor_lat[i] << (i * 8));
			longitude = longitude + (coor_lon[i] << (i * 8));
		}
		coordRoute[index].latitude = latitude / 100000000 - 90;
		coordRoute[index].longitude = longitude / 100000000 - 180;
		
	}
	
}

void Parse_To_Total_Num_Coords(u8 *ArrayData)
{
	u8 _type = 0;
	
	if (ArrayData[2] == PARCEL_ID_SaveCoord) _type = SAVE_ZONE;
	if (ArrayData[2] == PARCEL_ID_PointCoord) _type = ROUTE;
	if (_type == SAVE_ZONE)
	{
		total_NUM_Points_SaveZone = ArrayData[3];
	}
	
	if (_type == ROUTE)
	{
		total_NUM_Points_Route = ArrayData[3];
	}
}

void Parse_To_Mode(u8 *ArrayData)
{
	if (ArrayData[2] == PARCEL_ID_Mode)
	{
		MODE_CONTROL = (enum List_of_MODES_of_CONTROL)ArrayData[3];
		
		if (MODE_CONTROL == MANUAL_CONTROL_ON)
		{
			RC_timer = xTaskGetTickCount();
			delta_timeout_RC = 0;
		}
	}
}

void Parse_To_Error(u8* ArrayData)
{
}

void Parse_To_Confirm(u8 *ArrayData)
{
}

void Parce_To_CAN_Command(u8* ArrayData)
{
	u32 ext_id = 0;
	u8 id0 = ArrayData[2];
	u8 id1 = ArrayData[3];
	u8 localCAN_BASKET[8];
	
	if (id0 == 0x12)
	{
		__nop();
	}
	
	ext_id = CAN_EXT_ID_MASK | id0 << 8 | id1 << 16;
	localCAN_BASKET[0] = ArrayData[4];
	localCAN_BASKET[1] = ArrayData[5];
	localCAN_BASKET[2] = ArrayData[6];
	localCAN_BASKET[3] = ArrayData[7];
	localCAN_BASKET[4] = ArrayData[8];
	localCAN_BASKET[5] = ArrayData[9];
	localCAN_BASKET[6] = ArrayData[10];
	localCAN_BASKET[7] = ArrayData[11];
	
	CAN_Send(&hcan1, localCAN_BASKET, ext_id);
	
}

void GET_CONFIRM_MSG (u8 *ArrayData, u8 *OutputArray)
{
	OutputArray[0] = ArrayData[0];
	OutputArray[1] = 0xFE;
	OutputArray[2] = ArrayData[1];
	OutputArray[3] = ArrayData[2];
	OutputArray[4] = ArrayData[3];
	OutputArray[5] = crc8((u8 *)&OutputArray[1], 4);
}

u32    	j = 0; 
u32    	g = 0;
u32    	gh = 0;
u8 		statComma = 0;
u8 		float_flag = 0;
i64 	GPS_al  = 251156;									// Высота с GPS модуля после парсинга NMEA

i32 	lat_int;
i32 	lon_int;
u32 	lat_float;
u32 	lon_float;

u32 ilat = 0;
double flat_minute_int = 0;
double flat_minute_dec = 0;
double flat_dec = 0;
double flat_fin = 0;
u32		mul_lat = 1;

u32 ilon = 0;
double flon_minute_int = 0;
double flon_minute_dec = 0;
double flon_dec = 0;
double flon_fin = 0;
u32		mul_lon = 1;

void Parse_To_NMEA(void)
{
	for (u16 i = 0; i < strlen(COM2_GPS_DATA); i++)
	{
		if (COM2_GPS_DATA[i] == 0x0D) 												// Ищем конец сообщения
		{
			statComma = 0;														// Обнуляем счетчик запятых

			lat_int =  char_to_int(GPSFixData.Latitude_int);					// Переводим в число целую часть широты
			lat_float = char_to_int(GPSFixData.Latitude_float);					// Переводим в число десятичную часть широты
			lon_int =  char_to_int(GPSFixData.Longitude_int);					// Переводим в число целую часть долготы
			lon_float = char_to_int(GPSFixData.Longitude_float);				// Переводим в число десятичну часть долготы
			GPS_al  = char_to_int(GPSFixData.alt_full);							// Получаем конечное INT значение высоты
			
			if (GPS_al < -10000) GPS_al = 0xffffffff;
			
			ilat = (u32)lat_int / 100;
			flat_minute_int = (u32)lat_int % 100;
			flat_minute_dec = (double)lat_float / mul_lat;
			flat_dec = (flat_minute_int + flat_minute_dec) / 60;
			flat_fin = ilat + flat_dec;
			
			
			ilon = (u32)lon_int / 100;
			flon_minute_int = (u32)lon_int % 100;
			flon_minute_dec = (double)lon_float / mul_lon;
			flon_dec = (flon_minute_int + flon_minute_dec) / 60;
			flon_fin = ilon + flon_dec;
			
			
//			if (flat_fin < 56 && flat_fin > 53)
			MASTER_coord_latitude = flat_fin;			// Получаем конечное INT значение широты
//			if (flon_fin < 62 && flon_fin > 60)
			MASTER_coord_longitude = flon_fin; 			// Получаем конечное INT значение долготы
			
			CAN_BASKET_ALT[0] = 4;
			CAN_BASKET_ALT[1] = GPS_al & 0xff;
			CAN_BASKET_ALT[2] = (GPS_al & 0xff00) >> 8;
			CAN_BASKET_ALT[3] = (GPS_al & 0xff0000) >> 16;
			CAN_BASKET_ALT[4] = (GPS_al & 0xff000000) >> 24;
			CAN_BASKET_ALT[5] = GPSFixData.ReceiverMode_2;
			CAN_BASKET_ALT[6] = GPSFixData.SatelliteNum_2;
			CAN_BASKET_ALT[7] = 0;
				
			global_buf[PACK_LEN_dozerParams * CAN_Tx_Modem_510_place + 0] = '$';
			global_buf[PACK_LEN_dozerParams * CAN_Tx_Modem_510_place + 1] = PACK_ID_dozerParams;
			global_buf[PACK_LEN_dozerParams * CAN_Tx_Modem_510_place + 2] = 0x10;
			global_buf[PACK_LEN_dozerParams * CAN_Tx_Modem_510_place + 3] = 0x05;
			for(u8 i = 0; i < 8; i++)
			{
				global_buf[PACK_LEN_dozerParams * CAN_Tx_Modem_510_place + i + 4] = CAN_BASKET_ALT[i];
			}
			global_buf[PACK_LEN_dozerParams * CAN_Tx_Modem_510_place + 12] = crc8(&global_buf[CAN_Tx_Modem_510_place * PACK_LEN_dozerParams + 1], 11);
		
			clear_RXBuffer((u8 *)GPSFixData.alt_full, sizeof(GPSFixData.alt_full));
			clear_RXBuffer((u8 *)GPSFixData.Latitude_float, sizeof(GPSFixData.Latitude_float));
			clear_RXBuffer((u8 *)GPSFixData.Latitude_int, sizeof(GPSFixData.Latitude_int));
			clear_RXBuffer((u8 *)GPSFixData.Longitude_int, sizeof(GPSFixData.Longitude_int));
			clear_RXBuffer((u8 *)GPSFixData.Longitude_float, sizeof(GPSFixData.Longitude_float));
			
			mul_lat = 1;
			mul_lon = 1;

			clear_RXBuffer((u8 *)COM2_GPS_DATA, GPS_BUF);
				
		}
		
		if (COM2_GPS_DATA[i] != 0x2C)												// Ищим запятую
		{
			switch(statComma)													// По количеству запятых определяем нужный параметр в сообщении
			{
				case 1: GPSFixData._time[g++] = COM2_GPS_DATA[i]; break;				// Записывам время

				case 2:  														// Записываем широту
					  if (float_flag == 0) GPSFixData.Latitude_int[g++] = COM2_GPS_DATA[i]; 
					  else 
					  {
						  GPSFixData.Latitude_float[g++] = COM2_GPS_DATA[i];
						  mul_lat *= 10;
					  }
					  
					  if (COM2_GPS_DATA[i] == 0x2E) { float_flag = 1; g = 0; }
				break;

				case 3: GPSFixData.NS = COM2_GPS_DATA[i]; break;						// Записываем Север/Юг

				case 4: 														// Записываем долготу
					  if (float_flag == 0) GPSFixData.Longitude_int[g++] = COM2_GPS_DATA[i]; 
					  else 
					  {
						  GPSFixData.Longitude_float[g++] = COM2_GPS_DATA[i];
						  mul_lon *= 10;
					  }
					  
					  if (COM2_GPS_DATA[i] == 0x2E) { float_flag = 1; g = 0; }
				break;

				case 5: GPSFixData.EW = COM2_GPS_DATA[i]; break;								  // Записываем Запад/Восток

				case 6: GPSFixData.ReceiverMode = COM2_GPS_DATA[i]; 
				break;			// Записываем код качества сигнала

				case 7: GPSFixData.SatelliteNum[g++] = COM2_GPS_DATA[i]; break;		// Записываем количество видимых спутников

				case 9: 														// Записываем высоту
					GPSFixData.alt_full[gh++] = COM2_GPS_DATA[i];

					if (COM2_GPS_DATA[i] == 0x2E) { float_flag = 1; g = 0; }
				break;
			}
		}
		else if (GPGGA)
		{
			statComma++;														// Счетчик запятых
			float_flag = 0;
			g = 0;
			gh = 0;
		}
	}
}


time_t time_to_send_rc;
time_t Dtime_to_send_rc;

void run_RC_drive(void)
{	
	 
	if (RC_SEND)
	{
		Dtime_to_send_rc = xTaskGetTickCount() - time_to_send_rc;
		RC_SEND = 0;
		
		CAN_BASKET_RC_DRIVING[0] = left_y1;    	
		CAN_BASKET_RC_DRIVING[1] = left_x1;    	
		CAN_BASKET_RC_DRIVING[2] = left_y2;    	
		CAN_BASKET_RC_DRIVING[3] = left_x2;    	                    
		CAN_BASKET_RC_DRIVING[4] = 0xFF;        	
		CAN_BASKET_RC_DRIVING[5] = left_btn;    	
		CAN_BASKET_RC_DRIVING[6] = 0x00;        	
		CAN_BASKET_RC_DRIVING[7] = 0xFF;        	
					
		CAN_BASKET_RC_DIR[0] = right_y1;    
		CAN_BASKET_RC_DIR[1] = right_x1;    
		CAN_BASKET_RC_DIR[2] = right_y2;    
		CAN_BASKET_RC_DIR[3] = right_x2;    
		CAN_BASKET_RC_DIR[4] = 0xFF;        
		CAN_BASKET_RC_DIR[5] = right_btn;
		CAN_BASKET_RC_DIR[6] = 0x00;        
		CAN_BASKET_RC_DIR[7] = 0xFF;
		
		time_to_send_rc = xTaskGetTickCount();
	}
}

double deltaLon = 0, e = 0;
double aa = 6378240, bb = 6356860;
double deltaLon_rad = 0;

double latitude1_rad;       // Пересчет координат цели в радианы
double longitude1_rad;
double latitude2_rad;
double longitude2_rad;
double alfa = 0;

double Azimuth_Calculating (double latitude1, double longitude1, double latitude2, double longitude2)
{
	// Формула на сайте https://planetcalc.ru/713/
	latitude1_rad 	= latitude1 	* DEG_TO_RAD;       // Пересчет координат цели в радианы
	longitude1_rad 	= longitude1 	* DEG_TO_RAD;
	latitude2_rad 	= latitude2 	* DEG_TO_RAD;
	longitude2_rad 	= longitude2 	* DEG_TO_RAD;

	if (fabs(longitude2 - longitude1) <= 180) 	deltaLon = longitude2 - longitude1;
	if ((longitude2 - longitude1) < -180) 		deltaLon = 360 + longitude2 - longitude1;
	if ((longitude2 - longitude1) > 180) 		deltaLon = longitude2 - longitude1 - 360;
	
	deltaLon_rad = deltaLon * DEG_TO_RAD;
	e = sqrt(1 - (bb * bb /aa / aa));

	alfa = atan2(deltaLon_rad, (log((tan((pi / 4) + (latitude2_rad / 2)) * pow((1 - e * sin(latitude2_rad)) / (1 + e * sin(latitude2_rad)), e / 2))) - log((tan((pi / 4) + (latitude1_rad / 2)) * pow((1 - e * sin(latitude1_rad)) / (1 + e * sin(latitude1_rad)), e / 2)))));
	
	alfa = alfa * RAD_TO_DEG;
	if (alfa < 0) alfa = 360 + alfa;

	return alfa;
}

void Set_Left_Joystick_poti(i8 joy_y, i8 joy_x)
{
	if (joy_y>100) joy_y = 100;
	if (joy_y<-100) joy_y = -100;
	if (joy_x>100) joy_x = 100;
	if (joy_x<-100) joy_x = -100;
	
	if (joy_y > 0)
	{
		CAN_BASKET_RC_DRIVING[1] = joy_y;
		CAN_BASKET_RC_DRIVING[3] = 255 - CAN_BASKET_RC_DRIVING[1];
	}
	else if (joy_y < 0)
	{
		CAN_BASKET_RC_DRIVING[3] = joy_y * (-1);
		CAN_BASKET_RC_DRIVING[1] = 255 - CAN_BASKET_RC_DRIVING[3];
	}
	else
	{
		CAN_BASKET_RC_DRIVING[1] = STOP_DRIVE;
		CAN_BASKET_RC_DRIVING[3] = STOP_DRIVE;	
	}
	
	if (joy_x > 0)
	{
		CAN_BASKET_RC_DRIVING[0] = joy_x;
		CAN_BASKET_RC_DRIVING[2] = 255 - CAN_BASKET_RC_DRIVING[0];
	}
	else if (joy_x < 0)
	{
		CAN_BASKET_RC_DRIVING[2] = joy_x * (-1);
		CAN_BASKET_RC_DRIVING[0] = 255 - CAN_BASKET_RC_DRIVING[2];	
	}
	else
	{
		CAN_BASKET_RC_DRIVING[2] = STOP_DRIVE;
		CAN_BASKET_RC_DRIVING[0] = STOP_DRIVE;
	}
}

void Set_Right_Joystick_poti(i8 joy_y, i8 joy_x)
{
	if (joy_y>100) joy_y = 100;
	if (joy_y<-100) joy_y = -100;
	if (joy_x>100) joy_x = 100;
	if (joy_x<-100) joy_x = -100;
	
	if (joy_y > 0)
	{
		CAN_BASKET_RC_DIR[1] = joy_y;
		CAN_BASKET_RC_DIR[3] = 255 - CAN_BASKET_RC_DIR[1];
	}
	else if (joy_y < 0)
	{
		CAN_BASKET_RC_DIR[3] = joy_y * (-1);
		CAN_BASKET_RC_DIR[1] = 255 - CAN_BASKET_RC_DIR[3];
	}
	else
	{
		CAN_BASKET_RC_DIR[1] = STOP_DRIVE;
		CAN_BASKET_RC_DIR[3] = STOP_DRIVE;	
	}
	
	if (joy_x > 0)
	{
		CAN_BASKET_RC_DIR[0] = joy_x;
		CAN_BASKET_RC_DIR[2] = 255 - CAN_BASKET_RC_DIR[0];
	}
	else if (joy_x < 0)
	{
		CAN_BASKET_RC_DIR[2] = joy_x * (-1);
		CAN_BASKET_RC_DIR[0] = 255 - CAN_BASKET_RC_DIR[2];	
	}
	else
	{
		CAN_BASKET_RC_DIR[2] = STOP_DRIVE;
		CAN_BASKET_RC_DIR[0] = STOP_DRIVE;
	}
}

double 			check_zone = 0;						// Радиус вокруг цели для проверки попадания в точку маршрута
double 			Target_lat_i;
double 			Target_lon_i;
double 			My_lat_i;
double 			My_lon_i;
u8 				turn_step = 0;
i16 			joy_x_set = 0;
i16 			joy_y_set = 0;
time_t timeout_navesnoe = 0;
u8 Navesnoe_ready = 0;
u8 Navesnoe_step = 0;


void run_gps_drive(void)  // Функция работы автоматического движения по gps координатам
{
	
	QUEUE_t msg;
	TARGET_coord_latitude = coordRoute[Current_Route_Point].latitude;
	TARGET_coord_longitude = coordRoute[Current_Route_Point].longitude;
	
	Target_lat_i = TARGET_coord_latitude * 10000000;
	Target_lon_i = TARGET_coord_longitude * 10000000;
	My_lat_i = MASTER_coord_latitude * 10000000;
	My_lon_i = MASTER_coord_longitude * 10000000;
	
//	TARGET_coord_latitude = 50;
//	TARGET_coord_longitude = 60;
//	SLAVE_coord_latitude = 51;
//	SLAVE_coord_longitude = 61;
//	MASTER_coord_latitude = 52;
//	MASTER_coord_longitude = 62;
	if (Navesnoe_ready)
	{
		if (TARGET_coord_latitude != 0 && TARGET_coord_longitude !=0 &&	MASTER_coord_latitude !=0 && MASTER_coord_longitude !=0 && SLAVE_coord_latitude != 0 && SLAVE_coord_longitude != 0)
		{
			check_zone = sqrt((My_lat_i - Target_lat_i) * (My_lat_i - Target_lat_i) + (My_lon_i - Target_lon_i) * (My_lon_i - Target_lon_i));
			ANGLE_btw_TARGET_N_DOZER = Azimuth_Calculating(MASTER_coord_latitude, MASTER_coord_longitude, TARGET_coord_latitude, TARGET_coord_longitude);
	//		// Задание допустимого радиуса цели
			if (check_zone > 130 && !next)
			{
				if ((ANGLE_btw_TARGET_N_DOZER < 180) && (HEADING < 180))                 // Вычисление направления поворота движения
				{
					DELTA_ANGLE_btw_CURSE_N_HEADING = fabs(ANGLE_btw_TARGET_N_DOZER - HEADING);
					
					if ((ANGLE_btw_TARGET_N_DOZER - HEADING) < -1)			_curse = LEFT_DRIVE;
					else if ((ANGLE_btw_TARGET_N_DOZER - HEADING) > 1)		_curse = RIGHT_DRIVE;
					else													_curse = FORWARD_DRIVE;
				}

				if ((ANGLE_btw_TARGET_N_DOZER < 180) && (HEADING > 180))
				{
					if ((360 - HEADING + ANGLE_btw_TARGET_N_DOZER) < 180)
					{
																			_curse = RIGHT_DRIVE;
						DELTA_ANGLE_btw_CURSE_N_HEADING = 360 - HEADING + ANGLE_btw_TARGET_N_DOZER;;
					}
					else
					{
																			_curse = LEFT_DRIVE;
						DELTA_ANGLE_btw_CURSE_N_HEADING = HEADING - ANGLE_btw_TARGET_N_DOZER;
					}
				}

				if ((ANGLE_btw_TARGET_N_DOZER > 180) && (HEADING < 180))
				{
					
					if ((360 - ANGLE_btw_TARGET_N_DOZER + HEADING) < 180)    
					{
																			_curse = LEFT_DRIVE;
						DELTA_ANGLE_btw_CURSE_N_HEADING = 360 - ANGLE_btw_TARGET_N_DOZER + HEADING;
					}
					else
					{
																			_curse = RIGHT_DRIVE;
						DELTA_ANGLE_btw_CURSE_N_HEADING = ANGLE_btw_TARGET_N_DOZER - HEADING;
					} 
				
				}

				if ((ANGLE_btw_TARGET_N_DOZER > 180) && (HEADING > 180))
				{
					DELTA_ANGLE_btw_CURSE_N_HEADING = fabs(ANGLE_btw_TARGET_N_DOZER - HEADING);
					if ((ANGLE_btw_TARGET_N_DOZER - HEADING) > 1)			_curse = RIGHT_DRIVE;
					else if ((ANGLE_btw_TARGET_N_DOZER - HEADING) < -1)		_curse = LEFT_DRIVE;
					else                                          			_curse = FORWARD_DRIVE;
				}
			}
			else if (Num_Route_Point_DONE < total_NUM_Points_Route)
			{
				Navesnoe_ready = 0;
				next = 0;
				msg.data[0] = '$';
				msg.data[1] = 0x13;
				msg.data[2] = Num_Route_Point_DONE;
				msg.data[3] = crc8(&msg.data[1], 2);
				
				xQueueSend(SendQueueHandle, &msg.data, osWaitForever);
				
				coordRoute[Current_Route_Point].latitude = 0;
				coordRoute[Current_Route_Point].longitude = 0;
				Current_Route_Point++;
				Num_Route_Point_DONE++;
				
				if (Current_Route_Point == RoutePoints_COUNT) Current_Route_Point = 0;
				if (Num_Route_Point_DONE == total_NUM_Points_Route) 
				{
					MODE_CONTROL = STOP_AUTO;
				}
				_curse = NO_MOVEMENT;
			}
			else
			{
				Navesnoe_ready = 0;
				_curse = NO_MOVEMENT;
			}
			
			if (check_zone > 400)
			{
				joy_y_set = 80;
			}
			else
			{
				joy_y_set = check_zone / 4;
				if (joy_y_set > 80) joy_y_set = 80;
				if (joy_y_set < 20) joy_y_set = 20;
			}
			
			switch (_curse)                                       // Вычисление значений ШИМ для гусениц
			{
				case NO_MOVEMENT:
						joy_x_set = 0;
						joy_y_set = 0;
						Set_Left_Joystick_poti(joy_y_set, joy_x_set);
						turn_in_place = 1;
				break;

				case RIGHT_DRIVE:
						if (!turn_in_place)
						{
							joy_x_set = DELTA_ANGLE_btw_CURSE_N_HEADING * 10;
							if (joy_x_set > 60) joy_x_set = 60;
							if (joy_x_set < 20) joy_x_set = 20;
							
							Set_Left_Joystick_poti(joy_y_set, (i8)joy_x_set);
						}
						else if (turn_step == 0) 
						{
							joy_y_set = 0;
							joy_x_set = 100;
							Set_Left_Joystick_poti(joy_y_set, joy_x_set);
							turn_step = 1;
							osDelay(200);
						}
						else
						{
							joy_x_set = 100;
							joy_y_set = DELTA_ANGLE_btw_CURSE_N_HEADING * 5;
							if (joy_y_set > 60) joy_y_set = 60;
							if (joy_y_set < 30) joy_y_set = 30;
							Set_Left_Joystick_poti((i8)joy_y_set, joy_x_set);
						}
				break;

				case LEFT_DRIVE:
						if (!turn_in_place)
						{
							joy_x_set = DELTA_ANGLE_btw_CURSE_N_HEADING * (-10);
							if (joy_x_set < -60) joy_x_set = -60;
							if (joy_x_set > -20) joy_x_set = -20;
							
							Set_Left_Joystick_poti(joy_y_set, (i8)joy_x_set);
						}
						else if (turn_step == 0)
						{
							joy_y_set = 0;
							joy_x_set = -100;
							Set_Left_Joystick_poti(joy_y_set, joy_x_set);
							turn_step = 1;
							osDelay(200);
						}
						else
						{
							joy_x_set = -100;
							joy_y_set = DELTA_ANGLE_btw_CURSE_N_HEADING * 5;
							if (joy_y_set > 60) joy_y_set = 60;
							if (joy_y_set < 30) joy_y_set = 30;
							Set_Left_Joystick_poti((i8)joy_y_set, joy_x_set);
						}
				break;

				case FORWARD_DRIVE:
							if (Navesnoe_step == 4)
							{
									Navesnoe_ready = 0;
									timeout_navesnoe = xTaskGetTickCount();
							}
							else
							{
									joy_x_set = 0;
									Set_Left_Joystick_poti(joy_y_set, joy_x_set);
									turn_in_place = 0;
									turn_step = 0;
							}
				
							
									
				break;
				
				case BACKWARD_DRIVE:
					
				break;
			}

			CAN_BASKET_RC_DRIVING[4] = 0xFF;
			CAN_BASKET_RC_DRIVING[5] = left_btn;
			CAN_BASKET_RC_DRIVING[6] = 0x00;
			CAN_BASKET_RC_DRIVING[7] = 0xFF;
			
			CAN_BASKET_RC_DIR[0] = STOP_DRIVE;
			CAN_BASKET_RC_DIR[1] = STOP_DRIVE;
			CAN_BASKET_RC_DIR[2] = STOP_DRIVE;
			CAN_BASKET_RC_DIR[3] = STOP_DRIVE;
			CAN_BASKET_RC_DIR[4] = 0xFF;
			CAN_BASKET_RC_DIR[5] = right_btn;
			CAN_BASKET_RC_DIR[6] = 0x00;
			CAN_BASKET_RC_DIR[7] = 0xFF;
			
			CAN_BASKET_RC_SPEED_ENGINE[0] = 0xFF;
			CAN_BASKET_RC_SPEED_ENGINE[1] = 0xFF;
			CAN_BASKET_RC_SPEED_ENGINE[2] = 0xFF;
			CAN_BASKET_RC_SPEED_ENGINE[3] = speed_level;
			CAN_BASKET_RC_SPEED_ENGINE[4] = 0xFF;
//			CAN_BASKET_RC_SPEED_ENGINE[5] = RC_AUTO << CONTROL_TYPE | SICK_ACTIVATED << SICK_DEFINITION;; 	// Идентификатор автономного управления 0 bit [0 - ручное, 1 - автономное]
				CAN_BASKET_RC_SPEED_ENGINE[5] = 0xff;																							// Игнорирования препятствия			1 bit [0 - блокировавка, 1 - игнор]
			CAN_BASKET_RC_SPEED_ENGINE[6] = 0x00;
			CAN_BASKET_RC_SPEED_ENGINE[7] = 0x01;

		}
	}
	else
	{
		switch(Navesnoe_step)
		{
			case 0: right_btn = 0x10;
							Navesnoe_step++;
							timeout_navesnoe = xTaskGetTickCount();
			break;
			
			case 1:
							if (xTaskGetTickCount() - timeout_navesnoe > 1000)
							{
								right_btn = 0x00;
								Navesnoe_step++;
								timeout_navesnoe = xTaskGetTickCount();
							}
			break;
							
			case 2:
							Set_Right_Joystick_poti(-100, 0);
							if (xTaskGetTickCount() - timeout_navesnoe > 8000)
							{
								Navesnoe_step++;
								timeout_navesnoe = xTaskGetTickCount();
							}
			break;
							
			case 3:
							Set_Right_Joystick_poti(0, 0);
							Navesnoe_ready = 1;
							Navesnoe_step++;
			break;
			
			case 4:
							Set_Right_Joystick_poti(100, 0);
							if (xTaskGetTickCount() - timeout_navesnoe > 1000)
							{
								right_btn = 0x08;
								Navesnoe_step++;
								timeout_navesnoe = xTaskGetTickCount();
							}
			break;
							
			case 5:
			if (xTaskGetTickCount() - timeout_navesnoe > 5000)
			{
				right_btn = 0x00;
				Navesnoe_step++;
				timeout_navesnoe = xTaskGetTickCount();
			}
			break;
							
			case 6:
							Set_Right_Joystick_poti(0, 0);
							right_btn = 0x10;
							Navesnoe_step++;
							timeout_navesnoe = xTaskGetTickCount();
			break;
			
			case 7:
							if (xTaskGetTickCount() - timeout_navesnoe > 1000)
							{
								right_btn = 0x00;
								Navesnoe_step = 0;
								Navesnoe_ready = 1;
							}
			break;
			default:
			break;
				
		}
		CAN_BASKET_RC_DRIVING[0] = 0x00;    	
		CAN_BASKET_RC_DRIVING[1] = 0x00;    	
		CAN_BASKET_RC_DRIVING[2] = 0x00;    	
		CAN_BASKET_RC_DRIVING[3] = 0x00;    	                    
		CAN_BASKET_RC_DRIVING[4] = 0xFF;        	
		CAN_BASKET_RC_DRIVING[5] = 0x00;    	
		CAN_BASKET_RC_DRIVING[6] = 0x00;        	
		CAN_BASKET_RC_DRIVING[7] = 0xFF;        	
					    
		CAN_BASKET_RC_DIR[4] = 0xFF;        
		CAN_BASKET_RC_DIR[5] = right_btn;
		CAN_BASKET_RC_DIR[6] = 0x00;        
		CAN_BASKET_RC_DIR[7] = 0xFF;
		
		CAN_BASKET_RC_SPEED_ENGINE[0] = 0xFF;
		CAN_BASKET_RC_SPEED_ENGINE[1] = 0xFF;
		CAN_BASKET_RC_SPEED_ENGINE[2] = 0xFF;
		CAN_BASKET_RC_SPEED_ENGINE[3] = speed_level;
		CAN_BASKET_RC_SPEED_ENGINE[4] = 0xFF;
//		CAN_BASKET_RC_SPEED_ENGINE[5] = RC_AUTO << CONTROL_TYPE | SICK_ACTIVATED << SICK_DEFINITION;; 	// Идентификатор автономного управления 0 bit [0 - ручное, 1 - автономное]
		CAN_BASKET_RC_SPEED_ENGINE[5] = 0xFF;																																																// Игнорирования препятствия			1 bit [0 - блокировавка, 1 - игнор]
		CAN_BASKET_RC_SPEED_ENGINE[6] = 0x00;
		CAN_BASKET_RC_SPEED_ENGINE[7] = 0x01;
	}
}
u8 btn_cnt = 0;
time_t joy_time = 0;
time_t joy_btn_time = 0;
void STOP_MOVING(void)
{
	CAN_BASKET_RC_DRIVING[0] = 0;
	CAN_BASKET_RC_DRIVING[1] = 0;
	CAN_BASKET_RC_DRIVING[2] = 0;
	CAN_BASKET_RC_DRIVING[3] = 0;
	CAN_BASKET_RC_DRIVING[4] = 0xFF;
	CAN_BASKET_RC_DRIVING[5] = 0;
	CAN_BASKET_RC_DRIVING[6] = 0;
	CAN_BASKET_RC_DRIVING[7] = 0xFF;
	
	CAN_BASKET_RC_DIR[0] = 0;
	CAN_BASKET_RC_DIR[1] = 0;
	CAN_BASKET_RC_DIR[2] = 0;
	CAN_BASKET_RC_DIR[3] = 0;
	CAN_BASKET_RC_DIR[4] = 0xFF;
	CAN_BASKET_RC_DIR[5] = 0;
	CAN_BASKET_RC_DIR[6] = 0;
	CAN_BASKET_RC_DIR[7] = 0xFF;

	SPEED_ENGINE = 0;
	CAN_BASKET_RC_SPEED_ENGINE[0] = 0xFF;
	CAN_BASKET_RC_SPEED_ENGINE[1] = 0xFF;
	CAN_BASKET_RC_SPEED_ENGINE[2] = 0xFF;
	CAN_BASKET_RC_SPEED_ENGINE[3] = speed_level;
	CAN_BASKET_RC_SPEED_ENGINE[4] = 0xFF;
	CAN_BASKET_RC_SPEED_ENGINE[5] = 0xFF;
	
	CAN_BASKET_RC_SPEED_ENGINE[6] = 0;
	
	CAN_BASKET_RC_SPEED_ENGINE[7] = 0x01;
}

u8 				parsed_buffer					[255];					// Буфер для расшифрованных данных от приложения
u8 				xls_protocol_buffer				[20];					// Буфер для расшифрования данных согласно протоколоу
u8				received_led = 0;
u8				transmit_led = 0;
void getting_data(void)
{
	QUEUE_t msg;
	u8 confirm = 0;
	
	clear_RXBuffer(parsed_buffer, 255);
	while(parcel_count)
	{
//		GET_DATA_PARCEL(RECEIVE_SERVER_BUF, parsed_buffer);
		GET_DATA_PARCEL(RECEIVE_SERVER_BUF, parsed_buffer);
		
		confirm = 0;
		if (parsed_buffer[0] == '$')
		{
			switch(parsed_buffer[1])
			{
				case (u8)PACK_ID_tNum: 			Parse_To_Total_Num_Coords(parsed_buffer);
												confirm = 1;
				break;
				case (u8)PACK_ID_Coords: 		Parse_To_GPS_Points(parsed_buffer);
												confirm = 1;
				break;	
				case (u8)PACK_ID_Mode: 			Parse_To_Mode(parsed_buffer);
												confirm = 1;
				break;
				case (u8)PACK_ID_RC: 			confirm = 0;
												if (parsed_buffer[2] == PARCEL_ID_RC) 	
													Parse_To_RC_Command(parsed_buffer);
												else
												{
													Parce_To_CAN_Command(parsed_buffer);
													confirm = 1;
												}
												
				break;
				case (u8)PACK_ID_Err: 			Parse_To_Error(parsed_buffer);
												confirm = 1;
				break;
												
				case (u8)PACK_ID_Confirm:		Parse_To_Confirm(parsed_buffer);
												
												confirm = 0;
//												queue_cnt++;
				break;
												
				case (u8)PACK_ID_Request:
												if (parsed_buffer[3] == PARCEL_ID_Request_Coords)
												{
													SENDING_COORDS();
												}
												
												if (parsed_buffer[3] == PARCEL_ID_Request_CAN)
												{
													SENDING_CAN();
												}
				break;
			}
			
//			if (queue_init) queue_cnt = 1;
			if (confirm) 
			{
				GET_CONFIRM_MSG(parsed_buffer, msg.data);
				xQueueSend(SendQueueHandle, &msg.data, osWaitForever);
				clear_RXBuffer(parsed_buffer, 255);
			}
			Application_get_timer = xTaskGetTickCount();
			received_led = 1;
//			else queue_cnt++;
			
//			if (queue_cnt == 4) queue_cnt = 0;
		}
		
		parcel_count--;
	}
	clear_RXBuffer((u8 *)RADIO_DATA1, 30);
	clear_RXBuffer((u8 *)RECEIVE_SERVER_BUF, SERVER_BUF);
	SERVER_BUF_CNT = 0;

}

u8 GET_DATA_PARCEL(char *xBuf, u8 *result_mas1)
{
	
	u16 inc = 0, ti = 0, threshold = 0;
	u8 pack_length1 = 0;
	
	u8 dollar_flag1 = 0;
	u8 crc_calc = 0;
	u8 crc_received = 0;
	clear_RXBuffer(xls_protocol_buffer, 20);
	
	for(ti = 0; ti < RADIO_BUF; ti++)
	{
		if (xBuf[ti] == '$') 
		{
			dollar_flag1 = 1;
			threshold = ti;
			break;
		}
	}
	
	if (ti == SERVER_BUF)
	{		
		return ERROR;
	}
		
	switch (xBuf[threshold + 1])
	{
		case (u8)PACK_ID_tNum: pack_length1 = PACK_LEN_tNum;
		break;
		case (u8)PACK_ID_Coords: pack_length1 = PACK_LEN_Coords;
		break;
		case (u8)PACK_ID_Mode: pack_length1 = PACK_LEN_Mode;
		break;
		case (u8)PACK_ID_RC: pack_length1 = PACK_LEN_RC;
		break;
		case (u8)PACK_ID_Err: pack_length1 = PACK_LEN_Err;
		break;
		case (u8)PACK_ID_Confirm: pack_length1 = PACK_LEN_ALIVE;
		break;
		case (u8)PACK_ID_Request: pack_length1 = PACK_LEN_Request;
		break;
	
		
		default: pack_length1 = 0;
	}
	xBuf[threshold] = 0;
	for (inc = 0; inc < pack_length1; inc++)
	{
		xls_protocol_buffer[inc] = xBuf[threshold + inc + 1];
		xBuf[threshold + inc + 1] = 0;
	}
	if (dollar_flag1)
	{
		crc_received = xls_protocol_buffer[pack_length1 - 1];
		crc_calc = crc8(xls_protocol_buffer, pack_length1 - 1);
		
		if (crc_calc == crc_received) 
		{
			result_mas1[0] = 0x24;
			for(inc = 0; inc < pack_length1; inc++)
			{
				result_mas1[inc+1] = xls_protocol_buffer[inc];
			}
			return pack_length1 + threshold + 1;
		}
		else return ERROR;
	}
	else return ERROR;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* USER CODE BEGIN 1 */
	
	/* Смещение таблицы векторов */
//	SCB->VTOR = FLASH_BASE | 0x00004000U;
	clear_RXBuffer((u8 *)RADIO_DATA1, 30);
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
	MX_CAN1_Init();
	MX_CAN2_Init();
	MX_SPI1_Init();
	MX_UART4_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_DMA_Init();
	MX_USART6_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_UARTExCOM1_GPS_ReceiveToIdle_DMA();
	HAL_UARTExCOM2_GPS_ReceiveToIdle_IT();
	HAL_UARTExRadio_ReceiveToIdle_IT();

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
	
	#ifdef CONTROL_BLOCK
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

	/* creation of myLedTask */
	myLedTaskHandle = osThreadNew(StartLedTask, NULL, &myLedTask_attributes);
	
	/* creation of myGsmTask */
	myRadioTaskHandle = osThreadNew(StartRadioTask, NULL, &myRadioTask_attributes);

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
	#endif
	
	#ifdef ROVER_BLOCK
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
	
	/* creation of myGpsTask */
	myGpsTaskHandle = osThreadNew(StartGpsTask, NULL, &myGpsTask_attributes);

	/* creation of myLedTask */
	myLedTaskHandle = osThreadNew(StartLedTask, NULL, &myLedTask_attributes);
	
	/* creation of mySysTickTask */
	mySysTickTaskHandle = osThreadNew(StartmySysTickTask, NULL, &mySysTickTask_attributes);
	
	/* creation of myUsartTask */
	myUsartTaskHandle = osThreadNew(StartUsartTask, NULL, &myUsartTask_attributes);
	#endif

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
	hcan1.Init.Prescaler = 12;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = ENABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = ENABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CAN1_Init 2 */

	/////////////////// Filter CAN 1 ////////////////////////
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;

	if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);
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
	hcan2.Init.AutoBusOff = ENABLE;
	hcan2.Init.AutoWakeUp = DISABLE;
	hcan2.Init.AutoRetransmission = DISABLE;
	hcan2.Init.ReceiveFifoLocked = DISABLE;
	hcan2.Init.TransmitFifoPriority = ENABLE;
	if (HAL_CAN_Init(&hcan2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CAN2_Init 2 */

	sFilterConfig.FilterBank = 14;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if(HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
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
	huart4.Init.BaudRate = 19200;
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
	huart1.Init.BaudRate = 19200;
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
	huart3.Init.BaudRate = 19200;
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
	huart6.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	/* DMA2_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
	/* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
	HAL_GPIO_WritePin(GPIOC, CS_PIN_Pin|POWER_KEY_Pin, GPIO_PIN_RESET);

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

	/*Configure GPIO pins : CS_PIN_Pin POWER_KEY_Pin */
	GPIO_InitStruct.Pin = CS_PIN_Pin|POWER_KEY_Pin;
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
u8 buf1[8] = {1, 2, 3, 4, 5, 6, 7, 8};
u8 buf2[8] = {8, 7, 6, 5, 4, 3, 2, 1};


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
time_t timer_CAN_SEND;
uint64_t mulLat;
uint64_t mulLon;
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
	/* init code for USB_DEVICE */
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 5 */

	/* Infinite loop */
	for(;;)
	{
			if (xTaskGetTickCount() - timer_CAN_SEND >= 96)
			{
				CAN_Send(&hcan1, CAN_BASKET_RC_DRIVING, 		CAN_Remote_Control_A_ID);
				CAN_Send(&hcan1, CAN_BASKET_RC_DIR, 			CAN_Remote_Control_B_ID);
				CAN_Send(&hcan1, CAN_BASKET_RC_SPEED_ENGINE, 	CAN_Remote_Control_C_ID);
				CAN_Send(&hcan1, CAN_BASKET_ALT, 				CAN_ALT_ID);
				
				timer_CAN_SEND = xTaskGetTickCount();
			}
			
			if (RC_BTN_SEND)
			{
				CAN_Send(&hcan1, CAN_BASKET_RC_START_BUTTON, 	CAN_Remote_Control_sButton_ID);
				RC_BTN_SEND = 0;
			}
			
		osDelay(1);	
		
		
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
	u32 led1_on_timer = xTaskGetTickCount();
	u32 led0_on_timer = xTaskGetTickCount();
	
	u32 led1_off_timer = xTaskGetTickCount();
	u32 led0_off_timer = xTaskGetTickCount();
	
	u32 led1_blink_time = rand()%1000;
	u32 led0_blink_time = rand()%1000;
	
	u32 led1_blofk_time = rand()%100;
	u32 led0_blofk_time = rand()%100;
	
	/* Infinite loop */
	for(;;)
	{
//		if (xTaskGetTickCount() - led1_on_timer > led1_blink_time)
//		{
//			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
//			if (xTaskGetTickCount() - led1_off_timer > led1_blofk_time)
//			{
//				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
//				led1_blink_time = rand()%400;
//				led1_blofk_time = rand()%100;
//				led1_on_timer = xTaskGetTickCount();
//			}
//		}
//		else led1_off_timer = xTaskGetTickCount();
		if (REMOTE_CONNECT_OK)
		{
			if (transmit_led)
			{
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				if (xTaskGetTickCount() - led1_off_timer > 50)
				{
					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
					transmit_led = 0;
				}
			}
			else led1_off_timer = xTaskGetTickCount();
			
			if (received_led)
			{
				
					HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
					if (xTaskGetTickCount() - led0_off_timer > 50)
					{
						HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
						received_led = 0;
					}
			}
			else led0_off_timer = xTaskGetTickCount();
		}
		else
		{
			if (xTaskGetTickCount() - led1_on_timer > led1_blink_time)
			{
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				if (xTaskGetTickCount() - led1_off_timer > led1_blofk_time)
				{
					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
					led1_blink_time = rand()%400;
					led1_blofk_time = rand()%100;
					led1_on_timer = xTaskGetTickCount();
				}
			}
		else 
			led1_off_timer = xTaskGetTickCount();
		
			if (xTaskGetTickCount() - led0_on_timer > led0_blink_time)
			{
				HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
				if (xTaskGetTickCount() - led0_off_timer > led0_blofk_time)
				{
					HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
					led0_blink_time = rand()%400;
					led0_blofk_time = rand()%100;
					led0_on_timer = xTaskGetTickCount();
				}
			}
			else led0_off_timer = xTaskGetTickCount();
			
		}
		
		osDelay(10);

	}
	/* USER CODE END StartLedTask */
}

/* USER CODE BEGIN StartRadioTask */
/**
* @brief Function implementing the myGsmTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END StartRadioTask */
void StartRadioTask(void *argument)
{
	/* USER CODE BEGIN StartRadioTask */
	/* Infinite loop */
	for(;;)
	{
		if (incData)
		{
			incData = 0;
			getting_data();
			
		}
		else osDelay(10);

	  
	}
	/* USER CODE END StartRadioTask */
}

/* USER CODE BEGIN Header_StartGpsTask */
/**
* @brief Function implementing the myGpsTask thread.
* @param argument: Not used
* @retval None
*/
time_t	gps_data_timer = 0;
/* USER CODE END Header_StartGpsTask */
void StartGpsTask(void *argument)
{
	/* USER CODE BEGIN StartGpsTask */
	/* Infinite loop */
	for(;;)
	{
		if (GPGGA)												// Если принята 
		{
			Parse_To_NMEA();
			GPGGA = 0;
			gps_data_timer = xTaskGetTickCount();
		}
		
		
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
size_t freemem = 0;
time_t delta_timeout_gps = 0;
/* USER CODE END Header_StartmySysTickTask */
void StartmySysTickTask(void *argument)
{
	/* USER CODE BEGIN StartmySysTickTask */
	/* Infinite loop */
	for(;;)
	{
		freemem = xPortGetFreeHeapSize();

		delta_timeout_Can_coords = xTaskGetTickCount() - timer_CAN_coords;
		delta_timeout_gps = xTaskGetTickCount() - gps_data_timer;
		if (delta_timeout_gps > 5000)
		{
			MASTER_coord_latitude = 0;
			MASTER_coord_longitude = 0;
		}
		
		if (delta_timeout_Can_coords > 1000)
		{
			SLAVE_coord_latitude = 0;
			SLAVE_coord_longitude = 0;
		}
								
		if ((USART1->CR1 & 0x30) != 0x30)
		{
			HAL_UARTExRadio_ReceiveToIdle_IT();
		}
		
		if (__HAL_UART_GET_FLAG(UART_COM2_GPS_HANDLE, UART_FLAG_ORE) != RESET)
		{
			__HAL_UART_CLEAR_OREFLAG(UART_COM2_GPS_HANDLE);
			huart2.ErrorCode = 0;

			HAL_UARTExCOM2_GPS_ReceiveToIdle_IT();
		}
		
		if (__HAL_UART_GET_FLAG(UART_RADIO_HANDLE, UART_FLAG_ORE) != RESET)
		{
			__HAL_UART_CLEAR_OREFLAG(UART_RADIO_HANDLE);
			huart1.ErrorCode = 0;
			
			HAL_UARTExRadio_ReceiveToIdle_IT();
		}
		
		if (__HAL_UART_GET_FLAG(UART_COM1_GPS_HANDLE, UART_FLAG_ORE) != RESET)
		{
			__HAL_UART_CLEAR_OREFLAG(UART_COM1_GPS_HANDLE);
			huart6.ErrorCode = 0;
			
			HAL_UARTExCOM1_GPS_ReceiveToIdle_DMA();

		}
		
		if (xTaskGetTickCount() - Application_get_timer < 5000) REMOTE_CONNECT_OK = 1;
		else 
		{
			REMOTE_CONNECT_OK = 0;
			speed_level = 0;
		}
		
		delta_timeout_RC = xTaskGetTickCount() - RC_timer;
		if ((MODE_CONTROL == MANUAL_CONTROL_ON && delta_timeout_RC > 10000))
		{
			MODE_CONTROL = IDLE_MODE;
		}
			
			
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
//		if (queue_cnt == 0)
//		{
//			SENDING_COORDS();
//			queue_cnt++;			
//		}
		
//		if (queue_init)
		osDelay(1000);
//		else
//			osDelay(1);
		
	}
	/* USER CODE END StartmyMobileApplicationTask */
}

/* USER CODE BEGIN Header_StartmySendingServerDataTask */
/**
* @brief Function implementing the mySendingServer thread.
* @param argument: Not used
* @retval None
*/
u8 SEND_OK = 1;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
          if(huart == UART_RADIO_HANDLE)
          {
             SEND_OK = 1;     // можно установить какой-то флаг, сообщающий об окончании отправки
			 transmit_led = 1;
          }     
}

UBaseType_t uxNumberOfItems = 0;
/* USER CODE END Header_StartmySendingServerDataTask */
void StartmySendingServerDataTask(void *argument)
{
	/* USER CODE BEGIN StartmySendingServerDataTask */
	QUEUE_t msg;
	clear_RXBuffer(global_buf, PACK_LEN_dozerParams * 19);
	/* Infinite loop */
	for(;;)
	{
		uxNumberOfItems = uxQueueMessagesWaiting(SendQueueHandle);
//		if (xQueueReceive(SendQueueHandle, &msg, 0) == pdTRUE && REMOTE_CONNECT_OK)
		if (xQueueReceive(SendQueueHandle, &msg, 0) == pdTRUE)
		{
			if (msg.data[1] == PACK_ID_dozerParams)
			{
				while(!SEND_OK) osDelay(50);
				HAL_UART_Transmit_IT(UART_RADIO_HANDLE, global_buf, PACK_LEN_dozerParams * 19);
				SEND_OK = 0;
				
			}
			
						
			if (msg.data[1] == PACK_ID_myCoords)
			{
				while(!SEND_OK) osDelay(50);
				HAL_UART_Transmit_IT(UART_RADIO_HANDLE, msg.data, PACK_LEN_myCoords);
//				HAL_UART_Transmit_IT(UART_RADIO_HANDLE, "HELLO\r\n", 7);
				SEND_OK = 0;
			}
			
			if (msg.data[1] == PACK_ID_Confirm || msg.data[1] == PACK_ID_Err)
			{
				while(!SEND_OK) osDelay(50);
				HAL_UART_Transmit_IT(UART_RADIO_HANDLE, msg.data, PACK_LEN_Confirm);
				SEND_OK = 0;
			}
			
			if (msg.data[1] == PACK_ID_CompletePoint)
			{
				while(!SEND_OK) osDelay(50);
				HAL_UART_Transmit_IT(UART_RADIO_HANDLE, msg.data, PACK_LEN_CompletePoint);
				SEND_OK = 0;
			}
			
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
		HEADING = Azimuth_Calculating(SLAVE_coord_latitude, SLAVE_coord_longitude, MASTER_coord_latitude, MASTER_coord_longitude);
		HEADING = filter(HEADING);
		switch (MODE_CONTROL)
		{
			case MANUAL_CONTROL_ON:		run_RC_drive();			
			break;
			
			
			case MANUAL_CONTROL_OFF:	MODE_CONTROL = IDLE_MODE;			
			break;
			
			
			case START_AUTO:			run_gps_drive();			
			break;
			
			
			case PAUSE_AUTO:			STOP_MOVING();
										osDelay(100);
			break;
			
			
			case RESUME_AUTO:			MODE_CONTROL = START_AUTO;			
			break;
			
			
			case STOP_AUTO:
										Num_Route_Point_DONE = 0;
										Current_Route_Point = 0;
										total_NUM_Points_Route = 0;
										TARGET_coord_latitude = 0;
										TARGET_coord_longitude = 0;
										ANGLE_btw_TARGET_N_DOZER = 0;
										HEADING = 0;
										turn_in_place = 1;
										_curse = NO_MOVEMENT;
			
										for(uint8_t i = 0; i < RoutePoints_COUNT; i++)
										{
											coordRoute[i].latitude = 0;
											coordRoute[i].longitude = 0;
										}
										
										Navesnoe_ready = 0;
										Navesnoe_step = 0;

																				
										MODE_CONTROL = IDLE_MODE;
			break;
										
			
			case DISCONNECT_RC:			osDelay(100);
										
			break;
										
			
			case IDLE_MODE: 			STOP_MOVING();
										osDelay(100);
			break;
		}
		
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
		if(USB_flag && (xTaskGetTickCount() - USB_timer > 10))
		{
			USB_Cplt = 1;
			if (strstr((const char *)USB_Buf, "$GPS")) 
			{
				virtual_gps = 1;
				virtual_bt = 0;
				virtual_surv = 0;
				USB_Cplt = 0;
				USB_cnt = 0;
			}
			
			if (strstr((const char *)USB_Buf, "$BT"))
			{
				virtual_gps = 0;
				virtual_bt = 1;
				virtual_surv = 0;
				USB_Cplt = 0;
				USB_cnt = 0;
			}
			
			if (strstr((const char *)USB_Buf, "$OFF"))
			{
				virtual_gps = 0;
				virtual_bt = 0;
				virtual_surv = 0;
				USB_Cplt = 0;
				USB_cnt = 0;
			}
			
			if (strstr((const char *)USB_Buf, "$SURV"))
			{
				virtual_gps = 0;
				virtual_bt = 0;
				virtual_surv = 1;
				USB_Cplt = 0;
				USB_cnt = 0;
			}
			if (USB_Buf[0] == '/') next = 1;
			
			
			USB_flag = 0;
			
		}
		
		if (USB_Cplt) 
		{
			if (virtual_gps)
			HAL_UART_Transmit_IT(UART_COM1_GPS_HANDLE, USB_Buf,USB_cnt);

			if (virtual_bt)
			HAL_UART_Transmit_IT(UART_RADIO_HANDLE, USB_Buf,USB_cnt);	
			USB_Cplt = 0;
			USB_cnt = 0;
		}


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
		HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
		HAL_Delay(50);
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

