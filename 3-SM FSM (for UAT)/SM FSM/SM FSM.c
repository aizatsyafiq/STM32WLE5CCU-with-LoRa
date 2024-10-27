/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "i2c.h"
#include "subghz.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "radio_driver.h"
#include "bme280.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	STATE_NULL,
	STATE_MASTER,
	STATE_SLAVE
} state_t;

typedef enum
{
	SSTATE_NULL,
	SSTATE_RX,
	SSTATE_TX
} substate_t;

typedef struct
{
	state_t state;
	substate_t subState;
	uint32_t rxTimeout;
	uint32_t rxMargin;
	uint32_t randomDelay;
	char rxBuffer[RX_BUFFER_SIZE];
	uint8_t rxSize;
} pingPongFSM_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RF_FREQUENCY                                400000000 /* Hz */ //27222
#define TX_OUTPUT_POWER                             20	       /* dBm */ //use 12 max for new design?
#define LORA_BANDWIDTH                              4         /* kHz */
#define LORA_SPREADING_FACTOR                       10
#define LORA_CODINGRATE                             1
#define LORA_PREAMBLE_LENGTH                        8         /* Same for Tx and Rx */
#define LORA_SYMBOL_TIMEOUT                         5         /* Symbols */

#define TXRX_ENABLED								0	/* 1 = enable Tx/Rx in while loop. 0 = disable any Tx and Rx */
#define TRANSMIT_ONLY								1	/* 1 = set to transmit only. 0 = ping pong */
#define RECEIVE_ONLY								1	/* 1 = set to receive only. 0 = ping pong */
#define FIRMWARE_FILENAME							"CM_WLE55CCU6_2.bin"
#define FIRMWARE_VERSION							"0.1"
#define PAYLOADLENGTH								20

#define BME280_CONNECTED							0
#define RS485_CONNECTED								0
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
void (*volatile eventReceptor)(pingPongFSM_t *const fsm);
PacketParams_t packetParams;
SUBGRF_Status_t status;
const RadioLoRaBandwidths_t Bandwidths[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500, LORA_BW_015, LORA_BW_031, LORA_BW_062 };
uint8_t counter = 0;

//BME280 var
float temperature;		//change float to something else later
float humidity;
float pressure;

struct bme280_dev dev;
struct bme280_data comp_data;
int8_t rslt = 0;

char temp_string[50];
char hum_string[50];
char pres_string[50];

//RS485
/*
 * RO - UART RX
 * DI - UART TX
 * DE & RE - shorted - GPIO output
 */
uint8_t TxData[16];
uint8_t RxData[16];
int indx = 0;

bool SMreadyToTx = false; //flag for SM Tx mode. 1 = received cmd from gs, sm move into tx mode.

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void radioInit(void);
void RadioOnDioIrq(RadioIrqMasks_t radioIrq);
void eventTxDone(pingPongFSM_t *const fsm);
void eventRxDone(pingPongFSM_t *const fsm);
void eventTxTimeout(pingPongFSM_t *const fsm);
void eventRxTimeout(pingPongFSM_t *const fsm);
void eventRxError(pingPongFSM_t *const fsm);
void enterMasterRx(pingPongFSM_t *const fsm);
void enterSlaveRx(pingPongFSM_t *const fsm);
void enterMasterTx(pingPongFSM_t *const fsm);
void enterSlaveTx(pingPongFSM_t *const fsm);
void transitionRxDone(pingPongFSM_t *const fsm);
void blinkLED(void);
void printUid(UART_HandleTypeDef *huart);
void printBoardInfo(UART_HandleTypeDef *huart);
void checki2cInitialize(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);
void checkLoraInit(UART_HandleTypeDef *huart);
void startUpFWCheck(UART_HandleTypeDef *huart, I2C_HandleTypeDef *hi2c);
uint32_t GetBandwidthInHz(RadioLoRaBandwidths_t bandwidth);
void switchTransmitOn(void);
void switchTransmitOff(void);
void switchReceiveOn(void);
void switchReceiveOff(void);
void measureBME(UART_HandleTypeDef *huart);
void getBMEValueInHex(uint8_t *sensorData, uint8_t *dataSize, UART_HandleTypeDef *huart);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void sendData(uint8_t *data);
int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void user_delay_ms(uint32_t period);
int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	pingPongFSM_t fsm;
	char uartBuff[100];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	/*** GPIO Configuration (for debugging) ***/
	/* DEBUG_SUBGHZSPI_NSSOUT = PA4
	 * DEBUG_SUBGHZSPI_SCKOUT = PA5
	 * DEBUG_SUBGHZSPI_MISOOUT = PA6
	 * DEBUG_SUBGHZSPI_MOSIOUT = PA7
	 * DEBUG_RF_HSE32RDY = PA10
	 * DEBUG_RF_NRESET = PA11
	 * DEBUG_RF_SMPSRDY = PB2
	 * DEBUG_RF_DTB1 = PB3 <---- Conflicts with RF_IRQ0
	 * DEBUG_RF_LDORDY = PB4
	 * RF_BUSY = PA12
	 * RF_IRQ0 = PB3
	 * RF_IRQ1 = PB5
	 * RF_IRQ2 = PB8
	 */

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Enable GPIO Clocks
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	// DEBUG_SUBGHZSPI_{NSSOUT, SCKOUT, MSIOOUT, MOSIOUT} pins
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF13_DEBUG_SUBGHZSPI;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// DEBUG_RF_{HSE32RDY, NRESET} pins
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
	GPIO_InitStruct.Alternate = GPIO_AF13_DEBUG_RF;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// DEBUG_RF_{SMPSRDY, LDORDY} pins
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_4;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// RF_BUSY pin
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Alternate = GPIO_AF6_RF_BUSY;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// RF_{IRQ0, IRQ1, IRQ2} pins
	GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_8;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SUBGHZ_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	RadioLoRaBandwidths_t currentBandwidth = Bandwidths[LORA_BANDWIDTH];
	uint32_t bandwidthHz = GetBandwidthInHz(currentBandwidth);

	HAL_Delay(2000);

	sprintf(uartBuff, "LORA_MODULATION\r\nLORA_BW=%lu Hz\r\nLORA_SF=%d\r\n", bandwidthHz, LORA_SPREADING_FACTOR);
	HAL_UART_Transmit(&huart1, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);
	radioInit();

#if RS485_CONNECTED == 1
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 16);
#endif

#if BME280_CONNECTED == 1
	/* BME280 init */
	dev.dev_id = BME280_I2C_ADDR_SEC;
	dev.intf = BME280_I2C_INTF;
	dev.read = user_i2c_read;
	dev.write = user_i2c_write;
	dev.delay_ms = user_delay_ms;

	rslt = bme280_init(&dev);
	//error check
	if(rslt != BME280_OK){
		HAL_UART_Transmit(&huart1, (uint8_t *)rslt, sizeof(rslt), HAL_MAX_DELAY);
	}

	/* BME280 settings */
	dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	dev.settings.osr_p = BME280_OVERSAMPLING_16X;
	dev.settings.osr_t = BME280_OVERSAMPLING_2X;
	dev.settings.filter = BME280_FILTER_COEFF_16;
	rslt = bme280_set_sensor_settings(BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL, &dev);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//	uint32_t rnd = 0;
	SUBGRF_SetDioIrqParams(IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
//	rnd = SUBGRF_GetRandom();

	fsm.state = STATE_NULL;
	fsm.subState = SSTATE_NULL;
	fsm.rxTimeout = 3000; // 3000 ms
	fsm.rxMargin = 200;   // 200 ms
//	fsm.randomDelay = rnd >> 22; // [0, 1023] ms

//	HAL_Delay(fsm.randomDelay);
	SUBGRF_SetDioIrqParams( IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
			IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
			IRQ_RADIO_NONE,
			IRQ_RADIO_NONE );
//	SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
//	SUBGRF_SetRx(fsm.rxTimeout << 6);
//	fsm.state = STATE_MASTER;
//	fsm.subState = SSTATE_RX;

	//startUpFWCheck(&huart1, &hi2c1);
	blinkLED();

	while (1) //AAAAAAAAAAAAAAAA
	{
		enterMasterRx(&fsm);
		eventReceptor = NULL;
		while (eventReceptor == NULL);
		eventReceptor(&fsm);
		//if command ok - set flag
		if(SMreadyToTx){
			HAL_UART_Transmit(&huart1, (uint8_t *)"Tx MODE\r\n", 9, HAL_MAX_DELAY);
			switchTransmitOn();
			HAL_Delay(3000); //wait for gs to be in rx before transmitting packet
			SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX);
			SUBGRF_WriteRegister(0x0889, (SUBGRF_ReadRegister(0x0889) | 0x04));
			uint8_t payloadsize = 20; //20 byte as mock packet
			char payload[21] = "ABCDEFGHIJKLMNOPQRST\0"; //size is payloadsize + 1
			packetParams.Params.LoRa.PayloadLength = payloadsize;
			SUBGRF_SetPacketParams(&packetParams);
			SUBGRF_SendPayload((uint8_t *)payload, payloadsize, 0);
			HAL_UART_Transmit(&huart1, (uint8_t *)payload, payloadsize, HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
			HAL_Delay(2500);
			switchTransmitOff();
			HAL_UART_Transmit(&huart1, (uint8_t *)"Tx DONE\r\n\n", 10, HAL_MAX_DELAY);
			SMreadyToTx = false;
		}

#if RS485_CONNECTED == 1
		snprintf(TxData, "F103 %d", indx++);
		sendData(TxData);
		HAL_Delay(1000);
#endif

#if BME280_CONNECTED == 1
		measureBME(&huart1);
#endif
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEDiv = RCC_HSE_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief  Initialize the Sub-GHz radio and dependent hardware.
 * @retval None
 */
void radioInit(void)
{
	// Initialize the hardware (SPI bus, TCXO control, RF switch)
	status = SUBGRF_Init(RadioOnDioIrq);

	// Use DCDC converter if `DCDC_ENABLE` is defined in radio_conf.h
	// "By default, the SMPS clock detection is disabled and must be enabled before enabling the SMPS." (6.1 in RM0453)
	SUBGRF_WriteRegister(SUBGHZ_SMPSC0R, (SUBGRF_ReadRegister(SUBGHZ_SMPSC0R) | SMPS_CLK_DET_ENABLE));
	SUBGRF_SetRegulatorMode();

	// Use the whole 256-byte buffer for both TX and RX
	SUBGRF_SetBufferBaseAddress(0x00, 0x00);

	SUBGRF_SetRfFrequency(RF_FREQUENCY);
	SUBGRF_SetRfTxPower(TX_OUTPUT_POWER);
	SUBGRF_SetStopRxTimerOnPreambleDetect(false);

	SUBGRF_SetPacketType(PACKET_TYPE_LORA);

	SUBGRF_WriteRegister( REG_LR_SYNCWORD, ( LORA_MAC_PRIVATE_SYNCWORD >> 8 ) & 0xFF );
	SUBGRF_WriteRegister( REG_LR_SYNCWORD + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF );

	ModulationParams_t modulationParams;
	modulationParams.PacketType = PACKET_TYPE_LORA;
	modulationParams.Params.LoRa.Bandwidth = Bandwidths[LORA_BANDWIDTH];
	modulationParams.Params.LoRa.CodingRate = (RadioLoRaCodingRates_t)LORA_CODINGRATE;
	modulationParams.Params.LoRa.LowDatarateOptimize = 0x00;
	modulationParams.Params.LoRa.SpreadingFactor = (RadioLoRaSpreadingFactors_t)LORA_SPREADING_FACTOR;
	SUBGRF_SetModulationParams(&modulationParams);

	packetParams.PacketType = PACKET_TYPE_LORA;
	packetParams.Params.LoRa.CrcMode = LORA_CRC_OFF; //ON to OFF
	packetParams.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH; //variable to fixed
	packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
	packetParams.Params.LoRa.PayloadLength = PAYLOADLENGTH; //0xFF to 8
	packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
	SUBGRF_SetPacketParams(&packetParams);

	//SUBGRF_SetLoRaSymbNumTimeout(LORA_SYMBOL_TIMEOUT);

	// WORKAROUND - Optimizing the Inverted IQ Operation, see DS_SX1261-2_V1.2 datasheet chapter 15.4
	// RegIqPolaritySetup @address 0x0736
	SUBGRF_WriteRegister( 0x0736, SUBGRF_ReadRegister( 0x0736 ) | ( 1 << 2 ) );
}

/**
 * @brief  Receive data trough SUBGHZSPI peripheral
 * @param  radioIrq  interrupt pending status information
 * @retval None
 */
void RadioOnDioIrq(RadioIrqMasks_t radioIrq)
{
	switch (radioIrq)
	{
	case IRQ_TX_DONE:
		eventReceptor = eventTxDone;
		break;
	case IRQ_RX_DONE:
		eventReceptor = eventRxDone;
		break;
	case IRQ_RX_TX_TIMEOUT:
		if (SUBGRF_GetOperatingMode() == MODE_TX)
		{
			eventReceptor = eventTxTimeout;
		}
		else if (SUBGRF_GetOperatingMode() == MODE_RX)
		{
			eventReceptor = eventRxTimeout;
		}
		break;
	case IRQ_CRC_ERROR:
		eventReceptor = eventRxError;
		break;
	default:
		break;
	}
}

/**
 * @brief  Process the TX Done event
 * @param  fsm pointer to FSM context
 * @retval None
 */
void eventTxDone(pingPongFSM_t *const fsm)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)"Event TX Done\r\n", 15, HAL_MAX_DELAY);
}

/**
 * @brief  Process the RX Done event
 * @param  fsm pointer to FSM context
 * @retval None
 */
void eventRxDone(pingPongFSM_t *const fsm)
{
	switchReceiveOff();
	HAL_Delay(200);

	uint8_t payloadLength = 0;
	uint8_t rxStartBufferPointer = 0;
	char commandOne[2] = "A";
	//get command
	SUBGRF_GetRxBufferStatus(&payloadLength, &rxStartBufferPointer);
	//read command
	SUBGRF_ReadBuffer(rxStartBufferPointer, (uint8_t *)fsm->rxBuffer, payloadLength);
	//print command (for debug)
	HAL_UART_Transmit(&huart1, (uint8_t *)fsm->rxBuffer, 1, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);

	char* p = fsm->rxBuffer;
	if(strncmp(p, commandOne, 1) == 0){
		SMreadyToTx = true;
		HAL_UART_Transmit(&huart1, (uint8_t *)"SM received command to transmit 20 bytes to GS\r\n", 48, HAL_MAX_DELAY);
	}
}

/**
 * @brief  Process the TX Timeout event
 * @param  fsm pointer to FSM context
 * @retval None
 */
void eventTxTimeout(pingPongFSM_t *const fsm)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)"Event TX Timeout\r\n", 18, HAL_MAX_DELAY);
	switchTransmitOff();
}

/**
 * @brief  Process the RX Timeout event
 * @param  fsm pointer to FSM context
 * @retval None
 */
void eventRxTimeout(pingPongFSM_t *const fsm)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)"Event RX Timeout\r\n", 18, HAL_MAX_DELAY);
	switchReceiveOff();
}

/**
 * @brief  Process the RX Error event
 * @param  fsm pointer to FSM context
 * @retval None
 */
void eventRxError(pingPongFSM_t *const fsm)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)"Event Rx Error\r\n", 16, HAL_MAX_DELAY);
	switchReceiveOff();
}

/**
 * @brief  Entry actions for the RX sub-state of the Master state
 * @param  fsm pointer to FSM context
 * @retval None
 */
void enterMasterRx(pingPongFSM_t *const fsm)
{
	switchReceiveOn();
//	HAL_Delay(1000);
	SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);

	HAL_UART_Transmit(&huart1, (uint8_t *)"Rx MODE\r\n", 9, HAL_MAX_DELAY);
	SUBGRF_SetDioIrqParams( IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
			IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
			IRQ_RADIO_NONE,
			IRQ_RADIO_NONE );
	packetParams.Params.LoRa.PayloadLength = 1;
	SUBGRF_SetPacketParams(&packetParams);
	SUBGRF_SetRx(0xFFFFFF); //continuous
}

/**
 * @brief  Entry actions for the RX sub-state of the Slave state
 * @param  fsm pointer to FSM context
 * @retval None
 */
void enterSlaveRx(pingPongFSM_t *const fsm)
{
//
}

/**
 * @brief  Entry actions for the TX sub-state of the Master state
 * @param  fsm pointer to FSM context
 * @retval None
 */
void enterMasterTx(pingPongFSM_t *const fsm)
{
	//not used

//	switchTransmitOn();
//
//	HAL_UART_Transmit(&huart1, (uint8_t *)"Master Tx start\r\n", 17, HAL_MAX_DELAY);
//	SUBGRF_SetDioIrqParams( IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
//			IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
//			IRQ_RADIO_NONE,
//			IRQ_RADIO_NONE );
//	SUBGRF_SetSwitch(RFO_HP, RFSWITCH_TX);
//	// Workaround 5.1 in DS.SX1261-2.W.APP (before each packet transmission)
//	SUBGRF_WriteRegister(0x0889, (SUBGRF_ReadRegister(0x0889) | 0x04));
//	packetParams.Params.LoRa.PayloadLength = PAYLOADLENGTH;
//	SUBGRF_SetPacketParams(&packetParams);
//	SUBGRF_SendPayload((uint8_t *)"ABCDEFGHIJKLMNOPQRST", PAYLOADLENGTH, 0);
//
//	HAL_Delay(2500);
//	switchTransmitOff();
}

/**
 * @brief  Entry actions for the TX sub-state of the Slave state
 * @param  fsm pointer to FSM context
 * @retval None
 */
void enterSlaveTx(pingPongFSM_t *const fsm)
{
//
}


/**
 * @brief  Transition actions executed on every RX Done event (helper function)
 * @param  fsm pointer to FSM context
 * @retval None
 */
void transitionRxDone(pingPongFSM_t *const fsm)
{
#if TRANSMIT_ONLY == 0

	PacketStatus_t packetStatus;
	char uartBuff[50];

	// Workaround 15.3 in DS.SX1261-2.W.APP (because following RX w/ timeout sequence)
	SUBGRF_WriteRegister(0x0920, 0x00);
	SUBGRF_WriteRegister(0x0944, (SUBGRF_ReadRegister(0x0944) | 0x02));

	SUBGRF_GetPayload((uint8_t *)fsm->rxBuffer, &fsm->rxSize, 0xFF);
	SUBGRF_GetPacketStatus(&packetStatus);

	sprintf(uartBuff, "RssiValue=%d dBm, SnrValue=%d Hz\r\n", packetStatus.Params.LoRa.RssiPkt, packetStatus.Params.LoRa.SnrPkt);
	HAL_UART_Transmit(&huart1, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);

#endif
}

/**
 * @brief Blink LED
 */
void blinkLED(void){
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
	HAL_Delay(250);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
	HAL_Delay(250);
}

/**
 * @brief Check i2c Init
 */
void checki2cInitialize(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart)
{
	char buffer[39];
	if(HAL_I2C_GetState(hi2c) == HAL_I2C_STATE_READY){
		snprintf(buffer, sizeof(buffer),"I2C INIT..........................OK\r\n");
		HAL_UART_Transmit(huart, (uint8_t*)buffer, sizeof(buffer), HAL_MAX_DELAY);
	}else{
		snprintf(buffer, sizeof(buffer),"I2C INIT........................FAIL\r\n");
		HAL_UART_Transmit(huart, (uint8_t*)buffer, sizeof(buffer), HAL_MAX_DELAY);
	}
}

/**
 * @brief Check LoRa Init
 */
void checkLoraInit(UART_HandleTypeDef *huart)
{
	char buffer[39];

	if(status == SUBGRF_OK){
		snprintf(buffer, sizeof(buffer), "LoRa INIT.........................OK\r\n");
	}else{
		snprintf(buffer, sizeof(buffer), "LoRa INIT.......................FAIL\r\n");
	}
	HAL_UART_Transmit(huart, (uint8_t*)buffer, sizeof(buffer), HAL_MAX_DELAY);
}

/**
 * @brief Display info
 */
void printBoardInfo(UART_HandleTypeDef *huart)
{
	char buffer[50];
	printUid(huart);
	snprintf(buffer, sizeof(buffer), "FW NAME : %s\r\n", FIRMWARE_FILENAME);
	HAL_UART_Transmit(huart, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
	HAL_Delay(50);
	snprintf(buffer, sizeof(buffer), "FW VER : %s\r\n", FIRMWARE_VERSION);
	HAL_UART_Transmit(huart, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
	HAL_Delay(50);
	snprintf(buffer, sizeof(buffer), "FW DATE : %s\r\n", __DATE__);
	HAL_UART_Transmit(huart, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
	HAL_Delay(50);
}

/**
 * @brief Print unique ID
 */
void printUid(UART_HandleTypeDef *huart)
{
	uint32_t uid[3];
	char buffer[30];  // 8 chars for hex, 1 for '\r', 1 for '\n', 1 for null terminator

	uid[0] = HAL_GetUIDw0();
	uid[1] = HAL_GetUIDw1();
	uid[2] = HAL_GetUIDw2();

	//    for (int i = 0; i < 3; i++)
	//    {
	//        snprintf(buffer, sizeof(buffer), "%08lX\r\n", uid[i]);
	//        HAL_UART_Transmit(huart, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
	//    }

	//show only Device ID
	snprintf(buffer, sizeof(buffer), "UNIQUE ID : %08lX\r\n", uid[2]);
	HAL_UART_Transmit(huart, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/**
 * @brief FW startup check; compiled with other start checks
 */
void startUpFWCheck(UART_HandleTypeDef *huart, I2C_HandleTypeDef *hi2c)
{
	char buffer[49];
	//check FW name and version
	printBoardInfo(huart);
	//check i2c pins init
	checki2cInitialize(hi2c, huart);
	HAL_Delay(50);
	//check LoRa init
	checkLoraInit(huart);
	HAL_Delay(50);
	//check LED blinking 3 times
	snprintf(buffer, sizeof(buffer),"LED blinking 3 times..............(self-check)\r\n");
	HAL_UART_Transmit(huart, (uint8_t*)buffer, sizeof(buffer), HAL_MAX_DELAY);
	blinkLED();
	blinkLED();
	blinkLED();
}

/**
 * @brief Bandwidth selection for display ONLY
 */
uint32_t GetBandwidthInHz(RadioLoRaBandwidths_t bandwidth)
{
	switch(bandwidth)
	{
	case LORA_BW_500: return 500000;
	case LORA_BW_250: return 250000;
	case LORA_BW_125: return 125000;
	case LORA_BW_062: return 62500;
	case LORA_BW_041: return 41670;
	case LORA_BW_031: return 31250;
	case LORA_BW_020: return 20830;
	case LORA_BW_015: return 15630;
	case LORA_BW_010: return 10420;
	case LORA_BW_007: return 7810;
	default: return 0; // Invalid bandwidth
	}
}

/**
 * @brief RF switches for CM - Tx ON sequence
 */
void switchTransmitOn(void){
	HAL_GPIO_WritePin(RFSWEN_GPIO_Port, RFSWEN_Pin, 1);
	HAL_Delay(50);
	HAL_GPIO_WritePin(RFSel_GPIO_Port, RFSel_Pin, 1);
	HAL_Delay(50);
	HAL_GPIO_WritePin(Ven_PA_GPIO_Port, Ven_PA_Pin, 1);
	HAL_Delay(50);
}

/**
 * @brief RF switches for CM - Tx OFF sequence
 */
void switchTransmitOff(void){
	HAL_GPIO_WritePin(Ven_PA_GPIO_Port, Ven_PA_Pin, 0);
	HAL_Delay(50);
	HAL_GPIO_WritePin(RFSel_GPIO_Port, RFSel_Pin, 0);
	HAL_Delay(50);
	HAL_GPIO_WritePin(RFSWEN_GPIO_Port, RFSWEN_Pin, 0);
	HAL_Delay(50);
}

/**
 * @brief RF switch for CM - Rx ON sequence
 */
void switchReceiveOn(void){
	HAL_GPIO_WritePin(RFSWEN_GPIO_Port, RFSWEN_Pin, 1);
	HAL_Delay(50);
	HAL_GPIO_WritePin(RFSel_GPIO_Port, RFSel_Pin, 0);
	HAL_Delay(50);
	HAL_GPIO_WritePin(Ven_LNA_GPIO_Port, Ven_LNA_Pin, 1);
	HAL_Delay(50);
}

/**
 * @brief RF switches for CM - Rx OFF sequence
 */
void switchReceiveOff(void){
	HAL_GPIO_WritePin(Ven_LNA_GPIO_Port, Ven_LNA_Pin, 0);
	HAL_Delay(50);
	HAL_GPIO_WritePin(RFSel_GPIO_Port, RFSel_Pin, 0);
	HAL_Delay(50);
	HAL_GPIO_WritePin(RFSWEN_GPIO_Port, RFSWEN_Pin, 0);
	HAL_Delay(50);
}

/**
 * @brief Function for BME280
 */
int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len){
	if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), &reg_addr, 1, 10) != HAL_OK) return -1;
	if(HAL_I2C_Master_Receive(&hi2c1, (id << 1) | 0x01, data, len, 10) != HAL_OK) return -1;

	return 0;
}

/**
 * @brief Function for BME280
 */
void user_delay_ms(uint32_t period)
{
	HAL_Delay(period);
}

/**
 * @brief Function for BME280
 */
int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	int8_t *buf;
	buf = malloc(len +1);
	buf[0] = reg_addr;
	memcpy(buf +1, data, len);

	if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), (uint8_t*)buf, len + 1, HAL_MAX_DELAY) != HAL_OK) return -1;

	free(buf);
	return 0;
}


/**
 * @brief Get BME280 measurement, save to the 3 vars
 */
void measureBME(UART_HandleTypeDef *huart){
	/* Forced mode setting, switched to SLEEP mode after measurement */
	rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
	//	dev.delay_ms(40);
	HAL_Delay(1000);
	/* Get Data */
	rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
	if(rslt == BME280_OK)
	{
		temperature = comp_data.temperature / 100.0;
		humidity = comp_data.humidity / 1024.0;
		pressure = comp_data.pressure / 10000.0;
	}
	//	rslt = bme280_set_sensor_mode(BME280_SLEEP_MODE, &dev);

	// Print sensor value to UART if needed
	char str[50];
	snprintf(str, sizeof(str), "temp:%.2f hum:%.2f pres:%.2f\r\n",temperature, humidity, pressure);
	HAL_UART_Transmit(huart, (uint8_t *)str, sizeof(str), HAL_MAX_DELAY);
}

void getBMEValueInHex(uint8_t *sensorData, uint8_t *dataSize, UART_HandleTypeDef *huart){
	//	int tempInt = temperature * 10;
	//	int humInt = humidity * 10;
	//	int presInt = pressure * 10;

	//	strncpy(packetArray[packetCount].data, "%03x%03x", tempInt, humInt);
	//	char str[14];
	//save data into array:
	//	snprintf(packetArray[packetCount].data, sizeof(packetArray[packetCount].data), "%03x%03x", tempInt, humInt);
	//	*dataSize = snprintf((char*)sensorData, SENSOR_DATA_SIZE, "%03x%03x", tempInt, humInt);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 16);
}

/* USER CODE END 4 */

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
