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
#include "subghz.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include "radio_driver.h"
#include "stm32wlxx_nucleo.h"

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

#define RF_FREQUENCY                                400000000 /* Hz */ //900M
#define TX_OUTPUT_POWER                             14        /* dBm */
#define LORA_BANDWIDTH                              3         /* Hz */ //4
#define LORA_SPREADING_FACTOR                       10			//12
#define LORA_CODINGRATE                             1
#define LORA_PREAMBLE_LENGTH                        8         /* Same for Tx and Rx */
#define LORA_SYMBOL_TIMEOUT                         5         /* Symbols */

#define PAYLOADLENGTH								20

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

void (*volatile eventReceptor)(pingPongFSM_t *const fsm);
PacketParams_t packetParams;

const RadioLoRaBandwidths_t Bandwidths[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500, LORA_BW_031, LORA_BW_062 };

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
void printUid(UART_HandleTypeDef *huart);
uint32_t GetBandwidthInHz(RadioLoRaBandwidths_t bandwidth);
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
	MX_USART2_UART_Init();
	MX_SUBGHZ_Init();
	/* USER CODE BEGIN 2 */

	BSP_LED_Init(LED_GREEN);
	BSP_LED_Init(LED_RED);

	RadioLoRaBandwidths_t currentBandwidth = Bandwidths[LORA_BANDWIDTH];
	uint32_t bandwidthHz = GetBandwidthInHz(currentBandwidth);

	sprintf(uartBuff, "LORA_MODULATION\r\nLORA_BW=%lu Hz\r\nLORA_SF=%d\r\n", bandwidthHz, LORA_SPREADING_FACTOR);
	HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);
	radioInit();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	// get random number
	uint32_t rnd = 0;
	SUBGRF_SetDioIrqParams(IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
	rnd = SUBGRF_GetRandom();

	fsm.state = STATE_NULL;
	fsm.subState = SSTATE_NULL;
	fsm.rxTimeout = 3000; // 3000 ms to 5000ms
	fsm.rxMargin = 200;   // 200 ms
	fsm.randomDelay = rnd >> 22; // [0, 1023] ms

	HAL_Delay(fsm.randomDelay);
	SUBGRF_SetDioIrqParams( IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
			IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
			IRQ_RADIO_NONE,
			IRQ_RADIO_NONE );
	SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
	SUBGRF_SetRx(fsm.rxTimeout << 6);
	fsm.state = STATE_MASTER;
	fsm.subState = SSTATE_RX;

	printUid(&huart2);

	const uint8_t payloadsize = 20;

	while (1)
	{
		//TX MODE
		//enter tx
		HAL_UART_Transmit(&huart2, (uint8_t *)"Tx MODE\r\n", 9, HAL_MAX_DELAY);
		SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX);
		SUBGRF_WriteRegister(0x0889, (SUBGRF_ReadRegister(0x0889) | 0x04));
		packetParams.Params.LoRa.PayloadLength = payloadsize;
		SUBGRF_SetPacketParams(&packetParams);
		SUBGRF_SendPayload((uint8_t *)payload, payloadsize, 2000 << 6);
		HAL_UART_Transmit(&huart2, (uint8_t *)payload, payloadsize, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
		HAL_Delay(2000);
		HAL_UART_Transmit(&huart2, (uint8_t *)"Tx DONE\r\n\n", 10, HAL_MAX_DELAY);
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

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
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
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

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
	snprintf(buffer, sizeof(buffer), "UNIQUE ID : %08lX\r\n", uid[1]);
	HAL_UART_Transmit(huart, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/**
 * @brief  Initialize the Sub-GHz radio and dependent hardware.
 * @retval None
 */
void radioInit(void)
{
	// Initialize the hardware (SPI bus, TCXO control, RF switch)
	SUBGRF_Init(RadioOnDioIrq);

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
	packetParams.Params.LoRa.PayloadLength = PAYLOADLENGTH; //0xFF to 7
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
	//do nothing
}


/**
 * @brief  Process the RX Done event
 * @param  fsm pointer to FSM context
 * @retval None
 */
void eventRxDone(pingPongFSM_t *const fsm)
{
	uint8_t payloadLength = 0;
	uint8_t rxStartBufferPointer = 0;
	//get payload
	SUBGRF_GetRxBufferStatus(&payloadLength, &rxStartBufferPointer);
	//read payload
	SUBGRF_ReadBuffer(rxStartBufferPointer, (uint8_t *)fsm->rxBuffer, payloadLength);

	HAL_UART_Transmit(&huart2, (uint8_t *)fsm->rxBuffer, payloadLength, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);

	PacketStatus_t packetStatus;
	char uartBuff[50];
	SUBGRF_GetPacketStatus(&packetStatus);
	sprintf(uartBuff, "RssiValue=%d dBm, SnrValue=%d Hz\r\n", packetStatus.Params.LoRa.RssiPkt, packetStatus.Params.LoRa.SnrPkt);
	HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);
}


/**
 * @brief  Process the TX Timeout event
 * @param  fsm pointer to FSM context
 * @retval None
 */
void eventTxTimeout(pingPongFSM_t *const fsm)
{
	//do nothing
}


/**
 * @brief  Process the RX Timeout event
 * @param  fsm pointer to FSM context
 * @retval None
 */
void eventRxTimeout(pingPongFSM_t *const fsm)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)"Event RX Timeout\r\n", 18, HAL_MAX_DELAY);
}


/**
 * @brief  Process the RX Error event
 * @param  fsm pointer to FSM context
 * @retval None
 */
void eventRxError(pingPongFSM_t *const fsm)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)"Event Rx Error\r\n", 16, HAL_MAX_DELAY);
}


/**
 * @brief  Entry actions for the RX sub-state of the Master state
 * @param  fsm pointer to FSM context
 * @retval None
 */
void enterMasterRx(pingPongFSM_t *const fsm)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)"Master Rx start\r\n", 17, HAL_MAX_DELAY);
	SUBGRF_SetDioIrqParams( IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
			IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
			IRQ_RADIO_NONE,
			IRQ_RADIO_NONE );
	SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
	SUBGRF_SetRx(0xFFFFFF); //continuous
}


/**
 * @brief  Entry actions for the RX sub-state of the Slave state
 * @param  fsm pointer to FSM context
 * @retval None
 */
void enterSlaveRx(pingPongFSM_t *const fsm)
{
	//do nothing
}


/**
 * @brief  Entry actions for the TX sub-state of the Master state
 * @param  fsm pointer to FSM context
 * @retval None
 */
void enterMasterTx(pingPongFSM_t *const fsm)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)"Master Tx start\r\n", 17, HAL_MAX_DELAY);
	SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX);
	SUBGRF_WriteRegister(0x0889, (SUBGRF_ReadRegister(0x0889) | 0x04));
	packetParams.Params.LoRa.PayloadLength = PAYLOADLENGTH;//0x4 to PAYLOADLENGTH
	SUBGRF_SetPacketParams(&packetParams);
	SUBGRF_SendPayload((uint8_t *)"ABCDEFGHIJKLMNOPQRST", PAYLOADLENGTH, 0);
	HAL_UART_Transmit(&huart2, (uint8_t*)"ABCDEFGHIJKLMNOPQRST\r\n", PAYLOADLENGTH+2, HAL_MAX_DELAY);
}


/**
 * @brief  Entry actions for the TX sub-state of the Slave state
 * @param  fsm pointer to FSM context
 * @retval None
 */
void enterSlaveTx(pingPongFSM_t *const fsm)
{
	//do nothing
}


/**
 * @brief  Transition actions executed on every RX Done event (helper function)
 * @param  fsm pointer to FSM context
 * @retval None
 */
void transitionRxDone(pingPongFSM_t *const fsm)
{
	// PacketStatus_t packetStatus;
	// char uartBuff[50];

	// // Workaround 15.3 in DS.SX1261-2.W.APP (because following RX w/ timeout sequence)
	// SUBGRF_WriteRegister(0x0920, 0x00);
	// SUBGRF_WriteRegister(0x0944, (SUBGRF_ReadRegister(0x0944) | 0x02));

	// SUBGRF_GetPayload((uint8_t *)fsm->rxBuffer, &fsm->rxSize, 0xFF);
	// SUBGRF_GetPacketStatus(&packetStatus);

	// sprintf(uartBuff, "RssiValue=%d dBm, SnrValue=%d Hz\r\n", packetStatus.Params.LoRa.RssiPkt, packetStatus.Params.LoRa.SnrPkt);
	// HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);
}

uint32_t GetBandwidthInHz(RadioLoRaBandwidths_t bandwidth){
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
