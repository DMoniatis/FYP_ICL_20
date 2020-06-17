/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  * FINAL YEAR CODE PROJECT CODE FOR CHARGER/BMS FOR OPEN SOURCE WHEEL CHAIR
  * DATE: 17/06/20, TIME: 03:11
  * AUTHOR: DIMITRIS MONIATIS
  * DEPARTMENT OF ELECTRICAL AND ELECTRONICS ENGINEERING
  * IMPERIAL COLLEGE LONDON
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Functions
void applyPID();
void updateADC1Vars();
void updateADC2Vars();
void updateADC3Vars();
void updateADC4Vars();
void CAN_Testing_function();
void calculateActualValues();
void calculateTemps();
void initialisePIDController();
void goToSleep();
void executeOperation();
void balanceCells();
void healthCheck();
void blinkErrorCode();
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
ADC_HandleTypeDef hadc4;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;
DMA_HandleTypeDef hdma_adc4;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

// PID CONSTANTS FOR BUCK AND BOOST CONVERTER
static float KP_I_buck = 10.0f;
static float KI_I_buck = 0.5f;
static float KD_I_buck = 0.0f;

static float KP_V_buck = 20.0f;
static float KI_V_buck = 0.1f;
static float KD_V_buck = 0.05f;

static float KP_I_boost = 0.3f;
static float KI_I_boost = 0.01f;
static float KD_I_boost = 0.0f;

static float KP_V_boost = 1000.0f;
static float KI_V_boost = 0.1f;
static float KD_V_boost = 0.0f;

//Parser variables
char parsedOperand[6] = {0};
uint32_t operand = 0;

//CAN bus addresses
uint16_t OwnID = 0x124;
uint16_t RemoteID = 0x123;

//CAN bus variables
static CanTxMsgTypeDef myTxMessage;
static CanRxMsgTypeDef myRxMessage;
volatile char CanRxData[8] = {0};
volatile char TxRx_data[8];//used for UART DMA
char msg[512]; //used for UART transmission
char opCode[3]; //used for storing the

double batteryCharge = 13000.0;//stores battery charge in mAh

//Booleans
bool boostEnable = 0;
bool chargerEnable = 0;
bool balancingComplete = 0;
bool balanceEnable = 0;
bool blockCellVoltageUpdating = 0; //used to prevent cell voltages from updating when the cell balancers are enabled
bool goToSleepFlag = 0; //boolean that if set to 1 puts the system to sleep
bool boostEnableOld = 0;//used to reset PID variables in case the converter changes from boost to buck and vice versa
bool operationFromCan = 0; //used to know whether the operation was sent from CANbus or UART

//Protection flags
bool battOverCurrTrip = 0;
bool battOverVoltTrip = 0;
bool battUnderVoltTrip = 0;
bool battOverTempTrip = 0;
bool battUnderTempTrip = 0;
bool chgrOverTempTrip = 0;
bool chgrUnderTempTrip = 0;
bool battRegenCurrTrip = 0;
bool errorCodeAknowledged = 0;

//*******************************************************
//SAMPLED VARIABLES
//*******************************************************
uint16_t VinADC = 0;

uint16_t VCell0_ADC = 0;
uint16_t VCell1_ADC = 0;
uint16_t VCell2_ADC = 0;
uint16_t VCell3_ADC = 0;
uint16_t VCell4_ADC = 0;
uint16_t VCell5_ADC = 0;
uint16_t VCell6_ADC = 0;

uint16_t IOutChg_ADC = 0;
uint16_t IOutDschg_ADC = 0;

uint16_t temp1_ADC = 0;
uint16_t temp2_ADC = 0;
uint16_t temp3_ADC = 0;
uint16_t temp4_ADC = 0;

//*******************************************************
//S&H Temp calculation coefficients
//*******************************************************
//Thermistor 1 (Gold)
static double A1 = 0.000975893557500;
static double B1 = 0.000181244797500;
static double C1 = 0.000000175402880;

//Thermistor 2 (Red)
static double A2 = 0.001274837425000;
static double B2 = 0.000141531470200;
static double C2 = 0.000000269726180;

//Thermistor 3 (Green)
static double A3 = 0.001238480204000;
static double B3 = 0.000147554002700;
static double C3 = 0.000000237244079;

//Thermistor 4 (Purple)
static double A4 = 0.001129719482000;
static double B4 = 0.000158533587600;
static double C4 = 0.000000219104505;

//Thermistor resistances
double rTherm1 = 0;
double rTherm2 = 0;
double rTherm3 = 0;
double rTherm4 = 0;

//Thermistor temperatures in Celcius
float temp1 = 0;
float temp2 = 0;
float temp3 = 0;
float temp4 = 0;


//*******************************************************
//VOLTAGE PID VARIABLES
//*******************************************************
static float kP_V = 0.0f;
static float kI_V = 0.0f;
static float kD_V = 0.0f;

static float pidMinLimit_V = 0.0f;
static float currentLimit = 500.0f;
static float integralMax_V = 4000.0f;
static float integralMin_V = -4000.0f;

static float error_V = 0.0;
static float prevError_V = 0.0;
static float proportional_V = 0.0;
static float integral_V = 0.0;
static float derivative_V = 0.0;
static float presentValue_V = 0.0;
static float pidOutput_V = 0.0;
static float setpoint_V = 25.2;

//*******************************************************
//CURRENT PID VARIABLES
//*******************************************************
static float kP_I = 0.0f;
static float kI_I = 0.0f;
static float kD_I = 0.0f;

static float pidMinLimit_I = 20.0f;
static float pidMaxLimit_I = 980.0f;
static float integralMax_I = 1000.0f;
static float integralMin_I = -1000.0f;

static float error_I = 0.0;
static float prevError_I = 0.0;
static float proportional_I = 0.0;
static float integral_I = 0.0;
static float derivative_I = 0.0;
static float presentValue_I = 0.0;
static float setpoint_I = 0.0;
static float pidOutput_I = 0.0;

//cell tap voltages in V
float VC0 = 0.0;
float VC1 = 0.0;
float VC2 = 0.0;
float VC3 = 0.0;
float VC4 = 0.0;
float VC5 = 0.0;
float VC6 = 0.0;

//cell voltages in V
float vCell1 = 0.0;
float vCell2 = 0.0;
float vCell3 = 0.0;
float vCell4 = 0.0;
float vCell5 = 0.0;
float vCell6 = 0.0;

//vIn in V
float vIn = 0.0;

//currents in A
float IOutChg = 0.0;
float IOutDchg = 0.0;

uint32_t MOSFET = 1001; //duty cycle
uint32_t errorCode = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC4_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
static void CAN_FilterConfig(void); //configures the can filter
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file,char *ptr,int len)
{
	for (int i = 0; i < len; i++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
}

//DMA buffers
#define adc1BufferSize 1792
#define adc2BufferSize 320
#define adc3BufferSize 1280
#define adc4BufferSize 1536

uint16_t adc1Buffer[adc1BufferSize];
uint16_t adc2Buffer[adc2BufferSize];
uint16_t adc3Buffer[adc3BufferSize];
uint16_t adc4Buffer[adc4BufferSize];


//************************************************************************************************************************************************************
//--MAIN--MAIN--MAIN--MAIN--MAIN--MAIN--MAIN--MAIN--MAIN--MAIN--MAIN--MAIN--MAIN--MAIN--MAIN--MAIN--MAIN--MAIN--MAIN--MAIN--MAIN--MAIN--MAIN--MAIN--MAIN--MAIN
//************************************************************************************************************************************************************
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	sleepReset:

	//initialise the 4 ADC content holding arrays after a sleep to get correct ADC readings
	for (int i = 0; i < adc1BufferSize; i++ ){adc1Buffer[i] = 0;}
	for (int i = 0; i < adc2BufferSize; i++ ){adc2Buffer[i] = 0;}
	for (int i = 0; i < adc3BufferSize; i++ ){adc3Buffer[i] = 0;}
	for (int i = 0; i < adc4BufferSize; i++ ){adc4Buffer[i] = 0;}

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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_ADC4_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

  //CAN configuration
  CAN_FilterConfig(); //we initialise the CAN bus filter

  HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);

  //Set CAN transmission parameters
  hcan.pTxMsg->StdId = OwnID;
  hcan.pTxMsg->RTR = CAN_RTR_DATA;
  hcan.pTxMsg->IDE = CAN_ID_STD;
  hcan.pTxMsg->DLC = 8;

  //Start PWM for the buck boost MOSFETS
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1); //PMOS
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2); //NMOS

  //Calibrate ADCs
  while(HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED) != HAL_OK);
  while(HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED) != HAL_OK);
  while(HAL_ADCEx_Calibration_Start(&hadc3,ADC_SINGLE_ENDED) != HAL_OK);
  while(HAL_ADCEx_Calibration_Start(&hadc4,ADC_SINGLE_ENDED) != HAL_OK);

  //HAL_ADCEx_MultiModeStart_DMA(&hadc1,(uint32_t*)adc1Buffer, adc1BufferSize*2);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc1Buffer, adc1BufferSize);
  HAL_ADC_Start_DMA(&hadc2,(uint32_t*)adc2Buffer, adc2BufferSize);
  HAL_ADC_Start_DMA(&hadc3,(uint32_t*)adc3Buffer, adc3BufferSize);
  HAL_ADC_Start_DMA(&hadc4,(uint32_t*)adc4Buffer, adc4BufferSize);


  HAL_TIM_Base_Start_IT(&htim2); //start timer 2 for 500 Hz core ISR

  HAL_GPIO_WritePin(CB_En_GPIO_Port,CB_En_Pin, GPIO_PIN_SET); //connect load
  HAL_GPIO_WritePin(VC_En_GPIO_Port,VC_En_Pin, GPIO_PIN_SET); //enable cell voltage measurements
  HAL_GPIO_WritePin(Temp_En_GPIO_Port,Temp_En_Pin, GPIO_PIN_SET); //connect thermistors
  HAL_GPIO_WritePin(CANmode_GPIO_Port,CANmode_Pin, GPIO_PIN_RESET); //enable can transmitter

  sprintf(msg,">>>>> Prototype ready <<<<< \r\n");
  HAL_UART_Transmit(&huart3,msg, strlen(msg), HAL_MAX_DELAY);

  HAL_UART_Receive_DMA(&huart3,TxRx_data,8); //re-enable DMA for UART

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//**********************************************************************************************************************************************************
//--WHILE--WHILE--WHILE--WHILE--WHILE--WHILE--WHILE--WHILE--WHILE--WHILE--WHILE--WHILE--WHILE--WHILE--WHILE--WHILE--WHILE--WHILE--WHILE--WHILE--WHILE--WHILE
//**********************************************************************************************************************************************************
  while (1)
  {
	  if(goToSleepFlag == 1){goToSleep();goto sleepReset;} //allows correct exiting of sleep from CAN induced sleep
	  if(errorCode != 0){blinkErrorCode();}
	  else{HAL_GPIO_WritePin(timingPin_GPIO_Port, timingPin_Pin,GPIO_PIN_SET);}

 	 //UNCOMMENT FOR HUMAN READABLE UART REPORTING
	  sprintf(msg,"iD:%lu iC:%lu iSP:%lu Cap:%lu D:%lu V1:%lu V2:%lu V3:%lu V4:%lu V5:%lu V6:%lu vO:%lu vSP:%lu vI:%lu T1:%lu T2:%lu T3:%lu T4:%lu bstEn:%lu chgEn:%lu blEn:%lu blCpl:%lu E1:%lu E2:%lu E3:%lu E4:%lu E5:%lu E6:%lu E7:%lu E8:%lu \r\n",
			   (uint32_t)(IOutDchg*1000),(uint32_t)(IOutChg*1000),(uint32_t)currentLimit,(uint32_t)batteryCharge,MOSFET,(uint32_t)(vCell1*1000),(uint32_t)(vCell2*1000),(uint32_t)(vCell3*1000),(uint32_t)(vCell4*1000),(uint32_t)(vCell5*1000),(uint32_t)(vCell6*1000),(uint32_t)(VC6*1000),(uint32_t)(setpoint_V*1000),(uint32_t)(vIn*1000),(uint32_t)temp1,(uint32_t)temp2,(uint32_t)temp3,(uint32_t)temp4,
			   boostEnable,chargerEnable,balanceEnable,balancingComplete,battOverTempTrip,battUnderTempTrip,battOverVoltTrip,battUnderVoltTrip,battOverCurrTrip,battRegenCurrTrip,chgrOverTempTrip,chgrUnderTempTrip);
	  HAL_UART_Transmit(&huart3,msg, strlen(msg), HAL_MAX_DELAY);


	  //ENABLE FOR CSV FILE LOGGING
	  /*
	  sprintf(msg,"%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,\r\n",
			   (uint32_t)(IOutDchg*1000),(uint32_t)(IOutChg*1000),(uint32_t)currentLimit,(uint32_t)batteryCharge,MOSFET,(uint32_t)(vCell1*1000),(uint32_t)(vCell2*1000),(uint32_t)(vCell3*1000),(uint32_t)(vCell4*1000),(uint32_t)(vCell5*1000),(uint32_t)(vCell6*1000),(uint32_t)(VC6*1000),(uint32_t)(setpoint_V*1000),(uint32_t)(vIn*1000),(uint32_t)temp1,(uint32_t)temp2,(uint32_t)temp3,(uint32_t)temp4,
			   boostEnable,chargerEnable,balanceEnable,balancingComplete,battOverTempTrip,battUnderTempTrip,battOverVoltTrip,battUnderVoltTrip,battOverCurrTrip,battRegenCurrTrip,chgrOverTempTrip,chgrUnderTempTrip);
	  HAL_UART_Transmit(&huart3,msg, strlen(msg), HAL_MAX_DELAY);
	  */

	  HAL_Delay(200);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_TIM2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_PLLCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 7;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode 
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 5;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Common config 
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 5;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode 
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief ADC4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC4_Init(void)
{

  /* USER CODE BEGIN ADC4_Init 0 */

  /* USER CODE END ADC4_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC4_Init 1 */

  /* USER CODE END ADC4_Init 1 */
  /** Common config 
  */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc4.Init.ContinuousConvMode = ENABLE;
  hadc4.Init.DiscontinuousConvMode = DISABLE;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.NbrOfConversion = 6;
  hadc4.Init.DMAContinuousRequests = ENABLE;
  hadc4.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC4_Init 2 */

  /* USER CODE END ADC4_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
	  hcan.Instance = CAN;               // Register base address
	  hcan.Init.Prescaler = 16;           //Specifies the length of a time quantum.
	  hcan.pTxMsg = &myTxMessage;         //Pointer to transmit structure
	  hcan.pRxMsg = &myRxMessage;		  //Pointer to reception structure for RX FIFO0 msg
	  hcan.Init.Mode = CAN_MODE_NORMAL;	  //Specifies the CAN operating mode : Normal mode
	  hcan.Init.SJW = CAN_SJW_1TQ;        //Specifies the maximum number of time quanta the CAN hardware is allowed to lengthen or shorten a bit to perform resynchronization
	  hcan.Init.BS1 = CAN_BS1_11TQ;		  //Specifies the number of time quanta in Bit Segment 1
	  hcan.Init.BS2 = CAN_BS2_5TQ;		  //Specifies the number of time quanta in Bit Segment 2
	  hcan.Init.TTCM = DISABLE;			  //Enable or disable the time triggered communication mode
	  hcan.Init.ABOM = ENABLE;			  //Enable or disable the automatic bus-off management
	  hcan.Init.AWUM = ENABLE;			  //Enable or disable the automatic wake-up mode
	  hcan.Init.NART = DISABLE;			  //Enable or disable the non-automatic retransmission mode
	  hcan.Init.RFLM = DISABLE;		      //Enable or disable the receive FIFO Locked mode
	  hcan.Init.TXFP = DISABLE;           //Enable or disable the transmit FIFO priority
	  if (HAL_CAN_Init(&hcan) != HAL_OK)
	  {
		  Error_Handler();
	  }
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */

  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 256;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 562;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart3.Init.BaudRate = 230400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  /* DMA2_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Temp_En_Pin|BalC1_Pin|BalC2_Pin|BalC3_Pin 
                          |BalC4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CB_En_Pin|VC_En_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BalC5_Pin|BalC6_Pin|CANmode_Pin|timingPin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PwrBtn_Pin */
  GPIO_InitStruct.Pin = PwrBtn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PwrBtn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Temp_En_Pin BalC1_Pin BalC2_Pin BalC3_Pin 
                           BalC4_Pin */
  GPIO_InitStruct.Pin = Temp_En_Pin|BalC1_Pin|BalC2_Pin|BalC3_Pin 
                          |BalC4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CB_En_Pin VC_En_Pin */
  GPIO_InitStruct.Pin = CB_En_Pin|VC_En_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BalC5_Pin BalC6_Pin CANmode_Pin timingPin_Pin */
  GPIO_InitStruct.Pin = BalC5_Pin|BalC6_Pin|CANmode_Pin|timingPin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

//**********************************************************************************************************************************************************
//FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--
//**********************************************************************************************************************************************************

//**********************************************************************************************************************************************************
void updateADC3Vars() //sums and decimates the variables related to ADC3
{
	uint32_t VCell1_Accumulator = 0;
	uint32_t VCell4_Accumulator = 0;

	for(uint16_t i = 0;i < adc3BufferSize;i += 5)
	{
		VCell1_Accumulator += adc3Buffer[i];

		VCell4_Accumulator += adc3Buffer[i+1];
		VCell4_Accumulator += adc3Buffer[i+2];
		VCell4_Accumulator += adc3Buffer[i+3];
		VCell4_Accumulator += adc3Buffer[i+4];
	}
	VCell4_ADC = VCell4_Accumulator >> 6; // 1024 samples taken
	VCell1_ADC = VCell1_Accumulator >> 6; // 256 samples taken
}

//**********************************************************************************************************************************************************
void updateADC2Vars() //sums and decimates the variables related to ADC2
{
	uint32_t   Vin_Accumulator = 0;
	uint32_t Temp1_Accumulator = 0;
	uint32_t Temp2_Accumulator = 0;
	uint32_t Temp3_Accumulator = 0;
	uint32_t Temp4_Accumulator = 0;

	for(uint16_t i = 0;i < adc2BufferSize;i += 5)
	{
		Vin_Accumulator   += adc2Buffer[i];
		Temp1_Accumulator += adc2Buffer[i+1];
		Temp2_Accumulator += adc2Buffer[i+2];
		Temp3_Accumulator += adc2Buffer[i+3];
		Temp4_Accumulator += adc2Buffer[i+4];
	}

	VinADC    = Vin_Accumulator   >> 6;	// 64 samples taken
	temp1_ADC = Temp1_Accumulator >> 6; // 64 samples taken
	temp2_ADC = Temp2_Accumulator >> 6; // 64 samples taken
	temp3_ADC = Temp3_Accumulator >> 6; // 64 samples taken
	temp4_ADC = Temp4_Accumulator >> 6; // 64 samples taken
}

//**********************************************************************************************************************************************************
void updateADC1Vars() //sums and decimates the variables related to ADC1
{
	uint32_t VCell3_Accumulator = 0;
	uint32_t VCell6_Accumulator = 0;
	uint32_t IOutChg_Accumulator = 0;
	uint32_t IOutDschg_Accumulator = 0;

	for(uint16_t i = 0;i < adc1BufferSize;i += 7)
	{
		VCell6_Accumulator += adc1Buffer[i];
		VCell6_Accumulator += adc1Buffer[i+2];
		VCell6_Accumulator += adc1Buffer[i+4];
		VCell6_Accumulator += adc1Buffer[i+6];

		VCell3_Accumulator += adc1Buffer[i+1];

		IOutChg_Accumulator  += adc1Buffer[i+3];
		IOutDschg_Accumulator  += adc1Buffer[i+5];
	}

	VCell3_ADC = VCell3_Accumulator >> 6; // 256 samples taken
	VCell6_ADC = VCell6_Accumulator >> 6; // 1024 samples taken

	IOutChg_ADC = IOutChg_Accumulator >> 6; // 256 samples taken
	IOutDschg_ADC = IOutDschg_Accumulator >> 6; // 256 samples taken

}

//**********************************************************************************************************************************************************
void updateADC4Vars() //sums and decimates the variables related to ADC4
{

	uint32_t VCell0_Accumulator = 0;
	uint32_t VCell2_Accumulator = 0;
	uint32_t VCell5_Accumulator = 0;

	for(uint16_t i = 0;i < adc4BufferSize;i += 6)
	{
		VCell5_Accumulator += adc4Buffer[i+1];
		VCell5_Accumulator += adc4Buffer[i+2];
		VCell5_Accumulator += adc4Buffer[i+4];
		VCell5_Accumulator += adc4Buffer[i+5];

		VCell0_Accumulator += adc4Buffer[i];

		VCell2_Accumulator  += adc4Buffer[i+3];
	}

	VCell0_ADC = VCell0_Accumulator >> 6; // 256 samples taken
	VCell2_ADC = VCell2_Accumulator >> 6; // 256 samples taken
	VCell5_ADC = VCell5_Accumulator >> 6; // 1024 samples taken

}

//**********************************************************************************************************************************************************
void calculateActualValues() //translates ADC readings into real world quantities 500 times a second using MS Excel lines of best fit
{
	//current is in Amps voltages are in Volts
	//battery charge is in mAh
   if(blockCellVoltageUpdating == 0) //updates tap voltages only when we are not balance charging, otherwise voltages remain frozen
   {
	VC6 = 0.0003949414 * VCell6_ADC	+ 0.0108149390;
	VC5 = 0.0003353593 * VCell5_ADC	+ 0.0001946072;
	VC4 = 0.0002777437 * VCell4_ADC	+ 0.0037047450;
	VC3 = 0.0008159491 * VCell3_ADC	+ 0.0025696922;
	VC2 = 0.0005180698 * VCell2_ADC	+ 0.0031440949;
	VC1 = 0.0002572566 * VCell1_ADC	+ 0.0009649199;
	VC0 = 0.0000523515 * VCell0_ADC	- 0.0008254950;
   }

	IOutChg  = 0.0005065846 * IOutChg_ADC	-   0.0018371205 - 0.03;
	IOutDchg = 0.0020342438 * IOutDschg_ADC	-  0.0633905546 + 0.03;
	vIn      = 0.0061750796 * VinADC   	    -  0.0172273763;

	//accumulates battery capacity
	if (IOutChg >= IOutDchg){batteryCharge = batteryCharge + IOutChg/1800;}
	else					{batteryCharge = batteryCharge - IOutDchg/1800;}

	//calculated cell voltages
	vCell1 = VC1 - VC0;
	vCell2 = VC2 - VC1;
	vCell3 = VC3 - VC2;
	vCell4 = VC4 - VC3;
	vCell5 = VC5 - VC4;
	vCell6 = VC6 - VC5;
}

//**********************************************************************************************************************************************************
void calculateTemps() //calculates temperatures using the S&H method once a second
{
	rTherm1 = (247500 - 60.42480469*temp1_ADC)/(0.0008056640625*temp1_ADC);
	rTherm2 = (247500 - 60.42480469*temp2_ADC)/(0.0008056640625*temp2_ADC);
	rTherm3 = (495000 - 120.8496094*temp3_ADC)/(0.0008056640625*temp3_ADC);
	rTherm4 = (495000 - 120.8496094*temp4_ADC)/(0.0008056640625*temp4_ADC);

	temp1 = (float)(1/(A1 + B1*log(rTherm1) + C1*pow(log(rTherm1),3.0)) - 273.15);
	temp2 = (float)(1/(A2 + B2*log(rTherm2) + C2*pow(log(rTherm2),3.0)) - 273.15);
	temp3 = (float)(1/(A3 + B3*log(rTherm3) + C3*pow(log(rTherm3),3.0)) - 273.15);
	temp4 = (float)(1/(A4 + B4*log(rTherm4) + C4*pow(log(rTherm4),3.0)) - 273.15);
}

//**********************************************************************************************************************************************************
void initialisePIDController() //initialises the PID controller
{
	boostEnable = 0;
	boostEnableOld = 0;
    error_V = 0.0;
	prevError_V = 0.0;
	proportional_V = 0.0;
	integral_V = 0.0;
	derivative_V = 0.0;
	presentValue_V = 0.0;
	pidOutput_V = 0.0;

	error_I = 0.0;
	prevError_I = 0.0;
	proportional_I = 0.0;
	integral_I = 0.0;
	derivative_I = 0.0;
	presentValue_I = 0.0;
	setpoint_I = 0.0;
	pidOutput_I = 0.0;
}

//**********************************************************************************************************************************************************
void executeOperation() //CAN bus and UART parser
{
	//? = don't care as long as a character is there, $ = integer

	//change main constant current and constant voltage parameters in mA and mV respectively
	if 		(strcmp(opCode, "SCC") == 0){ currentLimit = (float)operand;  sprintf(TxRx_data,"SCC->ACK");}   // Set Charging Current: SCC?$$$$ in mA
	else if (strcmp(opCode, "SCV") == 0){ setpoint_V = (float)operand/1000; sprintf(TxRx_data,"SCV->ACK");} // Set Charging Voltage: SCV$$$$$ in mV

	//changing PID constants for buck and boost, boost = stepup(converter), buck = buck (converter) in the form $$.$$$
	else if (strcmp(opCode, "PCS") == 0){ KP_I_boost = (float)operand/100; sprintf(TxRx_data,"PCS->ACK"); } // Proportional Current Stepup: PCS$$$$$
	else if (strcmp(opCode, "ICS") == 0){ KI_I_boost = (float)operand/100; sprintf(TxRx_data,"ICS->ACK"); } // Integral 	  Current Stepup: ICS$$$$$
	else if (strcmp(opCode, "DCS") == 0){ KD_I_boost = (float)operand/100; sprintf(TxRx_data,"DCS->ACK"); } // Derivative   Current Stepup: DCS$$$$$
	else if (strcmp(opCode, "PVS") == 0){ KP_V_boost = (float)operand/100; sprintf(TxRx_data,"PVS->ACK"); } // Proportional Current Stepup: PVS$$$$$
	else if (strcmp(opCode, "IVS") == 0){ KI_V_boost = (float)operand/100; sprintf(TxRx_data,"IVS->ACK"); } // Integral 	  Current Stepup: IVS$$$$$
	else if (strcmp(opCode, "DVS") == 0){ KD_V_boost = (float)operand/100; sprintf(TxRx_data,"DVS->ACK"); } // Derivative   Current Stepup: DVS$$$$$
	else if (strcmp(opCode, "PCB") == 0){ KP_I_buck  = (float)operand/100; sprintf(TxRx_data,"PCB->ACK"); } // Proportional Current Buck  : PCB$$$$$
	else if (strcmp(opCode, "ICB") == 0){ KI_I_buck  = (float)operand/100; sprintf(TxRx_data,"ICB->ACK"); } // Integral 	  Current Buck  : ICB$$$$$
	else if (strcmp(opCode, "DCB") == 0){ KD_I_buck  = (float)operand/100; sprintf(TxRx_data,"DCB->ACK"); } // Derivative   Current Buck  : DCB$$$$$
	else if (strcmp(opCode, "PVB") == 0){ KP_V_buck  = (float)operand/100; sprintf(TxRx_data,"PVB->ACK"); } // Proportional Current Buck  : PVB$$$$$
	else if (strcmp(opCode, "IVB") == 0){ KI_V_buck  = (float)operand/100; sprintf(TxRx_data,"IVB->ACK"); } // Integral 	  Current Buck  : IVB$$$$$
	else if (strcmp(opCode, "DVB") == 0){ KD_V_buck  = (float)operand/100; sprintf(TxRx_data,"DVB->ACK"); } // Derivative   Current Buck  : DVB$$$$$


	else if (strcmp(opCode, "GTS") == 0){sprintf(TxRx_data,"GTS->ACK");goToSleepFlag = 1;}   // Go To Sleep
	else if (strcmp(opCode, "BCB") == 0){sprintf(TxRx_data,"BCB->ACK");initialisePIDController(); chargerEnable = 1; balancingComplete = 0;}   // Begin Charging Battery
	else if (strcmp(opCode, "SCB") == 0){sprintf(TxRx_data,"SCB->ACK");chargerEnable = 0;} // Stop Charging Battery

	else if (strcmp(opCode, "TC1") == 0){sprintf(TxRx_data,"%lu K",(uint32_t)(temp1+273.15f));} // Temperature Charger 1 : TC1?????
	else if (strcmp(opCode, "TC2") == 0){sprintf(TxRx_data,"%lu K",(uint32_t)(temp2+273.15f));} // Temperature Charger 2 : TC2?????
	else if (strcmp(opCode, "TB1") == 0){sprintf(TxRx_data,"%lu K",(uint32_t)(temp3+273.15f));} // Temperature Battery 1 : TB1?????
	else if (strcmp(opCode, "TB2") == 0){sprintf(TxRx_data,"%lu K",(uint32_t)(temp4+273.15f));} // Temperature Battery 2 : TB2?????

	else if (strcmp(opCode, "VC1") == 0){sprintf(TxRx_data,"%lu mV",(uint32_t)((vCell1)*1000));} // Voltage Cell 1: VC1?????
	else if (strcmp(opCode, "VC2") == 0){sprintf(TxRx_data,"%lu mV",(uint32_t)((vCell2)*1000));} // Voltage Cell 2: VC2?????
	else if (strcmp(opCode, "VC3") == 0){sprintf(TxRx_data,"%lu mV",(uint32_t)((vCell3)*1000));} // Voltage Cell 3: VC3?????
	else if (strcmp(opCode, "VC4") == 0){sprintf(TxRx_data,"%lu mV",(uint32_t)((vCell4)*1000));} // Voltage Cell 4: VC4?????
	else if (strcmp(opCode, "VC5") == 0){sprintf(TxRx_data,"%lu mV",(uint32_t)((vCell5)*1000));} // Voltage Cell 5: VC5?????
	else if (strcmp(opCode, "VC6") == 0){sprintf(TxRx_data,"%lu mV",(uint32_t)((vCell6)*1000));} // Voltage Cell 6: VC6?????

	else if (strcmp(opCode, "BTV") == 0){sprintf(TxRx_data,"%lu mV",(uint32_t)(VC6*1000));}      // Battery Terminal Voltage : BTV?????
	else if (strcmp(opCode, "BCC") == 0){sprintf(TxRx_data,"%lu mA",(uint32_t)(IOutChg*1000));}  // Battery Charging Current : BCC?????
	else if (strcmp(opCode, "BDC") == 0){sprintf(TxRx_data,"%lu mA",(uint32_t)(IOutDchg*1000));} // Battery Discharging Current : BDC?????
	else if (strcmp(opCode, "CIV") == 0){sprintf(TxRx_data,"%lu mV",(uint32_t)(vIn*1000));}      // Charger Input Voltage : CIV?????

	else if (strcmp(opCode, "RBC") == 0){sprintf(TxRx_data,"%lumAh",(uint32_t)(batteryCharge));} // Remaining Battery Charge : RBC?????
	else if (strcmp(opCode, "SBC") == 0){batteryCharge = (float)operand; sprintf(TxRx_data,"SBC->ACK");} // Set Battery Charge: SBCNNNNN

	else if (strcmp(opCode, "REC") == 0){sprintf(TxRx_data,"%lu",errorCode);} // Return Error Code : REC?????
	else if (strcmp(opCode, "CEC") == 0)
	{
		sprintf(TxRx_data,"CEC->ACK");
		errorCode = 0;
		battOverTempTrip  = 0;
		battUnderTempTrip = 0;
		chgrOverTempTrip = 0;
		chgrUnderTempTrip = 0;
		battOverVoltTrip = 0;
		battUnderVoltTrip = 0;
		battOverCurrTrip = 0;
		battRegenCurrTrip = 0;
		HAL_GPIO_WritePin(CB_En_GPIO_Port,CB_En_Pin, GPIO_PIN_SET); //disconnect load
	}

	else {sprintf(TxRx_data,"->ERROR");chargerEnable = 0;}

	HAL_UART_Transmit(&huart3,TxRx_data, strlen(TxRx_data), HAL_MAX_DELAY);

	//checks whether the command was received from CAN or UART, if from can only then enter IF statement, otherwise the program freezes as it tries to send data
	//over a disconnected CANbus forever
	if(operationFromCan == 1)
	{
		//transfer data received from UART via DMA to CAN buffer
		for (uint8_t i = 0; i < 8;i++)
		{
			hcan.pTxMsg->Data[i] = TxRx_data[i];
		}
		HAL_CAN_Transmit(&hcan, 10);
		operationFromCan = 0; //reset
	}

}


//**********************************************************************************************************************************************************
void applyPID() //calculates and applies cascaded PID control algorithm to MOSFETs of buck-boost converter
{
	//change PIDs according to whether we are boosting or bucking
	if (boostEnable == 1)
	{
	 kP_I = KP_I_boost;
	 kI_I = KI_I_boost;
	 kD_I = KD_I_boost;

     kP_V = KP_V_boost;
	 kI_V = KI_V_boost;
	 kD_V = KD_V_boost;
	}
	else
	{
	 kP_I = KP_I_buck;
	 kI_I = KI_I_buck;
	 kD_I = KD_I_buck;

	 kP_V = KP_V_buck;
	 kI_V = KI_V_buck;
	 kD_V = KD_V_buck;
	}

	//*******************************************************
	//OUTER VOLTAGE PID LOOP
	//*******************************************************
	presentValue_V = VC6 - VC0;

	error_V = setpoint_V - presentValue_V;

	proportional_V = kP_V * error_V;

	integral_V = integral_V + kI_V * error_V;
	if (integral_V > currentLimit){integral_V = currentLimit;}
	else if (integral_V < -currentLimit){integral_V = -currentLimit;}

	derivative_V = kD_V*(error_V - prevError_V);
	prevError_V = error_V;

	setpoint_I = proportional_V + integral_V + derivative_V;
	if (setpoint_I <= 0){setpoint_I = 0;}
	else if (setpoint_I >= currentLimit){setpoint_I = currentLimit;}

	setpoint_I = setpoint_I/1000;

	//*******************************************************
	//INNER CURRENT PID LOOP
	//*******************************************************
	presentValue_I = IOutChg;

	error_I = setpoint_I - presentValue_I;

	proportional_I = kP_I * error_I;

	integral_I = integral_I + kI_I * error_I;
	if (integral_I > integralMax_I){integral_I = integralMax_I;}
	else if (integral_I < integralMin_I){integral_I = integralMin_I;}

	derivative_I = kD_I*(error_I - prevError_I);
	prevError_I = error_I;

	pidOutput_I = proportional_I + integral_I + derivative_I;
	if (pidOutput_I <= pidMinLimit_I){pidOutput_I = pidMinLimit_I;}
	else if (pidOutput_I >= pidMaxLimit_I){pidOutput_I = pidMaxLimit_I;}

	MOSFET = (uint32_t)pidOutput_I;

	if (boostEnableOld != boostEnable) //if we have changed from boost to buck and vice versa, reset PID variables
	{
		 error_I = 0.0;
		 prevError_I = 0.0;
		 proportional_I = 0.0;
		 integral_I = 0.0;
		 derivative_I = 0.0;
		 presentValue_I = 0.0;
		 pidOutput_I = 0.0;
		 MOSFET = 0;
		 boostEnableOld = boostEnable;
	}

	if (boostEnable == 1)
	{
		TIM1->CCR2 = 1001; //turn on PMOS
		TIM1->CCR1 = pidMaxLimit_I - MOSFET;
	}
	else //if boost is disabled aka we are bucking
	{
		TIM1->CCR1 = 1001; //turn off NMOS
		TIM1->CCR2 = MOSFET;
	}

}

//**********************************************************************************************************************************************************
void balanceCells() //balances the 6 cells of the battery
{
	float cellVoltageArray[6] = {vCell1,vCell2,vCell3,vCell4,vCell5,vCell6};
	float cellMaxVoltage = 0.0f;
	float cellMinVoltage = 5.0f;
	uint8_t minVoltageCellNo = 0;

	//find cells with lowest and highest voltage along with their number
	for(int i = 0; i < 6;i++)
	{
		if (cellVoltageArray[i] < cellMinVoltage)
		{
			cellMinVoltage = cellVoltageArray[i];
			minVoltageCellNo = i + 1;
		}
		if (cellVoltageArray[i] > cellMaxVoltage)
		{
			cellMaxVoltage = cellVoltageArray[i];
		}
	}

	//if the difference between the cell voltages is below 10mV then stop balancing
	if(((cellMaxVoltage - cellMinVoltage) < 0.005) && (VC6 > (setpoint_V - 0.01)) && (VC6 < (setpoint_V + 0.01)))
	{
		balancingComplete = 1;
		balanceEnable = 0;
		HAL_GPIO_WritePin(BalC1_GPIO_Port,BalC1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BalC2_GPIO_Port,BalC2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BalC3_GPIO_Port,BalC3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BalC4_GPIO_Port,BalC4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BalC5_GPIO_Port,BalC5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BalC6_GPIO_Port,BalC6_Pin, GPIO_PIN_RESET);
	}
	else
	{
		switch(minVoltageCellNo) //discharge all cells with a voltage higher than the cell with the lowest voltage
		{
			case 1:
				HAL_GPIO_WritePin(BalC1_GPIO_Port,BalC1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(BalC2_GPIO_Port,BalC2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(BalC3_GPIO_Port,BalC3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(BalC4_GPIO_Port,BalC4_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(BalC5_GPIO_Port,BalC5_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(BalC6_GPIO_Port,BalC6_Pin, GPIO_PIN_SET);
			break;

			case 2:
				HAL_GPIO_WritePin(BalC1_GPIO_Port,BalC1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(BalC2_GPIO_Port,BalC2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(BalC3_GPIO_Port,BalC3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(BalC4_GPIO_Port,BalC4_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(BalC5_GPIO_Port,BalC5_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(BalC6_GPIO_Port,BalC6_Pin, GPIO_PIN_SET);
			break;

			case 3:
				HAL_GPIO_WritePin(BalC1_GPIO_Port,BalC1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(BalC2_GPIO_Port,BalC2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(BalC3_GPIO_Port,BalC3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(BalC4_GPIO_Port,BalC4_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(BalC5_GPIO_Port,BalC5_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(BalC6_GPIO_Port,BalC6_Pin, GPIO_PIN_SET);
			break;

			case 4:
				HAL_GPIO_WritePin(BalC1_GPIO_Port,BalC1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(BalC2_GPIO_Port,BalC2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(BalC3_GPIO_Port,BalC3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(BalC4_GPIO_Port,BalC4_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(BalC5_GPIO_Port,BalC5_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(BalC6_GPIO_Port,BalC6_Pin, GPIO_PIN_SET);
			break;

			case 5:
				HAL_GPIO_WritePin(BalC1_GPIO_Port,BalC1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(BalC2_GPIO_Port,BalC2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(BalC3_GPIO_Port,BalC3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(BalC4_GPIO_Port,BalC4_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(BalC5_GPIO_Port,BalC5_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(BalC6_GPIO_Port,BalC6_Pin, GPIO_PIN_SET);
			break;

			case 6:
				HAL_GPIO_WritePin(BalC1_GPIO_Port,BalC1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(BalC2_GPIO_Port,BalC2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(BalC3_GPIO_Port,BalC3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(BalC4_GPIO_Port,BalC4_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(BalC5_GPIO_Port,BalC5_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(BalC6_GPIO_Port,BalC6_Pin, GPIO_PIN_RESET);
			break;
		}
	}
}

//**********************************************************************************************************************************************************
void healthCheck()
{
	static bool firstRun = 1;

	if (firstRun == 0) //don't run checks the first time due to false trigger
	{
		//batt over temp protection
		if ((temp3 > 50)||(temp4 > 50))
		{
			battOverTempTrip  = 1;
		}

		//batt under temp protection
		if ((temp3 < 0) || (temp4 < 0))
		{
			battUnderTempTrip = 1;
		}

		//batt over voltage protection
		if ((vCell1 > 4.22)||(vCell2 > 4.22)||(vCell3 > 4.22)||(vCell4 > 4.22)||(vCell5 > 4.22)||(vCell6 > 4.22)||(VC6 > 25.4))
		{
			battOverVoltTrip = 1;
		}

		//batt under voltage protection
		if ((vCell1 < 2.5)||(vCell2 < 2.5)||(vCell3 < 2.5)||(vCell4 < 2.5)||(vCell5 < 2.5)||(vCell6 < 2.5)||(VC6 < 14.9))
		{
			battUnderVoltTrip = 1;
		}

		//batt over discharge current protection
		if (IOutDchg > 30.0)
		{
			battOverCurrTrip = 1;
		}

		//batt over regeneration current protection
		if (IOutChg  > 10.0)
		{
			battRegenCurrTrip = 1;
		}

		//charger over temp protection
		if ((temp1 > 100) || (temp2 > 100))
		{
			chgrOverTempTrip = 1;
		}

		//charger under temp protection
		if ((temp1 < -20) || (temp2 < -20 ))
		{
			chgrUnderTempTrip = 1;
		}

		//compile error code into the 8 digit errorCode variable
		errorCode = battOverTempTrip * 10000000 + battUnderTempTrip * 1000000 + battOverVoltTrip * 100000 + battUnderVoltTrip * 10000 + battOverCurrTrip * 1000 + battRegenCurrTrip * 100 + chgrOverTempTrip * 10 + chgrUnderTempTrip * 1;

		//if the error code is not cleared
		if (errorCode != 0)
		{
			chargerEnable = 0; //stop charging
			balanceEnable = 0; //stop balancing
			blockCellVoltageUpdating = 0; //allow cell voltage updating
			boostEnable = 0; //disable boost converter
			HAL_GPIO_WritePin(CB_En_GPIO_Port,CB_En_Pin, GPIO_PIN_RESET); //disconnect load
			HAL_GPIO_WritePin(BalC1_GPIO_Port,BalC1_Pin, GPIO_PIN_RESET); //stop discharging Cell 1
			HAL_GPIO_WritePin(BalC2_GPIO_Port,BalC2_Pin, GPIO_PIN_RESET); //stop discharging Cell 2
			HAL_GPIO_WritePin(BalC3_GPIO_Port,BalC3_Pin, GPIO_PIN_RESET); //stop discharging Cell 3
			HAL_GPIO_WritePin(BalC4_GPIO_Port,BalC4_Pin, GPIO_PIN_RESET); //stop discharging Cell 4
			HAL_GPIO_WritePin(BalC5_GPIO_Port,BalC5_Pin, GPIO_PIN_RESET); //stop discharging Cell 5
			HAL_GPIO_WritePin(BalC6_GPIO_Port,BalC6_Pin, GPIO_PIN_RESET); //stop discharging Cell 6
			TIM1->CCR2 = 0; //turn off PMOS
			TIM2->CCR1 = 1001; //turn on NMOS so as not to consume power from BJT connected to MCU
		}
	}
	else{firstRun = 0;}
}

//**********************************************************************************************************************************************************
void blinkErrorCode() //blinks the error code on the onboard green LED
{
	static uint16_t delayTime = 200;

	switch (errorCode)
	{
		case 10000000:  //Error code 1
			HAL_Delay(3*delayTime);
			HAL_GPIO_WritePin(timingPin_GPIO_Port,timingPin_Pin, GPIO_PIN_SET);
			HAL_Delay(delayTime);
			HAL_GPIO_WritePin(timingPin_GPIO_Port,timingPin_Pin, GPIO_PIN_RESET);
			HAL_Delay(delayTime);
			HAL_Delay(3*delayTime);
			break;

		case 1000000: //Error code 2
			HAL_Delay(3*delayTime);
			for(int i = 0; i < 2 ; i++)
			{
				HAL_GPIO_WritePin(timingPin_GPIO_Port,timingPin_Pin, GPIO_PIN_SET);
				HAL_Delay(delayTime);
				HAL_GPIO_WritePin(timingPin_GPIO_Port,timingPin_Pin, GPIO_PIN_RESET);
				HAL_Delay(delayTime);
			}
			HAL_Delay(3*delayTime);
			break;

		case 100000: //Error code 3
			HAL_Delay(3*delayTime);
			for(int i = 0; i < 3 ; i++)
			{
				HAL_GPIO_WritePin(timingPin_GPIO_Port,timingPin_Pin, GPIO_PIN_SET);
				HAL_Delay(delayTime);
				HAL_GPIO_WritePin(timingPin_GPIO_Port,timingPin_Pin, GPIO_PIN_RESET);
				HAL_Delay(delayTime);
			}
			HAL_Delay(3*delayTime);
			break;

		case 10000: //Error code 4
			HAL_Delay(3*delayTime);
			for(int i = 0; i < 4 ; i++)
			{
				HAL_GPIO_WritePin(timingPin_GPIO_Port,timingPin_Pin, GPIO_PIN_SET);
				HAL_Delay(delayTime);
				HAL_GPIO_WritePin(timingPin_GPIO_Port,timingPin_Pin, GPIO_PIN_RESET);
				HAL_Delay(delayTime);
			}
			HAL_Delay(3*delayTime);
			break;

		case 1000: //Error code 5
			HAL_Delay(3*delayTime);
			for(int i = 0; i < 5 ; i++)
			{
				HAL_GPIO_WritePin(timingPin_GPIO_Port,timingPin_Pin, GPIO_PIN_SET);
				HAL_Delay(delayTime);
				HAL_GPIO_WritePin(timingPin_GPIO_Port,timingPin_Pin, GPIO_PIN_RESET);
				HAL_Delay(delayTime);
			}
			HAL_Delay(3*delayTime);
			break;

		case 100:  //Error code 6
			HAL_Delay(3*delayTime);
			for(int i = 0; i < 6 ; i++)
			{
				HAL_GPIO_WritePin(timingPin_GPIO_Port,timingPin_Pin, GPIO_PIN_SET);
				HAL_Delay(delayTime);
				HAL_GPIO_WritePin(timingPin_GPIO_Port,timingPin_Pin, GPIO_PIN_RESET);
				HAL_Delay(delayTime);
			}
			HAL_Delay(3*delayTime);
			break;

		case 10:   //Error code 7
			HAL_Delay(3*delayTime);
			for(int i = 0; i < 7 ; i++)
			{
				HAL_GPIO_WritePin(timingPin_GPIO_Port,timingPin_Pin, GPIO_PIN_SET);
				HAL_Delay(delayTime);
				HAL_GPIO_WritePin(timingPin_GPIO_Port,timingPin_Pin, GPIO_PIN_RESET);
				HAL_Delay(delayTime);
			}
			HAL_Delay(3*delayTime);
			break;

		case 1:  //Error code 8
			HAL_Delay(3*delayTime);
			for(int i = 0; i < 8 ; i++)
			{
				HAL_GPIO_WritePin(timingPin_GPIO_Port,timingPin_Pin, GPIO_PIN_SET);
				HAL_Delay(delayTime);
				HAL_GPIO_WritePin(timingPin_GPIO_Port,timingPin_Pin, GPIO_PIN_RESET);
				HAL_Delay(delayTime);
			}
			HAL_Delay(3*delayTime);
			break;

		case 0:
			break;

		default:
				HAL_GPIO_WritePin(timingPin_GPIO_Port,timingPin_Pin, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_GPIO_WritePin(timingPin_GPIO_Port,timingPin_Pin, GPIO_PIN_RESET);
				HAL_Delay(50);
			break;

	}
}
//**********************************************************************************************************************************************************
void goToSleep() //puts the board to sleep
{
	goToSleepFlag = 0; //reset flag
	HAL_GPIO_WritePin(CB_En_GPIO_Port,CB_En_Pin, GPIO_PIN_RESET); //disconnect load
	HAL_GPIO_WritePin(VC_En_GPIO_Port,VC_En_Pin, GPIO_PIN_RESET); //disable voltage measurements
	HAL_GPIO_WritePin(Temp_En_GPIO_Port,Temp_En_Pin, GPIO_PIN_RESET); //disconnect thermistors
	HAL_GPIO_WritePin(BalC1_GPIO_Port,BalC1_Pin, GPIO_PIN_RESET); //stop discharging Cell 1
	HAL_GPIO_WritePin(BalC2_GPIO_Port,BalC2_Pin, GPIO_PIN_RESET); //stop discharging Cell 2
	HAL_GPIO_WritePin(BalC3_GPIO_Port,BalC3_Pin, GPIO_PIN_RESET); //stop discharging Cell 3
	HAL_GPIO_WritePin(BalC4_GPIO_Port,BalC4_Pin, GPIO_PIN_RESET); //stop discharging Cell 4
	HAL_GPIO_WritePin(BalC5_GPIO_Port,BalC5_Pin, GPIO_PIN_RESET); //stop discharging Cell 5
	HAL_GPIO_WritePin(BalC6_GPIO_Port,BalC6_Pin, GPIO_PIN_RESET); //stop discharging Cell 6
	HAL_GPIO_WritePin(timingPin_GPIO_Port,timingPin_Pin, GPIO_PIN_RESET); //turn off green LED
	HAL_GPIO_WritePin(CANmode_GPIO_Port,CANmode_Pin, GPIO_PIN_SET); //pull CAN tranceiver mode setting pin high to set it to low power mode

	//stop the PWM going to the buck-boost converter MOSFETs
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1); //PMOS
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2); //NMOS

	HAL_SuspendTick(); //suspend sis tick in order for the sis tick interrupt not to wake the device up
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI); //enter stop mode by enabling low power regulator and waiting for an interrupt to wake up

	//EXITING SLEEP MODE
	SystemClock_Config(); //re-initialise all the clocks
	HAL_ResumeTick(); //resume so we can work with HAL_Delay
	HAL_UART_Receive_DMA(&huart3,TxRx_data,8); //re-enable DMA for UART
	//no need to re-enable anything else here because sleep calls the main once exited using the GO TO statement
}

//**********************************************************************************************************************************************************
static void CAN_FilterConfig(void) //configures the CANbus filter
{
  CAN_FilterConfTypeDef  sFilterConfig;
  sFilterConfig.FilterNumber = 0;					 //Specifies the filter which will be initialized
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;	 //Specifies the filter mode to be initialized
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; //Specifies the filter scale
  sFilterConfig.FilterIdHigh = 0x000 <<5;  //0x124   //Specifies the filter identification number (MSBs for a 32-bit configuration, first one for a 16-bit configuration). shift by 5 is necessary
  sFilterConfig.FilterIdLow = 0x0000;                //Specifies the filter identification number (LSBs for a 32-bit configuration, second one for a 16-bit configuration).
  sFilterConfig.FilterMaskIdHigh = 0x0000; //0xFFE0	 //Specifies the filter mask number or identification number, according to the mode (MSBs for a 32-bit configuration,first one for a 16-bit configuration).
  sFilterConfig.FilterMaskIdLow = 0x0000;            //Specifies the filter mask number or identification number, according to the mode (LSBs for a 32-bit configuration, second one for a 16-bit configuration).
  sFilterConfig.FilterFIFOAssignment = 0;			 //Specifies the FIFO (0 or 1) which will be assigned to the filter.
  sFilterConfig.FilterActivation = ENABLE;			 //Enable or disable the filter
  sFilterConfig.BankNumber = 14;					 //Select the start slave bank filter

  if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }
}


//**********************************************************************************************************************************************************
//CALLBACKS--CALLBACKS--CALLBACKS--CALLBACKS--CALLBACKS--CALLBACKS--CALLBACKS--CALLBACKS--CALLBACKS--CALLBACKS--CALLBACKS--CALLBACKS--CALLBACKS--CALLBACKS--
//**********************************************************************************************************************************************************

//**********************************************************************************************************************************************************
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* CanHandle) //called when there is incoming traffic on the CAN bus
{
	//puts data received from CAN into a character buffer
	for (uint8_t i = 0; i < 8;i++)
	{
		CanRxData[i] = CanHandle->pRxMsg->Data[i];
	}

	//CAN code structure XXXNNNNN where XXX will be a 3 character opcode and NNNNN will be a 5 digit integer operand
	for (uint8_t i = 3; i < 8;i++)
	{
		parsedOperand[i-3] = CanHandle->pRxMsg->Data[i];//CanRxData[i];
		//CanRxData[i] = CanHandle->pRxMsg->Data[i];
	}

	operand = atoi(parsedOperand);
	HAL_CAN_Receive_IT(CanHandle, CAN_FIFO0);
	sprintf(msg,"BMSCharger MK2: message received from CAN: %c%c%c%c%c%c%c%c -- Opcode: %c%c%c -- Operand: %u \r\n",CanRxData[0],CanRxData[1],CanRxData[2],CanRxData[3],CanRxData[4],CanRxData[5],CanRxData[6],CanRxData[7],CanRxData[0],CanRxData[1],CanRxData[2],operand);
	HAL_UART_Transmit(&huart3,msg,strlen(msg),HAL_MAX_DELAY);

	opCode[0] = CanRxData[0];
	opCode[1] = CanRxData[1];
	opCode[2] = CanRxData[2];

	operationFromCan = 1;
	executeOperation();
}


//**********************************************************************************************************************************************************
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) //called when the DMA has filled the entire buffer
{
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_ADC_Stop_DMA(&hadc2);
	HAL_ADC_Stop_DMA(&hadc3);
	HAL_ADC_Stop_DMA(&hadc4);
}

//**********************************************************************************************************************************************************
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //core loop running on timer interrupt at 500 Hz
{
		static uint16_t counter = 0;
		static uint16_t cellBalanceCounter = 0;
		static uint8_t buckToBoostHandoverTimer = 0;
		static uint8_t balanceEnableTimer = 0;

		updateADC1Vars();
		updateADC2Vars();
		updateADC3Vars();
		updateADC4Vars();
		calculateActualValues();
		counter++;
		cellBalanceCounter++;

			if (counter == 500) //loop runs @ 1Hz
			{
				calculateTemps();
				healthCheck();

				//************************************************************************ SWITCHES FROM BUCK TO BOOST *************************************************************
				counter = 0;

				//makes sure that the bucking current has been low for 10 sec, indicating that we can't buck anymore so as to switch to boost
				if (((IOutChg*1000) < (currentLimit/2)) && (boostEnable == 0) && (balancingComplete == 0) && (chargerEnable == 1))
				{
					buckToBoostHandoverTimer++;
					if (buckToBoostHandoverTimer == 10)
					{
					  buckToBoostHandoverTimer = 0;
					  initialisePIDController();
					  boostEnable = 1;
					}
				 }
				 else {buckToBoostHandoverTimer = 0;} //resets the timer

				//************************************************************************ DEALS WITH BALANCING *************************************************************
				//if current is less than c/10 and balancing is not complete and charger is enabled and voltage is below setPoint by 500mV or above setPoint by 50mV and 20 seconds have passed then enable balancing
				if  ( ((IOutChg*1000) < (currentLimit/10)) && ((IOutChg*1000) < (200)) && (balancingComplete == 0) && (chargerEnable == 1) && (VC6 > (setpoint_V - 0.500)) && (VC6 < (setpoint_V + 0.050)))
				 {
					balanceEnableTimer++;
					if (balanceEnableTimer == 20)
					{
						balanceEnableTimer = 0;
						balanceEnable = 1;
					}
				 }
				else{balanceEnableTimer = 0;}
			  }


			if ((cellBalanceCounter == 2500) && (balanceEnable == 1)  && (chargerEnable == 1)) //enable cell voltage measurements after 5 seconds for 1 cycle
			{
				blockCellVoltageUpdating = 0;
				cellBalanceCounter = 0;
			}

			//if charge current is one tenth the set point,
			if ((cellBalanceCounter == 1) && (chargerEnable == 1) && (balanceEnable == 1))
			{
				balanceCells();				 //configure the balancer, ie start bleeding current
				blockCellVoltageUpdating = 1; //stop updating voltage
			}

			//allows the cells to sit for 1 second without after 4 second of balancing have elapsed
			if ((cellBalanceCounter == 2000)&&(blockCellVoltageUpdating == 1) && (chargerEnable == 1))
			{
				HAL_GPIO_WritePin(BalC1_GPIO_Port,BalC1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(BalC2_GPIO_Port,BalC2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(BalC3_GPIO_Port,BalC3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(BalC4_GPIO_Port,BalC4_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(BalC5_GPIO_Port,BalC5_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(BalC6_GPIO_Port,BalC6_Pin, GPIO_PIN_RESET);
			}

			//since ADC runs in single shot with DMA, re-enable them a soon as you calculate vars
			 HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc1Buffer, adc1BufferSize);
			 HAL_ADC_Start_DMA(&hadc2,(uint32_t*)adc2Buffer, adc2BufferSize);
			 HAL_ADC_Start_DMA(&hadc3,(uint32_t*)adc3Buffer, adc3BufferSize);
			 HAL_ADC_Start_DMA(&hadc4,(uint32_t*)adc4Buffer, adc4BufferSize);

			 if ((chargerEnable == 1) && (balancingComplete == 0))
			 {
				 applyPID();
			 }
			 else
			 {
				 TIM1->CCR1 = 1001; //turn off NMOS
				 TIM1->CCR2 = 0; //turn off PMOS
			 }

}


//**********************************************************************************************************************************************************
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) //called when UART receives data
{
	//parser code structure XXXNNNNN where XXX will be a 3 character opcode and NNNNN will be a 5 digit integer operand
	for (uint8_t i = 3; i < 8;i++)
	{
		parsedOperand[i-3] = TxRx_data[i];
	}

	operand = atoi(parsedOperand);

	sprintf(msg,"BMSCharger MK2: message received from UART: %c%c%c%c%c%c%c%c -- Opcode: %c%c%c -- Operand: %u \r\n",TxRx_data[0],TxRx_data[1],TxRx_data[2],TxRx_data[3],TxRx_data[4],TxRx_data[5],TxRx_data[6],TxRx_data[7],TxRx_data[0],TxRx_data[1],TxRx_data[2],operand);
	HAL_UART_Transmit(&huart3,msg,strlen(msg),HAL_MAX_DELAY);
	opCode[0] = TxRx_data[0];
	opCode[1] = TxRx_data[1];
	opCode[2] = TxRx_data[2];

	HAL_UART_Receive_DMA(&huart3,TxRx_data,8); //re-enable DMA for UART

	executeOperation(); //call parser
}

//**********************************************************************************************************************************************************
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) //ISR runs when the PwrBtn push button switch is pressed
{
	if (GPIO_Pin == PwrBtn_Pin)
	{
		//reset error code
		errorCode = 0;
		battOverTempTrip  = 0;
		battUnderTempTrip = 0;
		chgrOverTempTrip = 0;
		chgrUnderTempTrip = 0;
		battOverVoltTrip = 0;
		battUnderVoltTrip = 0;
		battOverCurrTrip = 0;
		battRegenCurrTrip = 0;
		HAL_GPIO_WritePin(CB_En_GPIO_Port,CB_En_Pin, GPIO_PIN_SET); //re-connect load
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
