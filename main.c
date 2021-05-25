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
	This code supports NeoPill, a Neopixel Emulator, where connecting the Neopixel serial data from a device (an Arduino dev board, for example)
	to a STM32 dev board converts the Neopixel serial data to USB serial data (a Neopixel->USB bridge.)
	A PC runs python code that takes this USB data and converts it to colored objects on the screen.
	
	This code is specific for the STM32F103C8T6 BluePill.
	
	Lower level:
	Read Neopixel serial data using SPI, where the pixel bitstream is fed into both MOSI and TIM4.
	TIM4 then generates a one-shot pulse (CH1 out) wired to SCK.
	The pulse is configured to match the bitstream timing T0H, T1H of <500ns = 0, >550ns = 1. 
	CH2 in delay is 3 clks (Output compare fast enable), and CH1 out has resolution of 13.88ns (72MHz sys clock, Prescaler = 0) per count.
	So each data bit generates a SCK pulse about 550ns duration.
	
	For 72MHz clock (13.88ns):
	TIM4 ARR for one pulse mode:
	550ns - (3*13.88) = 508.33ns
	508.33ns/13.88 = 36.6 counts min. Make this 37 (555ns) for TIM4 ARR.
	
	
	SPI data bytes are DMA'd to a buffer, and sent out USB.
	Input rate testing never got higher than 64kbytes/s, using Arduino Nano target. Max rate should be around 100kbytes/s.
	
	Neopixel End of Data frame (EoF) (causing each pixel to latch) is 6us or longer of low level, typically stated as >50us. 
	TIM3 is clocked from TIM4 ITR3, divided by 8, and is essentially a byte counter of the data going into the SPI. 
	Every 8 bits of Neopixel data TIM3's ISR starts/restarts TIM2 one-pulse delay. Upon TIM2 one-pulse timeout an EoF ISR triggers.
	
	TIM2 Pulse delay:
	50us/13.88ns = 3602 counts.
	
	The EoF ISR is used to initially sync the SPI data out the USB, assuming the PC side is ready to accept the data. Once sync'd
	and running there is no further syncing.
	
	For the PC to sync/re-sync with the Neopixel data frames:
	  1) send a one character 'F' to flush the USB (in case re-syncing) and reset the SPI interface. 
		2) the PC flushes its USB data as well.
		3) send a one character 'S' to start the data stream after the last EoF. 
		
	For tweaking TIM2, TIM4 timer values a simple text string from the PC is supported:
		'T nnn mmm'
		see pc_host_timing_req() below.
		
  Environment:
	  CubeMX 6.1.1
	  Keil uVision 5.26
		BluePill STM32F103C8T6, 16kRAM,64kFlash, 72MHz clk.
		All code in main.c, with small mods in usbd_cdc_if.c, usbd_cdc_if.h
	
	
	
	BluePill pinout/timers:
	
	PB7 TIM4 CH2 input <-serial Neopixel data from external dev bd.
	
	PB6 TIM4 CH1 output -> generates SPI2_SCK (to PB13), use jumper.
			TIM3 CH1 SPI2 input byte counter (no pins used)
			TIM2 CH1 -> frame end ITR (no pins used)
	
	PB15 SPI2 MOSI input, Rx DMA1 CH4, <-serial Neopixel data from external dev bd.
	PB13 SPI2 SCK input <- TIM4 CH1 (from PB6)
	
	LEDs
	PC13 LED_Pin (built-in): red heartbeat, various speeds.
	
	On PC side, set COM baud rate to max 12,000,000 .
	
	R. Elwin 5/2021
	
  */
	
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <limits.h>
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
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
extern uint8_t CDC_Flush_FS(void);

#define SPI_BSIZE	8192							// also look at USB's APP_TX_DATA_SIZE
uint8_t spi_rx_buffer[SPI_BSIZE];

//items used in an ISR
volatile uint32_t spi_rx_ctr;
volatile uint32_t tim2_ch1_isr_ctr;		//EoF ISR counter
volatile uint32_t frame_byte_ctr;
volatile uint8_t usb_sync_flag,usb_flush_flag;

//helpful debugging aids
uint32_t dbg_spi_isr, dbg_rx_rate;
uint32_t dbg_spi_err, dbg_spi_state;
uint32_t dbg_tim3_ch1_isr;
uint32_t dbg_dma_ctr;
uint32_t dbg_frame_overflow;
uint32_t dbg_tim3_cnt;

//PC host changes default input timings if >0
uint16_t pchost_tim4_Period;
uint16_t pchost_tim2_Pulse;

//USB counters, also helpful for debugging
uint32_t usb_fail;
uint32_t usb_busy;
uint32_t usb_bytes_tx_ctr;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//calcs difference between 2 timestamps -- just uint w/rollover.
// ts > last_ts
uint32_t calc_ts_et(uint32_t ts, uint32_t last_ts)
{
  if ( ts >= last_ts )
  {
    return ( ts - last_ts );
  }
  else
  {
    return ( 1 + ts - (ULONG_MAX - last_ts) );
  }
}

/*
Timing change request from PC, in text, 'T nnn mmm'
where:
nnn= T1H bit timing in clks.
mmm= TLL (or reset latch) End of Frame timeout pulse in counts.

(see notes at top)
For 72MHz clock (13.88ns)
ETR -> CH2 delay is 3 clks
TIM2 ARR for one-shot pulse for T1H of 550ns:
	550ns - (3*13.88) = 508.33ns
	508.33ns/13.88 = 36.6 counts min. Make this 38 for TIM4 Period.

TLL's TIM2 Pulse defaults to 3602 counts (50us). TIM2 Period > Pulse (I made this arbitrarily 50.)

*/
void pc_host_timing_req(uint8_t* Buf, uint32_t Len)
{
	char in_str[16];
	int i,j,n;
	uint16_t t1h, tll;
	
	//strip leading 'T', then '\0-terminate
	for( i=0, j=1; j<Len && i<sizeof(in_str)-1; i++, j++)
	{
		in_str[i] = Buf[j];
	}
	in_str[i] = '\0';
	n = sscanf( in_str, "%hu %hu", &t1h, &tll);
	
	//save if all's well
	if( n == 2 )
	{
		pchost_tim4_Period = t1h;
		pchost_tim2_Pulse = tll;
	}
}


//for SPI DMA circular buffer, Intr when wrap -- used to track overflow.
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	spi_rx_ctr += SPI_BSIZE;		//another block read
}

//for testing
void HAL_SPI_ErrorCallback (SPI_HandleTypeDef * hspi)
{
	dbg_spi_err = HAL_SPI_GetError(hspi);							//just returns hspi field, not register
	dbg_spi_state = HAL_SPI_GetState(hspi);
}


/*
TIMER Pulse complete ISR, both TIM2 and TIM3.
Be careful of which channel generates INTR as HAL may enable multiple channels.
Also be careful of ISR latency as the SPI input bit stream is about 10us per byte.

SPI input bit stream is counted by TIM3. Every 8 counts this ISR resets/restarts TIM2 one-shot.
Upon TIM2 one-shot completion, its ISR indicates an End of Frame.

Upon EoF TIM3, TIM4 counters are reset in case we started out of sync with the Neopixel bitstream.
*/

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	
	if( &htim3 == htim )
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // we're using TIM3 channel 1
		{
			//input SPI byte counter triggers this intr, resets TIM2 one-shot
			dbg_tim3_ch1_isr++;
			
			htim2.Instance->EGR |= TIM_EGR_UG;				// UG=1, clears counter, CEN =0
			htim2.Instance->CR1 |= TIM_CR1_CEN;				// CEN =1, now running , or use macro	__HAL_TIM_ENABLE(&htim4);	
		}
	}
	else if( &htim2 == htim )
	{
		// TIM2 one-shot expires indicating End of Frame (EoF)
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  // we're using TIM2 channel 2
		{
			uint16_t d;
			
			dbg_tim3_cnt+=htim3.Instance->CNT;		//test, see if TIM3 clocked during last delay. This indicates we're out of sync.
			
			//htim3.Instance->CR1 |= TIM_CR1_UDIS;		//URS=1, so no UG INT
			htim3.Instance->EGR |= TIM_EGR_UG;				// UG=1, clears TIM3 counter, in case we started out of sync with bitstream
			htim4.Instance->EGR |= TIM_EGR_UG;				//clear TIM4 too
			
			
			tim2_ch1_isr_ctr++;
			
			//this checks if we're overflowing SPI buffer -- for debug only
			frame_byte_ctr = spi_rx_ctr;
			d = __HAL_DMA_GET_COUNTER( &hdma_spi2_rx );
			if( d > 0 )
				frame_byte_ctr += (SPI_BSIZE - d);	// convert to byte counter with DMA register CNDTR, this is our latest EoF 
			
			if( frame_byte_ctr >= (usb_bytes_tx_ctr + SPI_BSIZE) )		//check if overflowing SPI buffer
				dbg_frame_overflow++;																		//debug counter
			
		}
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
	uint32_t led_heartbeat = 0;
	uint16_t dmaout_idx;
	uint16_t dmain_ctr, dmaout_ctr;
	uint16_t n, next_ctr;
	int e;
	uint8_t usb_sync_flag2, usb_flush_flag2;
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
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	
	
	usb_sync_flag2 = usb_sync_flag;		//init flags
		
	
	/****** resync ************/
	//In case PC re-syncs, re-init TIMers, SPI. Don't Touch USB as this causes COM port grief!
	label_resync:		
	
	//undo/redo SPI 
	__HAL_DMA_DISABLE_IT( &hdma_spi2_rx, DMA_IT_TC );
	HAL_SPI_DeInit (&hspi2);
	MX_SPI2_Init();
	
	//turn off TIM 2,3,4
	HAL_TIM_OnePulse_Stop(&htim4,TIM_CHANNEL_1);
	HAL_TIM_OnePulse_Stop_IT (&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);	
	
	//de-init'em
	HAL_TIM_OnePulse_DeInit(&htim2);
	HAL_TIM_OnePulse_DeInit(&htim3);
	HAL_TIM_IC_DeInit(&htim4);	
	
	//re-do'em
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	
	
	/******** end resync ***********/
	
	//wait for PC sync command
	while( usb_sync_flag2 == usb_sync_flag )
	{
		if( calc_ts_et( HAL_GetTick(), led_heartbeat ) >= 100 )		//fast pulse
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port , LED_Pin );
			led_heartbeat = HAL_GetTick();
		}
	}
	
	usb_sync_flag2 = usb_sync_flag;		//done w/resync
	usb_flush_flag2 = usb_flush_flag;	//in case flushed
	
	//'manually' change timings if requested
	if( pchost_tim4_Period )
	{
		htim4.Instance->ARR = pchost_tim4_Period;
	}
	if( pchost_tim2_Pulse )
	{	
		htim2.Instance->CCR1 = pchost_tim2_Pulse;
		htim2.Instance->ARR = pchost_tim2_Pulse + 50;  //ARR > CCR1
	}
	
	
	// start TIM 2,3,4 
	HAL_TIM_OnePulse_Start (&htim4, TIM_CHANNEL_1);				//SCK generator 
	HAL_TIM_OnePulse_Start_IT (&htim2, TIM_CHANNEL_2);		//Frame timer, ARR is duration of TMO (CCR1+1), CCR1 = TMO delay
	htim2.Instance->EGR |= TIM_EGR_UG;										// UG=1, clears counter, CEN =0, let TIM3 ISR restart TIM2 to sync properly
	__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);							//HAL turns on both CH1,2, turn off CH1 intr
	tim2_ch1_isr_ctr = 0;																	//reset before enabling TIM2 ISR
	
	HAL_TIM_IC_Start_IT (&htim3, TIM_CHANNEL_1);					//start SPI byte counter
	htim3.Instance->CR1 |= TIM_CR1_URS;										//only CNT generates INT
	
			
	//wait for Eof to sync to Neopixel bitstream
	while( tim2_ch1_isr_ctr == 0 )
	{
		if( calc_ts_et( HAL_GetTick(), led_heartbeat ) >= 200 )		//less fast pulse, in case SPI bitstream stopped.
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port , LED_Pin );
			led_heartbeat = HAL_GetTick();
		}
	}
	
	//Will begin a new frame, reset input counters before next frame starts. 
	__disable_irq();
	frame_byte_ctr = 0;
	spi_rx_ctr = 0;		
	usb_bytes_tx_ctr = 0;	
	dmaout_ctr = SPI_BSIZE;
	__enable_irq();
	
	//start SPI Rx
	HAL_SPI_Receive_DMA ( &hspi2, spi_rx_buffer, SPI_BSIZE);
	__HAL_DMA_ENABLE_IT( &hdma_spi2_rx, DMA_IT_TC );
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/* 
		Read SPI DMA downcounter, if changed convert to index, determine bytes to send to USB.
		Assumes did not wrap from the last read, otherwise buffer overrun (dbg_frame_overflow).
		Ignore CNDTR=0 as this means counter reload in progress -- not sure if our code ever sees this.
		*/
		
		dmain_ctr = __HAL_DMA_GET_COUNTER( &hdma_spi2_rx );	// DMA register CNDTR is decremented
		
		if( (dmain_ctr > 0) && (dmain_ctr != dmaout_ctr) )
		{
			if( dmaout_ctr > dmain_ctr )
			{
				n = dmaout_ctr - dmain_ctr;
				next_ctr = dmain_ctr;
			}
			else // dmain_ctr > dmaout_ctr, circular wrap occured, in_cnt =>0 then reloads to SPI_BSIZE
			{
				//do 2 transfers, 1st empties remaining buffer, then wraps, then perform this loop again next poll
				n = dmaout_ctr;				//remaining bytes
				next_ctr = SPI_BSIZE;	//reload out ctr
			}
			dmaout_idx = SPI_BSIZE - dmaout_ctr;	//convert to index 0..SPI_BSIZE-1
			dmaout_ctr = next_ctr;
						
			//blocking write to USB if waiting for previous transfer, if PC not connected/disconnects we'll overflow and stay in this loop.
			usb_busy = 0;
			while( (e = CDC_Transmit_FS( &spi_rx_buffer[dmaout_idx], n) ) != USBD_OK)
			{
				if( usb_flush_flag2 != usb_flush_flag )
				{	
					//PC wants to re-sync, while we're waiting
					CDC_Flush_FS();
					goto label_resync;
				}
				
				if(e == USBD_BUSY)
				{
					usb_busy++;						//in case waiting for last packet
				}
				else
					usb_fail++;						//some low-level failure, fairly rare.
			}
			
			
			if( e == USBD_OK )
			{
				usb_bytes_tx_ctr += n;	//this may wrap
			}
		}
		
		if( usb_flush_flag2 != usb_flush_flag )
		{	
			//PC wants to re-sync
			CDC_Flush_FS();
			goto label_resync;
		}
		
		//normal heartbeat LED
		if( calc_ts_et( HAL_GetTick(), led_heartbeat ) >= 500 )
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port , LED_Pin );
			led_heartbeat = HAL_GetTick();
		}
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3650;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 3602;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 7;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR3;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 37;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim4, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
