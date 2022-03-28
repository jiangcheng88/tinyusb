/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "../board.h"
#include "stm32f3xx_hal.h"

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+

// USB defaults to using interrupts 19, 20 and 42, however, this BSP sets the
// SYSCFG_CFGR1.USB_IT_RMP bit remapping interrupts to 74, 75 and 76.

// FIXME: Do all three need to be handled, or just the LP one?
// USB high-priority interrupt (Channel 74): Triggered only by a correct
// transfer event for isochronous and double-buffer bulk transfer to reach
// the highest possible transfer rate.
void USB_HP_IRQHandler(void)
{
  tud_int_handler(0);
}

// USB low-priority interrupt (Channel 75): Triggered by all USB events
// (Correct transfer, USB reset, etc.). The firmware has to check the
// interrupt source before serving the interrupt.
void USB_LP_IRQHandler(void)
{
  tud_int_handler(0);
}

// USB wakeup interrupt (Channel 76): Triggered by the wakeup event from the USB
// Suspend mode.
void USBWakeUp_RMP_IRQHandler(void)
{
  tud_int_handler(0);
}

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+
UART_HandleTypeDef UartHandle;


//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+

#define LED_PORT              GPIOE
#define LED_PIN               GPIO_PIN_9
#define LED_STATE_ON          1

#define BUTTON_PORT           GPIOA
#define BUTTON_PIN            GPIO_PIN_0
#define BUTTON_STATE_ACTIVE   1

#define UART_PORT             GPIOC
#define TX_PIN                GPIO_PIN_4
#define RX_PIN                GPIO_PIN_5 




/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s2;

DMA_HandleTypeDef hdma_spi2_rx;


/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_rx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */
// #if CFG_TUSB_DEBUG >= 2
//     TU_LOG2("DMA1_Channel4_IRQHandler SPI\r\n");
// #endif
  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

void time_sleep_ms(uint32_t time) // ms
{
  uint32_t start_ms =board_millis();

  while(  board_millis() - start_ms < time){
  };
}


/**
* @brief I2S MSP Initialization
* This function configures the hardware resources used in this example
* @param hi2s: I2S handle pointer
* @retval None
*/
void HAL_I2S_MspInit(I2S_HandleTypeDef* hi2s)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hi2s->Instance==SPI2)
  {
      
#if CFG_TUSB_DEBUG >= 2
    TU_LOG2("HAL_I2S_MspInit SPI\r\n");
#endif
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**I2S2 GPIO Configuration
    PB12     ------> I2S2_WS
    PB13     ------> I2S2_CK
    PB15     ------> I2S2_SD
    PC6     ------> I2S2_MCK
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  }

 // __HAL_RCC_SPI2_FORCE_RESET();

  /* Release the I2S peripheral clock reset */
 // __HAL_RCC_SPI2_RELEASE_RESET();

 


  /* Configure the hdma_i2sRx handle parameters */   
  hdma_spi2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY ; 
  hdma_spi2_rx.Init.PeriphInc  = DMA_PINC_DISABLE ; 
  hdma_spi2_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_spi2_rx.Init.PeriphDataAlignment =  DMA_PDATAALIGN_HALFWORD ; 
  hdma_spi2_rx.Init.MemDataAlignment = DMA_PDATAALIGN_HALFWORD ; 
  hdma_spi2_rx.Init.Mode = DMA_CIRCULAR ;
  hdma_spi2_rx.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_spi2_rx.Instance = DMA1_Channel4;

  if ( HAL_DMA_Init(&hdma_spi2_rx) != HAL_OK)
  {
    while(1){
#if CFG_TUSB_DEBUG >= 2
    TU_LOG2("HAL_DMA_Init ");
#endif
    }
  }

  __HAL_LINKDMA(&hi2s2, hdmarx, hdma_spi2_rx);


}


/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_MSB;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_32K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_SYSCLK;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    while(1);

  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}


void MX_DMA_Init(void)
{
   /* Enable the I2S DMA clock */
  __HAL_RCC_DMA1_CLK_ENABLE(); 
    /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}



/**
* @brief I2S MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hi2s: I2S handle pointer
* @retval None
*/
void HAL_I2S_MspDeInit(I2S_HandleTypeDef* hi2s)
{
    
#if CFG_TUSB_DEBUG >= 2
    TU_LOG2("HAL_I2S_MspDeInit " );
#endif
  if(hi2s->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**I2S2 GPIO Configuration
    PB12     ------> I2S2_WS
    PB13     ------> I2S2_CK
    PB15     ------> I2S2_SD
    PC6     ------> I2S2_MCK
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15);

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6);

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }

}
#define buffer_size  32

uint16_t  record[buffer_size]={0};

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            HSE PREDIV                     = 1
  *            PLLMUL                         = RCC_PLL_MUL9 (9)
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  RCC_PeriphClkInit;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Configures the USB clock */
  HAL_RCCEx_GetPeriphCLKConfig(&RCC_PeriphClkInit);
  RCC_PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;

  //RCC_PeriphClkInit.I2sClockSelection =  RCC_PERIPHCLK_I2S;

    
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  /* Enable Power Clock */
  __HAL_RCC_PWR_CLK_ENABLE();
}

void board_init(void)
{
  SystemClock_Config();

  MX_GPIO_Init();
  #if CFG_TUSB_OS  == OPT_OS_NONE
  // 1ms tick timer
  SysTick_Config(SystemCoreClock / 1000);
  #endif

  // Remap the USB interrupts
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_REMAPINTERRUPT_USB_ENABLE();

 // UART
  __HAL_RCC_GPIOC_CLK_ENABLE(); 
  GPIO_InitTypeDef GPIO_InitStruct ; 
  GPIO_InitStruct.Pin = (RX_PIN| TX_PIN) ; 
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL ; 
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1 ;
  HAL_GPIO_Init(UART_PORT , &GPIO_InitStruct);
  __HAL_RCC_USART1_CLK_ENABLE();


  UartHandle.Instance = USART1 ; 
  UartHandle.Init.BaudRate = CFG_BOARD_UART_BAUDRATE;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B ;
  UartHandle.Init.StopBits =  UART_STOPBITS_1 ; 
  UartHandle.Init.Parity = UART_PARITY_NONE ; 
  UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE ; 
  UartHandle.Init.Mode = UART_MODE_TX_RX ; 
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16 ;
  HAL_UART_Init(&UartHandle);



  // LED
  __HAL_RCC_GPIOE_CLK_ENABLE();
  //GPIO_InitTypeDef  GPIO_InitStruct ;
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

  // Button
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = BUTTON_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);

  /* Configure USB DM and DP pins */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = (GPIO_PIN_11 | GPIO_PIN_12);
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF14_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Enable USB clock
  __HAL_RCC_USB_CLK_ENABLE();
  

  MX_DMA_Init();
  //__HAL_RCC_DMA1_CLK_ENABLE();
  MX_I2S2_Init();
  time_sleep_ms(1000);
  HAL_StatusTypeDef ret = HAL_I2S_Receive_DMA(&hi2s2,record,buffer_size);

  
#if CFG_TUSB_DEBUG >= 2
    TU_LOG2("------------  = %d\n ",ret);
    //TU_LOG2("hi2s2.ErrorCode = %ld \n",hi2s2.ErrorCode);
#endif

}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, state ? LED_STATE_ON : (1-LED_STATE_ON));
}

uint32_t board_button_read(void)
{
  return BUTTON_STATE_ACTIVE == HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN);
}

int board_uart_read(uint8_t* buf, int len)
{
  (void) buf; (void) len;
  return 0;
}

int board_uart_write(void const * buf, int len)
{
  HAL_UART_Transmit(&UartHandle, (uint8_t*) buf, len, 0xffff);
  return 0;
}

#if CFG_TUSB_OS  == OPT_OS_NONE
volatile uint32_t system_ticks = 0;
void SysTick_Handler (void)
{
  system_ticks++;
}

uint32_t board_millis(void)
{
  return system_ticks;
}
#endif

void HardFault_Handler (void)
{
  asm("bkpt");
}

// Required by __libc_init_array in startup code if we are compiling using
// -nostdlib/-nostartfiles.
void _init(void)
{

}


uint16_t  usb_pcm[buffer_size] = {0};
uint16_t  usb_pcm1[buffer_size] = {0};
void half_function(void){
  
  uint16_t index =  0 ; 
  for(index = 0 ; index < buffer_size/2 ; index ++)
  {
    usb_pcm[index] = record[index+buffer_size/2];
  }
//   tud_audio_write((uint8_t *)usb_pcm, buffer_size / 2);

}

void comp_function(void){
  
  uint16_t index =  0 ; 
  for(index = 0 ; index < buffer_size/2 ; index ++)
  {
    usb_pcm1[index] = record[index];
  }
//  tud_audio_write((uint8_t *)usb_pcm1, buffer_size / 2);
}



int RxHalfComplete_Flag=0;
int RxComplete_Flag=0;

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
//   #if CFG_TUSB_DEBUG >= 2
//     TU_LOG2("HAL_I2S_RxHalfCpltCallback  status  = %d ",hi2s->State);
// #endif
	RxHalfComplete_Flag =1;
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
// #if CFG_TUSB_DEBUG >= 2
//     TU_LOG2("HAL_I2S_RxCpltCallback  status  = %d ",hi2s->State);
    
// #endif
	RxComplete_Flag=1;
}



