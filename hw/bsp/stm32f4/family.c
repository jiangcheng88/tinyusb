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

#include "stm32f4xx_hal.h"
#include "bsp/board.h"
#include "board.h"

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void OTG_FS_IRQHandler(void)
{
  tud_int_handler(0);
}

<<<<<<< HEAD
#define MIC_SAMPLE_FREQUENCY 48000
#define MIC_SAMPLES_PER_MS (MIC_SAMPLE_FREQUENCY/1000)  // == 48
#define MIC_NUM_CHANNELS    1
#define MIC_MS_PER_PACKET   1
#define MIC_SAMPLES_PER_PACKET (MIC_SAMPLES_PER_MS * MIC_MS_PER_PACKET) // == 48
uint16_t  record1[MIC_SAMPLES_PER_PACKET*2]={0};
#if 0
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_i2s2_rx;

// audio 



uint16_t  process1[MIC_SAMPLES_PER_PACKET];
uint16_t  _sendbuffer[MIC_SAMPLES_PER_PACKET];

// ======  F407VET6  //  


/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2s2_rx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */
 //TU_LOG2("get");
  /* USER CODE END DMA1_Stream3_IRQn 1 */
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
  /* USER CODE BEGIN SPI2_MspInit 0 */
#if CFG_TUSB_DEBUG >= 2
    TU_LOG2("HAL_I2S_MspInit");
#endif
  /* USER CODE END SPI2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2S2 GPIO Configuration
    PC2     ------> I2S2_ext_SD
    PC3     ------> I2S2_SD
    PB10     ------> I2S2_CK
    PB9     ------> I2S2_WS
    PC6     ------> I2S2_MCK
    */
    // GPIO_InitStruct.Pin = GPIO_PIN_2;
    // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    // GPIO_InitStruct.Pull = GPIO_NOPULL;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    // GPIO_InitStruct.Alternate = GPIO_AF6_I2S2ext;
    // HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2S2 DMA Init */
    /* I2S2_EXT_RX Init */
    hdma_i2s2_rx.Instance = DMA1_Stream3;
    hdma_i2s2_rx.Init.Channel = DMA_CHANNEL_0;
    hdma_i2s2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2s2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2s2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2s2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_i2s2_rx.Init.MemDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_i2s2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_i2s2_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_i2s2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    // hdma_i2s2_rx.Init.MemBurst  = DMA_MBURST_SINGLE ;
    // hdma_i2s2_rx.Init.PeriphBurst = DMA_PBURST_SINGLE ; 

    HAL_DMA_DeInit(&hdma_i2s2_rx);
    if (HAL_DMA_Init(&hdma_i2s2_rx) != HAL_OK)
    {
#if CFG_TUSB_DEBUG >= 2
    TU_LOG2("HAL_DMA_Init error ");
#endif
    }
    
    __HAL_LINKDMA(&hi2s2,hdmarx,hdma_i2s2_rx);
  
  /* USER CODE BEGIN SPI2_MspInit 1 */
  }
  /* USER CODE END SPI2_MspInit 1 */

}




/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}


/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_HIGH;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
#if CFG_TUSB_DEBUG >= 2
    TU_LOG2("HAL_I2S_Init error ");
#endif
    
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
* @brief I2S MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hi2s: I2S handle pointer
* @retval None
*/
void HAL_I2S_MspDeInit(I2S_HandleTypeDef* hi2s)
{
  if(hi2s->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**I2S2 GPIO Configuration
    PC2     ------> I2S2_ext_SD
    PC3     ------> I2S2_SD
    PB10     ------> I2S2_CK
    PB9     ------> I2S2_WS
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_2|GPIO_PIN_3);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_9);

    /* I2S2 DMA DeInit */
    HAL_DMA_DeInit(hi2s->hdmarx);
  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }

}
#endif 
=======
void OTG_HS_IRQHandler(void)
{
  tud_int_handler(1);
}

>>>>>>> edd8eb3279c2440e9d4590312f2104e58beafe12
//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+
UART_HandleTypeDef UartHandle;

void board_init(void)
{
  board_clock_init();
  //SystemCoreClockUpdate();

#if CFG_TUSB_OS == OPT_OS_NONE
  // 1ms tick timer
  SysTick_Config(SystemCoreClock / 1000);
#elif CFG_TUSB_OS == OPT_OS_FREERTOS
  // Explicitly disable systick to prevent its ISR runs before scheduler start
  SysTick->CTRL &= ~1U;

  // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
  NVIC_SetPriority(OTG_FS_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
#endif

  GPIO_InitTypeDef  GPIO_InitStruct;

  // LED
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

  board_led_write(false);


  // /*Configure GPIO pin : PD7 */
  // GPIO_InitStruct.Pin = GPIO_PIN_7;
  // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  // HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, 1);

  // Button
  GPIO_InitStruct.Pin = BUTTON_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = BUTTON_STATE_ACTIVE ? GPIO_PULLDOWN : GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);

#ifdef UART_DEV
  // UART
  GPIO_InitStruct.Pin       = UART_TX_PIN | UART_RX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = UART_GPIO_AF;
  HAL_GPIO_Init(UART_GPIO_PORT, &GPIO_InitStruct);

  UartHandle = (UART_HandleTypeDef){
    .Instance        = UART_DEV,
    .Init.BaudRate   = CFG_BOARD_UART_BAUDRATE,
    .Init.WordLength = UART_WORDLENGTH_8B,
    .Init.StopBits   = UART_STOPBITS_1,
    .Init.Parity     = UART_PARITY_NONE,
    .Init.HwFlowCtl  = UART_HWCONTROL_NONE,
    .Init.Mode       = UART_MODE_TX_RX,
    .Init.OverSampling = UART_OVERSAMPLING_16
  };
  HAL_UART_Init(&UartHandle);
#endif

  /* Configure USB FS GPIOs */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Configure USB D+ D- Pins */
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure VBUS Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ID Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

#ifdef STM32F412Zx
  /* Configure POWER_SWITCH IO pin */
  __HAL_RCC_GPIOG_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
#endif

  // Enable USB OTG clock
  __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

//  __HAL_RCC_USB_OTG_HS_CLK_ENABLE();

  board_vbus_sense_init();


//    MX_DMA_Init();

//    MX_I2S2_Init();

// HAL_StatusTypeDef ret  =HAL_I2S_Receive_DMA(&hi2s2,record1,MIC_SAMPLES_PER_PACKET*2);
// (void) ret ;
// #if CFG_TUSB_DEBUG >= 2
//     TU_LOG2("------------  = %d\n ",ret);
//     //TU_LOG2("hi2s2.ErrorCode = %ld \n",hi2s2.ErrorCode);
// #endif

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
#ifdef UART_DEV
  HAL_UART_Transmit(&UartHandle, (uint8_t*)(uintptr_t) buf, len, 0xffff);
  return len;
#else
  (void) buf; (void) len; (void) UartHandle;
  return 0;
#endif
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

int RxHalfComplete_Flag=0;
int RxComplete_Flag=0;

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
//   #if CFG_TUSB_DEBUG >= 2
//     TU_LOG2("HAL_I2S_RxHalfCpltCallback  status  = %d ",hi2s->State);
// #endif
  (void) hi2s;
	RxHalfComplete_Flag =1;
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
// #if CFG_TUSB_DEBUG >= 2
//     TU_LOG2("HAL_I2S_RxCpltCallback  status  = %d ",hi2s->State);
    
// #endif
  (void) hi2s;
	RxComplete_Flag=1;
}


#define HTONS(A)  ((((uint16_t)(A) & 0xff00) >> 8) | (((uint16_t)(A) & 0x00ff) << 8))

uint16_t  usb_pcm[MIC_SAMPLES_PER_PACKET * 2] = {0};
uint16_t  usb_pcm1[MIC_SAMPLES_PER_PACKET * 2] = {0};
volatile uint8_t   can_send1 = 0 ;
uint8_t   can_send2 = 0 ;
uint32_t  cnt =0 ;
void half_function(void){
  
 uint16_t index =  0 ; 
 // uint16_t temp = 0;
  for(index = 0 ; index < MIC_SAMPLES_PER_PACKET  ; index ++)
  {
   //   temp  = (uint16_t )((record1[index]>>8));
  //  process1[index] = temp;
  //  usb_pcm[index] = record1[index+buffer_size/2];
    //   uint8_t mid = (((record1[index]|0xFFFFFFFF)>>16)&(0xFF));
    //   uint8_t msb = (((record1[index]|0xFFFFFFFF)>>24)&(0xFF));
    //   usb_pcm[index] = (((uint32_t)msb<<8)| ((uint32_t)mid));
    //  // usb_pcm[index] = 0xFFFF;
    //   TU_LOG2("p=%x r= %lx\n",usb_pcm[index],record1[index]);
      usb_pcm[index] =HTONS(record1[index]);
  }
    can_send1  = 1;
 // tud_audio_write((uint8_t *)usb_pcm, MIC_SAMPLES_PER_PACKET);
}

void comp_function(void){
   
  uint16_t index =  0 ; 
  //uint16_t temp = 0;
  for(index = 0 ; index < MIC_SAMPLES_PER_PACKET   ; index ++)
  {
   // temp  = (uint16_t )((record1[index]));
  //  usb_pcm1[index] = record1[index];
  //  TU_LOG2("%d\n",usb_pcm1[index]);
  //    usb_pcm1[index] = 0x0000;
    usb_pcm1[index] = HTONS(record1[index+MIC_SAMPLES_PER_PACKET]);
  }
   can_send2 = 1;
 // tud_audio_write((uint8_t *)usb_pcm1, buffer_size / 2);
}

void send_pcm(void)
{
  tud_audio_write((uint8_t *)usb_pcm, MIC_SAMPLES_PER_PACKET);
  can_send1  = 0;
}

void send_pcm1(void)
{
  tud_audio_write((uint8_t *)usb_pcm1, MIC_SAMPLES_PER_PACKET);
  can_send2  = 0;
}