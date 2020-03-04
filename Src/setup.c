/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
tim1 master, enable -> trgo
tim8, gated slave mode, trgo by tim1 trgo. overflow -> trgo
adc1,adc2 triggered by tim8 trgo
adc 1,2 dual mode

ADC1             ADC2
R_Blau PC4 CH14  R_Gelb PC5 CH15
L_Grün PA0 CH01  L_Blau PC3 CH13
R_DC PC1 CH11    L_DC PC0 CH10
BAT   PC2 CH12   L_TX PA2 CH02
BAT   PC2 CH12   L_RX PA3 CH03

pb10 usart3 dma1 channel2/3
*/

#include "defines.h"
#include "config.h"
#include "comms.h"
#include <string.h>

TIM_HandleTypeDef htim_right;
TIM_HandleTypeDef htim_left;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

DMA_HandleTypeDef hdma_usart2_rx; // unused, tx only
DMA_HandleTypeDef hdma_usart2_tx;

volatile adc_buf_t adc_buffer;


void UART2_Init(void) {


  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
 
  huart2.Instance          = USART2;
  huart2.Init.BaudRate     = USART2_BAUD;
  huart2.Init.WordLength   = USART2_WORDLENGTH;
  huart2.Init.StopBits     = UART_STOPBITS_1;
  huart2.Init.Parity       = UART_PARITY_NONE;
  huart2.Init.Mode         = UART_MODE_TX;
  huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16; 

  HAL_UART_Init(&huart2);

  // USART2->CR3 |= USART_CR3_DMAT;  // | USART_CR3_DMAR | USART_CR3_OVRDIS;

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin   = GPIO_PIN_2;
  GPIO_InitStruct.Pull  = GPIO_PULLUP; //GPIO_NOPULL;
  GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
/* UART2_TX Init */
  hdma_usart2_tx.Instance                   = DMA1_Channel7;
  hdma_usart2_tx.Init.Direction             = DMA_MEMORY_TO_PERIPH;
  hdma_usart2_tx.Init.PeriphInc             = DMA_PINC_DISABLE;
  hdma_usart2_tx.Init.MemInc                = DMA_MINC_ENABLE;
  hdma_usart2_tx.Init.PeriphDataAlignment   = DMA_PDATAALIGN_BYTE;
  hdma_usart2_tx.Init.MemDataAlignment      = DMA_MDATAALIGN_BYTE;
  hdma_usart2_tx.Init.Mode                  = DMA_NORMAL;
  hdma_usart2_tx.Init.Priority              = DMA_PRIORITY_LOW;

  HAL_DMA_Init(&hdma_usart2_tx);

  __HAL_LINKDMA(&huart2, hdmatx, hdma_usart2_tx);
  // DMA1_Channel7->CCR   = 0;
  // DMA1_Channel7->CPAR  = (uint32_t) & (USART2->DR);
  // DMA1_Channel7->CNDTR = 0;
  // DMA1_Channel7->CCR   = DMA_CCR_MINC | DMA_CCR_DIR;
  // DMA1->IFCR           = DMA_IFCR_CTCIF7 | DMA_IFCR_CHTIF7 | DMA_IFCR_CGIF7;

  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);                                        
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_TXE);

}


void UART3_Init(void) {

  memset((void *)&usart3_it_TXbuffer, 0, sizeof(usart3_it_TXbuffer));
  memset((void *)&usart3_it_RXbuffer, 0, sizeof(usart3_it_RXbuffer));

  memset(&huart3, 0, sizeof(huart3));

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_USART3_CLK_ENABLE();

  huart3.Instance          = USART3;
  huart3.Init.BaudRate     = USART3_BAUD;
  huart3.Init.WordLength   = USART3_WORDLENGTH;
  huart3.Init.StopBits     = UART_STOPBITS_1;
  huart3.Init.Parity       = UART_PARITY_NONE;
  huart3.Init.Mode         = UART_MODE_TX_RX;  
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  HAL_UART_Init(&huart3);

  GPIO_InitTypeDef GPIO_InitStruct;
    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
    GPIO_InitStruct.Pin      = GPIO_PIN_10;
    GPIO_InitStruct.Pull     = GPIO_PULLUP;
    GPIO_InitStruct.Mode     = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed    = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
    GPIO_InitStruct.Pin      = GPIO_PIN_11;
    GPIO_InitStruct.Pull     = GPIO_NOPULL;
    GPIO_InitStruct.Mode     = GPIO_MODE_INPUT;
    GPIO_InitStruct.Speed    = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
    // start interrupt receive?
    HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
}


void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  ////////////////////////////////////////////////////
  // input pins without pull-up, fast speed
  // explicitly set what we want, so that a change above will not
  // cause a problem
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;

  GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING_FALLING;

  GPIO_InitStruct.Pin = LEFT_HALL_U_PIN;
  HAL_GPIO_Init(LEFT_HALL_U_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_HALL_V_PIN;
  HAL_GPIO_Init(LEFT_HALL_V_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_HALL_W_PIN;
  HAL_GPIO_Init(LEFT_HALL_W_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_HALL_U_PIN;
  HAL_GPIO_Init(RIGHT_HALL_U_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_HALL_V_PIN;
  HAL_GPIO_Init(RIGHT_HALL_V_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_HALL_W_PIN;
  HAL_GPIO_Init(RIGHT_HALL_W_PORT, &GPIO_InitStruct);
  ////////////////////////////////////////////////////


  ////////////////////////////////////////////////////
  // input pins with pull-up
  // explicitly set what we want, so that a change above will not
  // cause a problem
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;

  GPIO_InitStruct.Pin = CHARGER_PIN;
  HAL_GPIO_Init(CHARGER_PORT, &GPIO_InitStruct);
  ////////////////////////////////////////////////////

  ////////////////////////////////////////////////////
  // input pins without pull-up
  // explicitly set what we want, so that a change above will not
  // cause a problem
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;

  GPIO_InitStruct.Pin = BUTTON_PIN;
  HAL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);
  ////////////////////////////////////////////////////


  ////////////////////////////////////////////////////
  // slow output pins
  // explicitly set what we want, so that a change above will not
  // cause a problem
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;


  GPIO_InitStruct.Pin = LED_PIN;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BUZZER_PIN;
  HAL_GPIO_Init(BUZZER_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = OFF_PIN;
  HAL_GPIO_Init(OFF_PORT, &GPIO_InitStruct);
  ////////////////////////////////////////////////////


  ////////////////////////////////////////////////////
  // Analog input pins
  // explicitly set what we want, so that a change above will not
  // cause a problem
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = LEFT_DC_CUR_PIN;
  HAL_GPIO_Init(LEFT_DC_CUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_U_CUR_PIN;
  HAL_GPIO_Init(LEFT_U_CUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_V_CUR_PIN;
  HAL_GPIO_Init(LEFT_V_CUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_DC_CUR_PIN;
  HAL_GPIO_Init(RIGHT_DC_CUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_U_CUR_PIN;
  HAL_GPIO_Init(RIGHT_U_CUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_V_CUR_PIN;
  HAL_GPIO_Init(RIGHT_V_CUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DCLINK_PIN;
  HAL_GPIO_Init(DCLINK_PORT, &GPIO_InitStruct);

  //Analog in
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /////////////////////////////////////////////////////



  ////////////////////////////////////////////////////
  // PWM output pins
  // explicitly set what we want, so that a change above will not
  // cause a problem
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = LEFT_TIM_UH_PIN;
  HAL_GPIO_Init(LEFT_TIM_UH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_TIM_VH_PIN;
  HAL_GPIO_Init(LEFT_TIM_VH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_TIM_WH_PIN;
  HAL_GPIO_Init(LEFT_TIM_WH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_TIM_UL_PIN;
  HAL_GPIO_Init(LEFT_TIM_UL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_TIM_VL_PIN;
  HAL_GPIO_Init(LEFT_TIM_VL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_TIM_WL_PIN;
  HAL_GPIO_Init(LEFT_TIM_WL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_TIM_UH_PIN;
  HAL_GPIO_Init(RIGHT_TIM_UH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_TIM_VH_PIN;
  HAL_GPIO_Init(RIGHT_TIM_VH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_TIM_WH_PIN;
  HAL_GPIO_Init(RIGHT_TIM_WH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_TIM_UL_PIN;
  HAL_GPIO_Init(RIGHT_TIM_UL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_TIM_VL_PIN;
  HAL_GPIO_Init(RIGHT_TIM_VL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_TIM_WL_PIN;
  HAL_GPIO_Init(RIGHT_TIM_WL_PORT, &GPIO_InitStruct);
  ///////////////////////////////////////////////////
}

void MX_TIM_Init(void) {
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM8_CLK_ENABLE();

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_SlaveConfigTypeDef sTimConfig;

  htim_right.Instance               = RIGHT_TIM;
  htim_right.Init.Prescaler         = 0;
  htim_right.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
  htim_right.Init.Period            = 64000000 / 2 / PWM_FREQ;
  htim_right.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim_right.Init.RepetitionCounter = 0;
  htim_right.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_PWM_Init(&htim_right);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim_right, &sMasterConfig);

  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
  HAL_TIM_PWM_ConfigChannel(&htim_right, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim_right, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim_right, &sConfigOC, TIM_CHANNEL_3);

  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime         = DEAD_TIME;
  sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim_right, &sBreakDeadTimeConfig);

  htim_left.Instance               = LEFT_TIM;
  htim_left.Init.Prescaler         = 0;
  htim_left.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
  htim_left.Init.Period            = 64000000 / 2 / PWM_FREQ;
  htim_left.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim_left.Init.RepetitionCounter = 0;
  htim_left.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_PWM_Init(&htim_left);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_ENABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim_left, &sMasterConfig);

  sTimConfig.InputTrigger = TIM_TS_ITR0;
  sTimConfig.SlaveMode    = TIM_SLAVEMODE_GATED;
  HAL_TIM_SlaveConfigSynchronization(&htim_left, &sTimConfig);

  // Start counting >0 to effectively offset timers by the time it takes for one ADC conversion to complete.
  // This method allows that the Phase currents ADC measurements are properly aligned with LOW-FET ON region for both motors
  LEFT_TIM->CNT 		     = ADC_TOTAL_CONV_TIME;

  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
  HAL_TIM_PWM_ConfigChannel(&htim_left, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim_left, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim_left, &sConfigOC, TIM_CHANNEL_3);

  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime         = DEAD_TIME;
  sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim_left, &sBreakDeadTimeConfig);

  LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
  RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;

  HAL_TIM_PWM_Start(&htim_left, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim_left, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim_left, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim_left, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim_left, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim_left, TIM_CHANNEL_3);

  HAL_TIM_PWM_Start(&htim_right, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim_right, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim_right, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim_right, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim_right, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim_right, TIM_CHANNEL_3);

  htim_left.Instance->RCR = 1;

  __HAL_TIM_ENABLE(&htim_right);
}

void MX_ADC1_Init(void) {
  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

  __HAL_RCC_ADC1_CLK_ENABLE();

  hadc1.Instance                   = ADC1;
  hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode    = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion       = 5;
  HAL_ADC_Init(&hadc1);
  /**Enable or disable the remapping of ADC1_ETRGREG:
    * ADC1 External Event regular conversion is connected to TIM8 TRG0
    */
  __HAL_AFIO_REMAP_ADC1_ETRGREG_ENABLE();

  /**Configure the ADC multi-mode
    */
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);

  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.Channel = ADC_CHANNEL_11;  // pc1 left cur  ->  right
  sConfig.Rank    = 1;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  // sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  sConfig.Channel = ADC_CHANNEL_0;  // pa0 right a   ->  left
  sConfig.Rank    = 2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_14;  // pc4 left b   -> right
  sConfig.Rank    = 3;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_1;  // pc2 vbat
  sConfig.Rank    = 4;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  //temperature requires at least 17.1uS sampling time
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;  // internal temp
  sConfig.Rank    = 5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  hadc1.Instance->CR2 |= ADC_CR2_DMA | ADC_CR2_TSVREFE;

  __HAL_ADC_ENABLE(&hadc1);

  __HAL_RCC_DMA1_CLK_ENABLE();

  DMA1_Channel1->CCR   = 0;
  DMA1_Channel1->CNDTR = 5;
  DMA1_Channel1->CPAR  = (uint32_t) & (ADC1->DR);
  DMA1_Channel1->CMAR  = (uint32_t)&adc_buffer;
  DMA1_Channel1->CCR   = DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TCIE;
  DMA1_Channel1->CCR |= DMA_CCR_EN;

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/* ADC2 init function */
void MX_ADC2_Init(void) {
  ADC_ChannelConfTypeDef sConfig;

  __HAL_RCC_ADC2_CLK_ENABLE();

  // HAL_ADC_DeInit(&hadc2);
  // hadc2.Instance->CR2 = 0;
  /**Common config
    */
  hadc2.Instance                   = ADC2;
  hadc2.Init.ScanConvMode          = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode    = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion       = 5;
  HAL_ADC_Init(&hadc2);

 
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.Channel = ADC_CHANNEL_10;  // pc0 right cur   -> left
  sConfig.Rank    = 1;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  // sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  sConfig.Channel = ADC_CHANNEL_13;  // pc3 right b   -> left
  sConfig.Rank    = 2;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  sConfig.Channel = ADC_CHANNEL_15;  // pc5 left c   -> right
  sConfig.Rank    = 3;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  sConfig.Channel = ADC_CHANNEL_2;  // pa2 uart-l-tx
  sConfig.Rank    = 4;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  sConfig.Channel = ADC_CHANNEL_3;  // pa3 uart-l-rx
  sConfig.Rank    = 5;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  hadc2.Instance->CR2 |= ADC_CR2_DMA;
  __HAL_ADC_ENABLE(&hadc2);
}

void DMA1_Channel7_IRQHandler() // first hit after send
{
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
}

void DMA1_Stream7_IRQHandler() 
{
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
}

extern unsigned int buffer_position;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  
}
