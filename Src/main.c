/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "i2c.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
//------------------------------------------------------------------------------
/* Projeto    : NUCLEO-F072RB_eeprom_i2c
 * MCU        : STM32F072RB (NUCLEO-F072RB)
 * Copyright  : Atribuição (CC BY 4.0) Diego Moreno
 * Versão     : 1.0.0
 * Data	      : 17/02/2017
 * Descrição  : Grava uma página de 16 bytes na memória EEPROM M24C08-W via I2C.
 *		NUCLEO-F072RB Schematics - http://migre.me/tVEOR
 */

/* Includes */


/* Defines */
#define EEPROM_I2C_ADDRESS      (uint16_t)0xAE
#define EEPROM_ADDRESS          (uint16_t)0x0000
#define PAGE_SIZE               (uint16_t)2//bytes

//------------------------------------------------------------------------------
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t master_write[PAGE_SIZE];
uint8_t master_read[PAGE_SIZE];
uint8_t flag_TX_complete = 1;
uint8_t flag_RX_complete = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void i2c_interrupt(void);
void i2c_polling(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  
  uint32_t i = 0;
  
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    
    //Enchendo Buffer
    for(i = 0;i < PAGE_SIZE;i++)
    {
      master_write[i] = i + 1;   //1 até PAGE_SIZE
//      master_write[i] = PAGE_SIZE - i;  //PAGE_SIZE até 1
    }
    
    master_write[0] = 0xAB;
    master_write[1] = 0xCD;
    
    //Escrita e Leitura I2C por Interrupção
    i2c_interrupt();
    
    //Escrita e Leitura I2C por Polling
//    i2c_polling();
    
    
    while(1);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
   flag_TX_complete = 0;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  flag_RX_complete = 0;
}

void i2c_interrupt(void)
{
  //Esperando BUS estar pronto
  while(HAL_I2C_IsDeviceReady(&hi2c1, EEPROM_I2C_ADDRESS, 10, 1000) == HAL_TIMEOUT);
  while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
  
  //Escrevendo Byte
  HAL_I2C_Mem_Write_IT(&hi2c1, EEPROM_I2C_ADDRESS, EEPROM_ADDRESS, I2C_MEMADD_SIZE_16BIT, master_write, PAGE_SIZE);
  while(flag_TX_complete);
  flag_TX_complete = 1;
  
  //Esperando BUS estar pronto
  while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
  while(HAL_I2C_IsDeviceReady(&hi2c1, EEPROM_I2C_ADDRESS, 10, 1000) == HAL_TIMEOUT);
  while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
  
  //Lendo Byte
  HAL_I2C_Mem_Read_IT(&hi2c1, EEPROM_I2C_ADDRESS, EEPROM_ADDRESS, I2C_MEMADD_SIZE_16BIT, master_read, PAGE_SIZE);
  while(flag_RX_complete);
  flag_RX_complete = 1;
}

void i2c_polling(void)
{
  //Esperando BUS estar pronto
  while(HAL_I2C_IsDeviceReady(&hi2c1, EEPROM_I2C_ADDRESS, 10, 1000) == HAL_TIMEOUT);
  while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
  
  //Escrevendo Byte
  while(HAL_I2C_Mem_Write(&hi2c1, EEPROM_I2C_ADDRESS, EEPROM_ADDRESS, I2C_MEMADD_SIZE_16BIT, master_write, PAGE_SIZE, 1000) != HAL_OK);
  
  //Esperando BUS estar pronto
  while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
  while(HAL_I2C_IsDeviceReady(&hi2c1, EEPROM_I2C_ADDRESS, 10, 1000) == HAL_TIMEOUT);
  while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
  
  //Lendo Byte
  while(HAL_I2C_Mem_Read(&hi2c1, EEPROM_I2C_ADDRESS, EEPROM_ADDRESS, I2C_MEMADD_SIZE_16BIT, master_read, PAGE_SIZE, 1000) != HAL_OK);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
