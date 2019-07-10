/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether 
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "main.h"

#include "stm32f1xx_hal.h"

#include <stdint.h>

#include <stdbool.h>

#include <stdlib.h>

#include <string.h>

#include "stm32f1xx_hal_def.h"

#define OS_CONFIG_MAX_TASKS 10

// function prototypes
bool os_init(void);
void _save_state(void);
void _load_state(void);
void _load_initial_state(void);
bool os_scheduler(void);
void SysTick_Handler(void);
bool os_start(uint32_t systick_ticks);
int os_task_init(void( * handler)(void * p_params), void * p_task_params,
  uint8_t p_task_priority, uint32_t * p_stack, size_t stack_size);


void delay(unsigned int nCount);
void blinky(void * d);
void ledInit(void);

GPIO_InitTypeDef GPIO_InitStruct, GPIO_InitStruct2;

volatile struct os_task * os_curr_task;
volatile struct os_task * os_next_task;
volatile uint32_t mpsp, handler, params, xPSR, thread_return, os_priority;

char stack1[128];
char stack2[128];

//
// types
enum os_task_status {
  OS_TASK_STATUS_IDLE = 1,    /*Task has just been added to the queue*/
  OS_TASK_STATUS_ACTIVE,      /*The task is executing*/
  OS_TASK_STATUS_SUSPENDED,   /*The thread was suspended by thread synchronization
																module*/
};

struct os_task {
  volatile uint32_t sp;
  void( * handler)(void * p_params);
  void * p_params;
	uint32_t xPSR;
  volatile enum os_task_status status;
	volatile uint8_t priority;
};

static enum {
  OS_STATE_DEFAULT = 1,
    OS_STATE_INITIALIZED,
    OS_STATE_TASKS_INITIALIZED,
    OS_STATE_STARTED,
}
m_state = OS_STATE_DEFAULT;

static struct {
  struct os_task tasks[OS_CONFIG_MAX_TASKS];
  volatile uint8_t current_task;
  uint16_t size;
}
m_task_table;

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */


static void task_finished(void) {
  /* This function is called when some task handler returns. */

  volatile uint8_t i = 0;
  while (1) {
    i++;
  }
}

bool os_init(void) {
  if (m_state != OS_STATE_DEFAULT)
    return false;

  memset( &m_task_table, 0, sizeof(m_task_table));
  m_state = OS_STATE_INITIALIZED;

  m_task_table.size = 0;

  return true;
}


void os_task_suspend(uint8_t task_id) {
	m_task_table.tasks[task_id].status = OS_TASK_STATUS_SUSPENDED;
}

void os_task_resume(uint8_t task_id) {
	m_task_table.tasks[task_id].status = OS_TASK_STATUS_ACTIVE;
}
	

int os_task_init(void( * handler)(void * p_params), void * p_task_params,
  uint8_t p_task_priority, uint32_t * p_stack, size_t stack_size) {

  if (m_task_table.size >= OS_CONFIG_MAX_TASKS - 1)
    return false;
  struct os_task * p_task = &m_task_table.tasks[m_task_table.size];
  p_task->handler = handler;
	p_task->p_params = p_task_params;
  p_task->priority = p_task_priority;
	/* Initialize the task structure and set SP to the top of the stack
     minus 16 words (64 bytes) to leave space for storing 16 registers: */
  p_task->sp = (uint32_t) &p_stack[stack_size - 16];
  p_task->status = OS_TASK_STATUS_IDLE;

  //Save init. values of registers which will be restored on exc. return:   
  p_stack[stack_size - 1] = 0x01000000; // - XPSR: Default value (0x01000000)
  p_stack[stack_size - 2] = (uint32_t) handler; // - PC: Point to the handler function
  p_stack[stack_size - 3] = (uint32_t) task_finished; //- LR: Point to thread return handler
  p_stack[stack_size - 8] = (uint32_t) p_task_params; //- R0: Point to the handler function's parameter

  m_state = OS_STATE_TASKS_INITIALIZED;
 
  return  m_task_table.size++;
}
	
bool os_start(uint32_t systick_ticks) {
  if (m_state != OS_STATE_TASKS_INITIALIZED)
    return false;

  NVIC_SetPriority(SysTick_IRQn, 0x00); /* Highest possible priority */

  /* Start the SysTick timer: */
  uint32_t ret_val = SysTick_Config(systick_ticks);
  if (ret_val != 0)
	{
    return false;
	}
  /* Start the first task: */
  os_curr_task = &m_task_table.tasks[m_task_table.current_task];
  m_state = OS_STATE_STARTED;

  __set_PSP(os_curr_task -> sp + 64); /* Set PSP to the top of task's stack */
  __set_CONTROL(0x03); /* Switch to Unprivilleged Thread Mode with PSP */
  __ISB(); /* Execute ISB after changing CONTORL (recommended) */
	os_priority = os_curr_task->priority; 
	_load_initial_state();
}


uint32_t get_next_task() {
	uint32_t next_task_id = m_task_table.current_task;
	do {
		next_task_id++;
		next_task_id %= m_task_table.size;
	} while(m_task_table.tasks[next_task_id].status == OS_TASK_STATUS_SUSPENDED);
	return next_task_id;
}

bool os_scheduler(){
	bool is_preempted;
	is_preempted = false;
	uint32_t next_task = get_next_task();
	os_curr_task = &m_task_table.tasks[m_task_table.current_task];
	os_next_task = &m_task_table.tasks[next_task];
				
  // os_curr_task -> status = OS_TASK_STATUS_IDLE;

		if (os_curr_task->status == OS_TASK_STATUS_SUSPENDED) {
			is_preempted = true;
			os_next_task->status = OS_TASK_STATUS_ACTIVE;
			os_priority = os_next_task->priority;
			m_task_table.current_task = next_task;
		}
		else {
			os_priority--;
			if(os_priority <= 0){
				is_preempted = true;
				os_next_task -> status = OS_TASK_STATUS_ACTIVE;
				os_priority = os_next_task->priority;
				m_task_table.current_task = next_task;
			}
			else{
				is_preempted = false;
			}
		}
	
  /* Select next task: */
  /*
	m_task_table.current_task++;
  if (m_task_table.current_task >= m_task_table.size)
    m_task_table.current_task = 0;

	*/
  
	return is_preempted;
}
void SysTick_Handler(void) {
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
	
	
  /*
		register unsigned int cp15_control __asm("cp15:0:c1:c0:0");
		cp15_control |= 0x1; 
	*/

  /*
  __asm  { 
		  MRS r0, __get_PSP
	};
	*/
	if(os_scheduler())
	{
_save_state();
_load_state();
	}
}

void blinky(void * d) {
  int temp = * ((int * ) d);
  volatile int i = 0;
	while(1){
	i=0;
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
    
		while (i < temp) {
    /* Toggle LED on PA0 */
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);	
		// Reset bit will turn on LED (because the logic is interved)
		//delay(temp);
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
    //delay(temp);
		
		i++;
  }
}
	return;
}

void blinky2(void * d) {
  int temp = * ((int * ) d);
  volatile int i = 0;
while(1){
	i = 0;
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
  while (i < temp) {
    /* Toggle LED on PA0 */
    // Reset bit will turn on LED (because the logic is interved)
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
    //delay(temp);
    // Set bit will turn off LED (because the logic is interved)
    //HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
    //delay(temp);
    i++;
  }
}
	return;
}


// Delay function
void delay(unsigned int nCount) {
  volatile unsigned int i, j;

  for (i = 0; i < nCount; i++)
    for (j = 0; j < 0x2AFF; j++);
}

int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t delay = 100000;
	uint32_t delay2 = 1000000;
  //while(1)
  //{
 // blinky((void*)&delay);
  //blinky((void*)&delay2);
  //}
  os_init();
  memset(&stack1, 0xFF, 128);
	memset(&stack2, 0xFF, 128);
  uint8_t task_1 = os_task_init(blinky2, (void * ) &delay, 3,(uint32_t*) &stack1, 128/4);
  uint8_t task_2 = os_task_init(blinky, (void * ) &delay, 30,(uint32_t*) &stack2, 128/4);
	os_start(300);
  //  while (1)
  //  {

  //  /* USER CODE END WHILE */

  //  /* USER CODE BEGIN 3 */

  //  }
  //  /* USER CODE END 3 */

  //}
 }
/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /**Initializes the CPU, AHB and APB busses clocks 
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig( &RCC_OscInitStruct) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Initializes the CPU, AHB and APB busses clocks 
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Configure the Systick interrupt time 
   */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick 
   */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as 
 * Analog 
 * Input 
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {
  /* GPIO Ports Clock Enable */
  //__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
  /*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	
  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;3
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	
		__HAL_RCC_GPIOC_CLK_ENABLE();
  /*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	
  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct2.Pin = GPIO_PIN_13;
  GPIO_InitStruct2.Mode = GPIO_MODE_OUTPUT_PP;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;3
  GPIO_InitStruct2.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct2);
	
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char * file, int line) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while (1) {}
  /* USER CODE END Error_Handler_Debug */
}

# ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t * file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
