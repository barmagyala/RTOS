
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body
//  ******************************************************************************
//  * This notice applies to any and all portions of this file
//  * that are not between comment pairs USER CODE BEGIN and
//  * USER CODE END. Other portions of this file, whether
//  * inserted by the user or by software development tools
//  * are owned by their respective copyright owners.
//  *
//  * Copyright (c) 2020 STMicroelectronics International N.V.
//  * All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted, provided that the following conditions are met:
//  *
//  * 1. Redistribution of source code must retain the above copyright notice,
//  *    this list of conditions and the following disclaimer.
//  * 2. Redistributions in binary form must reproduce the above copyright notice,
//  *    this list of conditions and the following disclaimer in the documentation
//  *    and/or other materials provided with the distribution.
//  * 3. Neither the name of STMicroelectronics nor the names of other
//  *    contributors to this software may be used to endorse or promote products
//  *    derived from this software without specific written permission.
//  * 4. This software, including modifications and/or derivative works of this
//  *    software, must execute solely and exclusively on microcontroller or
//  *    microprocessor devices manufactured by or for STMicroelectronics.
//  * 5. Redistribution and use of this software other than as permitted under
//  *    this license is void and will automatically terminate your rights under
//  *    this license.
//  *
//  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
//  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
//  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
//  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
//  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
//  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
//  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
//  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
//  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  *
//  ******************************************************************************
//  */
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//#include "stm32f1xx_hal.h"
//#include "cmsis_os.h"
//#include <stdbool.h>

///* USER CODE BEGIN Includes */

///* USER CODE END Includes */

///* Private variables ---------------------------------------------------------*/
//UART_HandleTypeDef huart1;

//osThreadId defaultTaskHandle;

///* USER CODE BEGIN PV */
///* Private variables ---------------------------------------------------------*/

///* USER CODE END PV */

///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_USART1_UART_Init(void);
//void StartDefaultTask(void const * argument);

///* USER CODE BEGIN PFP */
///* Private function prototypes -----------------------------------------------*/

///* USER CODE END PFP */

///* USER CODE BEGIN 0 */

///* USER CODE END 0 */

///**
//  * @brief  The application entry point.
//  *
//  * @retval None
//  */
//  struct table{
//    int priority;//15 for iddle, the lower the number the higher  the priority, 1 highest prob
//    int ET;//expected execution time
//  //	int time;//number of times  it  was  actually executed->has to reach ET, if preempted  and time != ET so we  shouldexecuted  till =ET
//  };
//  
//  struct table arr[3];
//  struct table iddle;
//  
//  bool first=true;//if equal true than the task no taskwere executed  before
//  int currentTask;
//  
//  
//  QueueHandle_t Q = 0;
//  
//  void init(){
//    arr[0].priority = 2;//has higher priority than arr[1]
//    arr[0].ET = 4;//constant should not be modified anywhere
//  //	arr[0].time = 0;//time always starts with 0
//    
//    arr[1].priority = 4;
//    arr[1].ET = 2;//constant should not be modifies anywhere
//    //arr[1].time = 0;//time always starts with 0
//    
//    iddle.priority = 16;//the lowest priority--> cannot be changed 
//    iddle.ET = 1;//can be repeated
//    //iddle.time = 0;
//  }
//  void Preempt(void *task){
//		
//		
//	}
//  /*void Preempt(char New){
//    
//    int tasknum;
//    if(New=='1') tasknum = 1;
//    else if(New=='2')	tasknum = 2;
//    else tasknum = 0;
//    
//    //if New has lower priority than the task being executed
//    if(tasknum>currentTask || tasknum == 0){
//      for(int i=0;i<arr[tasknum].ET;i++)
//        xQueueSendToBack(Q,&New,10);
//      
//    }
//    else if(tasknum>= currentTask && tasknum != 0){
//      // if the priority is higher or the same --> we preempt here
//      for(int i=0;i<arr[tasknum].ET;i++)
//        xQueueSendToFront(Q,&New,10);
//    }
//    
//  }*/
//  
//  void task0(void *p){
//    //this is the iddle task
//    char c[29] = "Iddle task is being executed";
//    taskENTER_CRITICAL();
//		for (int i=0; i<29; i++){
//    HAL_UART_Transmit(&huart1,(uint8_t*)&c,i,10);
//		}
//    taskEXIT_CRITICAL();
//    currentTask = 0;
//    
//  }
//  
//  void task1(void* p){
//    
//    char c[23] = "task1 is being executed";
//    taskENTER_CRITICAL();
//		for (int i=0; i<23; i++){
//    HAL_UART_Transmit(&huart1,(uint8_t*)&c,1,10);
//		}
//    taskEXIT_CRITICAL();
//    currentTask = 1;
//    
//  }
//  
//  void task2(void* p){
//    
//    char c[23] = "task2 is being executed";
//    taskENTER_CRITICAL();
//		for (int i=0; i<23; i++){
//    HAL_UART_Transmit(&huart1,(uint8_t*)&c,1,10);
//		}
//    taskEXIT_CRITICAL();
//    currentTask = 2;
//    
//  }
//int main(void)
//{
//  /* USER CODE BEGIN 1 */

//  /* USER CODE END 1 */

//  /* MCU Configuration----------------------------------------------------------*/

//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();

//  /* USER CODE BEGIN Init */

//  /* USER CODE END Init */

//  /* Configure the system clock */
//  SystemClock_Config();

//  /* USER CODE BEGIN SysInit */

//  /* USER CODE END SysInit */

//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_USART1_UART_Init();
//  /* USER CODE BEGIN 2 */

//  /* USER CODE END 2 */

//  /* USER CODE BEGIN RTOS_MUTEX */
//  /* add mutexes, ... */
//  /* USER CODE END RTOS_MUTEX */

//  /* USER CODE BEGIN RTOS_SEMAPHORES */
//  /* add semaphores, ... */
//  /* USER CODE END RTOS_SEMAPHORES */

//  /* USER CODE BEGIN RTOS_TIMERS */
//  /* start timers, add new ones, ... */
//  /* USER CODE END RTOS_TIMERS */

//  /* Create the thread(s) */
//  /* definition and creation of defaultTask */
//  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
//  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

//  /* USER CODE BEGIN RTOS_THREADS */
//  /* add threads, ... */
//  /* USER CODE END RTOS_THREADS */

//  /* USER CODE BEGIN RTOS_QUEUES */
//  /* add queues, ... */
//  /* USER CODE END RTOS_QUEUES */
//  __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);
//	
//		//create the queue
//	Q = xQueueCreate(20, sizeof(int));//size of queue is 20

//	vQueueAddToRegistry( Q, "Queue" );

//	//create  the  tasks
//	xTaskCreate(task1, (const char*)"t1", 4096, NULL, 1, NULL);
//	xTaskCreate(task2, (const char*)"t2", 4096, NULL, 1, NULL);
//	xTaskCreate(task0, (const char*)"iddle", 4096, NULL, 1, NULL);
//	


//	init();//assign each task its initial values
//	
//	
//	//scheduling
//	/*
//	user inputs a number from 0 to 2
//	if the priority is higher, we save it in the front of the queue
//	if the priority is lower we save it in the back of t queue
//	*/
//	void *p;
//	//char USER_IN;//one digit at a time
//	//HAL_UART_Receive(&huart1,(uint8_t *)&USER_IN,1,500);//timeout duration = 500
//	//recceive userinput
//	if(first)
//	{
//		//firts task to be executed
//		if(USER_IN == '1')//user called task1
//			task1(p);
//		else
//			task2(p);
//		first = false;
//	}
//	else{
//		//not the first task being executed
//		Preempt(USER_IN);
//	}
//	
//	
//	//read  from queue and call the tasks
//	int c;
//	for(;;)// I am not sure about the inifite loop here
//	{
//		if(xQueueReceive(Q,(uint8_t*)&c,500)==pdPASS){//reads from Q and executes the tasks
//			if(c==1) task1(p);
//			else if(c==2) task2(p);
//			else task0(p);
//		}
//		
//	}

//  /* Start scheduler */
//  osKernelStart();

//  /* We should never get here as control is now taken by the scheduler */

//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//  while (1)
//  {

//  /* USER CODE END WHILE */

//  /* USER CODE BEGIN 3 */

//  }
//  /* USER CODE END 3 */

//}

///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//void SystemClock_Config(void)
//{

//  RCC_OscInitTypeDef RCC_OscInitStruct;
//  RCC_ClkInitTypeDef RCC_ClkInitStruct;

//    /**Initializes the CPU, AHB and APB busses clocks
//    */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.HSICalibrationValue = 16;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

//    /**Initializes the CPU, AHB and APB busses clocks
//    */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

//    /**Configure the Systick interrupt time
//    */
//  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

//    /**Configure the Systick
//    */
//  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

//  /* SysTick_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
//}

///* USART1 init function */
//static void MX_USART1_UART_Init(void)
//{

//  huart1.Instance = USART1;
//  huart1.Init.BaudRate = 115200;
//  huart1.Init.WordLength = UART_WORDLENGTH_8B;
//  huart1.Init.StopBits = UART_STOPBITS_1;
//  huart1.Init.Parity = UART_PARITY_NONE;
//  huart1.Init.Mode = UART_MODE_TX_RX;
//  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
//  if (HAL_UART_Init(&huart1) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

//}

///** Configure pins as
//        * Analog
//        * Input
//        * Output
//        * EVENT_OUT
//        * EXTI
//*/
//static void MX_GPIO_Init(void)
//{

//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOA_CLK_ENABLE();

//}

///* USER CODE BEGIN 4 */

///* USER CODE END 4 */

///* USER CODE BEGIN Header_StartDefaultTask */
///**
//  * @brief  Function implementing the defaultTask thread.
//  * @param  argument: Not used
//  * @retval None
//  */
///* USER CODE END Header_StartDefaultTask */
//void StartDefaultTask(void const * argument)
//{

//  /* USER CODE BEGIN 5 */
//  /* Infinite loop */
//  for(;;)
//  {
//    osDelay(1);
//  }
//  /* USER CODE END 5 */
//}

///**
//  * @brief  Period elapsed callback in non blocking mode
//  * @note   This function is called  when TIM1 interrupt took place, inside
//  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
//  * a global variable "uwTick" used as application time base.
//  * @param  htim : TIM handle
//  * @retval None
//  */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//  /* USER CODE BEGIN Callback 0 */

//  /* USER CODE END Callback 0 */
//  if (htim->Instance == TIM1) {
//    HAL_IncTick();
//  }
//  /* USER CODE BEGIN Callback 1 */

//  /* USER CODE END Callback 1 */
//}

///**
//  * @brief  This function is executed in case of error occurrence.
//  * @param  file: The file name as string.
//  * @param  line: The line in file as a number.
//  * @retval None
//  */
//void _Error_Handler(char *file, int line)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//  while(1)
//  {
//  }
//  /* USER CODE END Error_Handler_Debug */
//}

//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t* file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */

///**
//  * @}
//  */

///**
//  * @}
//  */

///************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2020 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"


/* USER CODE BEGIN Includes */
#include <stdbool.h>

#include <stdio.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define READYQ_SIZE 200
int readyQ_rear = - 1;
int readyQ_front = - 1;

struct TASK_TYPE{
  void *task;
  int priority; // 0 to 15
  int exectime;
  int stacksize;
};

//void (*ready_queue[READYQ_SIZE])();
struct TASK_TYPE ready_queue[READYQ_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Quetask(struct TASK_TYPE);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
  int isFull(int front, int rear)
  {
    if( (front == rear + 1) || (front == 0 && rear == READYQ_SIZE-1)) return 1;
    return 0;
  }

  int isEmpty(int front)
  {
    if(front == -1) return 1;
    return 0;
  }

void createTask(void *task, int priority, int exec_time, int stack_size)
{
  struct TASK_TYPE newtask;
    newtask.task = task;
    newtask.priority = priority;
    newtask.exectime = exec_time;
    newtask.stacksize = stack_size;
    Quetask(newtask);
}

  /*void init(){
    arr[0].priority = 2;//has higher priority than arr[1]
    arr[0].ET = 4;//constant should not be modified anywhere
  //	arr[0].time = 0;//time always starts with 0

    arr[1].priority = 4;
    arr[1].ET = 2;//constant should not be modifies anywhere
    //arr[1].time = 0;//time always starts with 0

    iddle.priority = 16;//the lowest priority--> cannot be changed
    iddle.ET = 1;//can be repeated
    //iddle.time = 0;
  }*/

  void Quetask(struct TASK_TYPE taskname){
    if (isEmpty(readyQ_front)){
      readyQ_front = 0;
      readyQ_rear = (readyQ_rear + 1) % READYQ_SIZE;
      ready_queue[readyQ_rear] = taskname;

      printf("enqueued in ready queue \n");
    }
    else if(isFull(readyQ_front, readyQ_rear)){
      printf("Ready Queue is full !! \n");
    }
    else{
      if ((readyQ_front == 0) && (readyQ_rear == 0)){
        if (taskname.priority<ready_queue[readyQ_rear].priority){
          readyQ_rear = (readyQ_rear + 1) % READYQ_SIZE;
          ready_queue[readyQ_rear] = taskname;
        }
        else{
          ready_queue[(readyQ_rear + 1) % READYQ_SIZE] = ready_queue[readyQ_rear];
          ready_queue[readyQ_rear] = taskname;
          readyQ_rear = (readyQ_rear + 1) % READYQ_SIZE;
        }
      }
      else{
        int i = readyQ_rear;
        while (i != readyQ_front){
          if (taskname.priority<ready_queue[i].priority){
            readyQ_rear = (readyQ_rear + 1) % READYQ_SIZE;
            ready_queue[(i+1) % READYQ_SIZE] = taskname;
            break;
          }
          else{
            ready_queue[(i+1) % READYQ_SIZE] = ready_queue[i];
            readyQ_rear = (readyQ_rear + 1) % READYQ_SIZE;
          }
          i = (i-1) % READYQ_SIZE;
        }
      }
    }
	}
	
	void idletask(){
	  char c[4] = "idle";
		
    HAL_UART_Transmit(&huart1,(uint8_t*)&c,4,10);
		
	}

  void task1(){
    char c[23] = "task1 is being executed";
		
    HAL_UART_Transmit(&huart1,(uint8_t*)&c,23,10);
		
  }

  void task2(){
    char c[23] = "task2 is being executed";
		
    HAL_UART_Transmit(&huart1,(uint8_t*)&c,23,10);
		
  }

  void task3(){
    char c[23] = "task3 is being executed";
   
    HAL_UART_Transmit(&huart1,(uint8_t*)&c,23,10);
   
  }

  void Dispatch(){
		void * executetask;
    if(isEmpty(readyQ_front)) {
      printf("Ready Queue is empty !! \n");
      return;
    }
    else {
			executetask= ready_queue[readyQ_front].task;
      void * executetask();
      if (readyQ_front == readyQ_rear){
        readyQ_front = -1;
        readyQ_rear = -1;
      }
      else {
        readyQ_front = (readyQ_front + 1) % READYQ_SIZE;
      }
    }
  }

int main(void)
{
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  int count = 1;

  /* USER CODE END WHILE */
  if (count == 1)
  {
    createTask(task1, 4, 3, 200);
    createTask(task1, 4, 3, 200);
    createTask(task1, 4, 3, 200);
  }
  else if (count == 2)
  {
    createTask(task2, 3, 3, 200);
    createTask(task2, 3, 3, 200);
    createTask(task2, 3, 3, 200);
  }
  else if (count == 3)
    createTask(task3, 2, 3, 200);

count++;
	if (isEmpty(readyQ_front))
		createTask(idletask, 15, 1, 0);
  
	Dispatch();
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{
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
