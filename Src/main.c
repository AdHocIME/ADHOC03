
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
  * Copyright (c) 2018 STMicroelectronics International N.V. 
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
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define mainHOST_NAME					"RTOSDemo"
#define mainDEVICE_NICK_NAME			"stm32"
#define NUM_LEDS	4

static TaskHandle_t xServerWorkTaskHandle = NULL;
static void prvServerWorkTask( void *pvParameters );
static void prvServerConnectionInstance( void *pvParameters );

#define FREERTOS_NO_SOCKET		NULL

#define ipconfigHTTP_TX_BUFSIZE				( 3 * ipconfigTCP_MSS )
#define ipconfigHTTP_TX_WINSIZE				( 2 )
#define ipconfigHTTP_RX_BUFSIZE				( 4 * ipconfigTCP_MSS )
#define ipconfigHTTP_RX_WINSIZE				( 4 )

/* FTP and HTTP servers execute in the TCP server work task. */
#define mainTCP_SERVER_TASK_PRIORITY	( tskIDLE_PRIORITY + 3 )
#define	mainTCP_SERVER_STACK_SIZE		( configMINIMAL_STACK_SIZE * 8 )

#ifndef configIPINIT_TASK_STACK_SIZE
	#define configIPINIT_TASK_STACK_SIZE ( 4 * configMINIMAL_STACK_SIZE )
#endif
/* The MAC address array is not declared const as the MAC address will
normally be read from an EEPROM and not hard coded (in real deployed
applications).*/
static uint8_t ucMACAddress[ 6 ] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55 };

static const uint8_t ucIPAddress[ 4 ] = { 10, 10, 10, 200 };
static const uint8_t ucNetMask[ 4 ] = { 255, 0, 0, 0 };
static const uint8_t ucGatewayAddress[ 4 ] = { 10, 10, 10, 1 };

/* The following is the address of an OpenDNS server. */
static const uint8_t ucDNSServerAddress[ 4 ] = { 208, 67, 222, 222 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void const * argument);

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
  /* USER CODE BEGIN 2 */
	FreeRTOS_IPInit( ucIPAddress,
				   ucNetMask,
				   ucGatewayAddress,
				   ucDNSServerAddress,
				   ucMACAddress );
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
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void vApplicationIPNetworkEventHook( eIPCallbackEvent_t eNetworkEvent )
{
static BaseType_t xTasksAlreadyCreated = pdFALSE;

    /* Both eNetworkUp and eNetworkDown events can be processed here. */
    if( eNetworkEvent == eNetworkUp )
    {
        /* Create the tasks that use the TCP/IP stack if they have not already
        been created. */
        if( xTasksAlreadyCreated == pdFALSE )
        {
            /*
             * For convenience, tasks that use FreeRTOS+TCP can be created here
             * to ensure they are not created before the network is usable.
             */
        	xTaskCreate( prvServerWorkTask, "SvrWork", mainTCP_SERVER_STACK_SIZE, NULL, mainTCP_SERVER_TASK_PRIORITY, &xServerWorkTaskHandle );

            xTasksAlreadyCreated = pdTRUE;
        }
    }
}
#define BUFFER_SIZE 1500

static void prvServerWorkTask( void *pvParameters )
{
	BaseType_t xNoTimeout = 0;

	/* The priority of this task can be raised now the disk has been
	initialised. */
//	vTaskPrioritySet( NULL, mainTCP_SERVER_TASK_PRIORITY );

	/* Wait until the network is up before creating the servers.  The
	notification is given from the network event hook. */
	//ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

	struct freertos_sockaddr xClient, xBindAddress;

	Socket_t xSocket, xConnectedSocket;

	xSocket = FreeRTOS_socket( FREERTOS_AF_INET, FREERTOS_SOCK_STREAM, FREERTOS_IPPROTO_TCP );

	if( xSocket != FREERTOS_INVALID_SOCKET  )
	{
		xBindAddress.sin_addr = FreeRTOS_GetIPAddress(); // Single NIC, currently not used
		xBindAddress.sin_port = FreeRTOS_htons( 80 );
		socklen_t xAddressLength = sizeof( xBindAddress );
		FreeRTOS_bind( xSocket, &xBindAddress, xAddressLength );
		FreeRTOS_listen( xSocket, 12 );

		FreeRTOS_setsockopt( xSocket, 0, FREERTOS_SO_RCVTIMEO, ( void * ) &xNoTimeout, sizeof( BaseType_t ) );
		FreeRTOS_setsockopt( xSocket, 0, FREERTOS_SO_SNDTIMEO, ( void * ) &xNoTimeout, sizeof( BaseType_t ) );


		WinProperties_t xWinProps;

		memset( &xWinProps, '\0', sizeof( xWinProps ) );
		/* The parent socket itself won't get connected.  The properties below
		will be inherited by each new child socket. */
		xWinProps.lTxBufSize = ipconfigHTTP_TX_BUFSIZE;
		xWinProps.lTxWinSize = ipconfigHTTP_TX_WINSIZE;
		xWinProps.lRxBufSize = ipconfigHTTP_RX_BUFSIZE;
		xWinProps.lRxWinSize = ipconfigHTTP_RX_WINSIZE;

		/* Set the window and buffer sizes. */
		FreeRTOS_setsockopt( xSocket, 0, FREERTOS_SO_WIN_PROPERTIES, ( void * ) &xWinProps,	sizeof( xWinProps ) );
		for( ;; )
		{
	        osDelay(250);
			/* Wait for incoming connections. */
			socklen_t xClientLength = sizeof( xClient );
			xConnectedSocket = FreeRTOS_accept( xSocket, &xClient, &xClientLength );
			//configASSERT( xConnectedSocket != FREERTOS_INVALID_SOCKET );
			if((xConnectedSocket != FREERTOS_NO_SOCKET)&&(xConnectedSocket != FREERTOS_INVALID_SOCKET)){
				static char cRxedData[ BUFFER_SIZE ];
				static char cTxedData[ BUFFER_SIZE ];
				BaseType_t lBytesReceived;
				lBytesReceived = FreeRTOS_recv( xConnectedSocket, &cRxedData, BUFFER_SIZE, 0 );
				//			static char cTxedData[ BUFFER_SIZE ];
				/* Spawn a RTOS task to handle the connection. */
				//xTaskCreate( prvServerConnectionInstance, "EchoServer", configMINIMAL_STACK_SIZE*2, ( void * ) xConnectedSocket, tskIDLE_PRIORITY,  NULL );
				char message1[] = "HTTP/1.1 200 OK\r\nConnection: close\r\n" \
						   "Content-Type: text/html; charset=UTF-8\r\n\r\n" \
						   "<html><body>Hello!<br>" \
						   "<form method='POST' action='led'>" \
						   "GREEN LED=<input type='text' name='GREEN' value='0' /><br/>" \
						   "ORANGE LED=<input type='text' name='ORANGE' value='0' /><br/>" \
						   "RED LED=<input type='text' name='RED' value='0' /><br/>" \
						   "BLUE LED=<input type='text' name='BLUE' value='0' /><br />" \
						   "<input type='submit' />" \
						   "</form>" \
						   "</body></html>";
				char *httpMethods[4] = {"GET", "POST", "PUT", "DELETE"};
				int8_t state, c;
				int i;
				c = 0;
				uint16_t LEDs[NUM_LEDS] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};
				if(memcmp(cRxedData, httpMethods[1], sizeof(httpMethods[1][0]))==0){
					for(i=0; i<4; i++) {
						char *LED_str[NUM_LEDS] = {"GREEN=", "ORANGE=", "RED=", "BLUE="};
						char *str;
						str = strstr(cRxedData, LED_str[i]);
						if(str == NULL) {
							/* Not found the corresponding LED color string. */
							continue;
						}
						c += 1;
						state = atoi(str + strlen(LED_str[i]));
						/* Set the state of the corresponding LED. */
						if(state == 1){
							HAL_GPIO_WritePin(GPIOD, LEDs[i], GPIO_PIN_SET);
						}
						else if(state == 0){
							HAL_GPIO_WritePin(GPIOD, LEDs[i], GPIO_PIN_RESET);
						}
					}
				}
	    		memcpy(cTxedData, message1, sizeof(message1));
				FreeRTOS_send( xConnectedSocket, &message1, sizeof(message1),0 );
				FreeRTOS_closesocket( xConnectedSocket );
			}
		}
	}
}

const char *pcApplicationHostnameHook( void )
{
	/* Assign the name "rtosdemo" to this network node.  This function will be
	called during the DHCP: the machine will be registered with an IP address
	plus this name. */
	return mainHOST_NAME;
}

BaseType_t xApplicationDNSQueryHook( const char *pcName )
{
BaseType_t xReturn;

	/* Determine if a name lookup is for this node.  Two names are given
	to this node: that returned by pcApplicationHostnameHook() and that set
	by mainDEVICE_NICK_NAME. */
	if( strcasecmp( pcName, pcApplicationHostnameHook() ) == 0 )
	{
		xReturn = pdPASS;
	}
	else if( strcasecmp( pcName, mainDEVICE_NICK_NAME ) == 0 )
	{
		xReturn = pdPASS;
	}
	else
	{
		xReturn = pdFAIL;
	}

	return xReturn;
}

void vApplicationMallocFailedHook( void )
{
volatile uint32_t ulMallocFailures = 0;

	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	ulMallocFailures++;
	taskDISABLE_INTERRUPTS();
	for( ;; );
}

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
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
