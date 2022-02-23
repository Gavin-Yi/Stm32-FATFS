/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    FS_WELCOME=0,
    FS_NAME_FILE,
    FS_NAME_PATH,
    FS_DELETE,
    FS_TEXT,
    FS_READ
} FS_STATUS;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define print(str) HAL_UART_Transmit_DMA(&huart1, (uint8_t *)(str), strlen(str)); osDelay(10)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId WriteTaskHandle;
osThreadId SendTaskHandle;
/* USER CODE BEGIN PV */
uint8_t SerialBuf[BUFFERSIZE];
unsigned char bRecFlag = 0;
unsigned char iLength = 0;
FS_STATUS Status = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
void StartWriteTask(void const * argument);
void StartSendTask(void const * argument);

/* USER CODE BEGIN PFP */
void WARNING(FRESULT res);
void Welcome_Page(void);
FRESULT scan_files(const TCHAR*);
//FRESULT split_file_path(const uint8_t *file_path, char* file_name);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void WARNING(FRESULT res)
{
    char local_temp[64];
    sprintf(local_temp, "Something wrong. Code: %d\r\n", res);
    print(local_temp);
    print("Return to Welcome Page.\r\n");
}

void Welcome_Page(void)
{
    print("Welcome to the FATFS System. Please choose your operation.\r\n");
    print("(For example, Input 'C1' for Operation 1)\r\n");
    print("1. List all files currently\r\n");
    print("2. Create a new file\r\n");
    print("3. Create a new directory\r\n");
    print("4. Delete a existed file\r\n");
    print("5. Modify a file (Enter 'q/Q' to quit)\r\n");
    print("6. Read a file\r\n");
    print("********************\r\n");
}

FRESULT scan_files(const TCHAR* path)
{
    static FILINFO fno;     // Define file information obj
    FRESULT res;            // Define result obj
    DIR dir;                // Define directory obj
    char pathBuf[32]={0};      // Define path array
    char fileBuf[64]={0};       // Define file array
    unsigned char i;

    memcpy(pathBuf, path, strlen(path));
    res = f_opendir(&dir, path);
    if(res == FR_OK)
    {
        for(;;)
        {
            res = f_readdir(&dir, &fno); // Read directory, Return the result and point to file information
            if(res != FR_OK || fno.fname[0] == 0) // Open failed or end of diretory
                break;
            if(fno.fattrib == AM_DIR)   // This is a directory instead of file
            {
                i = strlen(pathBuf);    // Get the path's original length
                sprintf(&pathBuf[i], "/%s", fno.fname);
                res = scan_files(pathBuf);
                if(res != FR_OK)
                    break;
                pathBuf[i] = 0;
            }
            else
            {
                sprintf(fileBuf, "--%s/%s\r\n", path, fno.fname);
                print(fileBuf);
            }
        }
    }
    else
    {
        char temp[64];
        sprintf(temp, "Open Directory Failed. Please Check: %s\r\n", (const char *)(res+'0'));
        print(temp);
    }

    f_closedir(&dir);   // close directory
    return res;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

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
    MX_SDIO_SD_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();
    MX_FATFS_Init();
    /* USER CODE BEGIN 2 */
    print("******** Stm32-FATFS Program ********\r\n");
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

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* definition and creation of WriteTask */
    osThreadDef(WriteTask, StartWriteTask, osPriorityNormal, 0, 1024);
    WriteTaskHandle = osThreadCreate(osThread(WriteTask), NULL);

    /* definition and creation of SendTask */
    osThreadDef(SendTask, StartSendTask, osPriorityNormal, 0, 1024);
    SendTaskHandle = osThreadCreate(osThread(SendTask), NULL);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

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
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 128;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 8;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

    /* USER CODE BEGIN SDIO_Init 0 */

    /* USER CODE END SDIO_Init 0 */

    /* USER CODE BEGIN SDIO_Init 1 */

    /* USER CODE END SDIO_Init 1 */
    hsd.Instance = SDIO;
    hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
    hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
    hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
    hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
    hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd.Init.ClockDiv = 4;
    /* USER CODE BEGIN SDIO_Init 2 */

    /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
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
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA2_Stream2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    /* DMA2_Stream3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    /* DMA2_Stream6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
    /* DMA2_Stream7_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartWriteTask */
/**
  * @brief  Function implementing the WriteTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartWriteTask */
void StartWriteTask(void const * argument)
{
    /* USER CODE BEGIN 5 */
    /* Infinite loop */
    for(;;)
    {
        osDelay(1000);
    }
    /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSendTask */
/**
* @brief Function implementing the SendTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSendTask */
void StartSendTask(void const * argument)
{
    /* USER CODE BEGIN StartSendTask */
    FATFS FatFs;
    // The Controller state machine
    HAL_UART_Receive_DMA(&huart1, (uint8_t *)SerialBuf, sizeof SerialBuf);
    // Enter Welcome Page if SD mounted succeed
    if(f_mount(&FatFs, SDPath, 0) == FR_OK)
        Welcome_Page();
    /* Infinite loop */
    for(;;)
    {
        if(bRecFlag)
        {
            // 1. Receive
            uint8_t datalength = BUFFERSIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
            iLength += datalength;
            // 2. Analyze message
            char temp[2] = {0};
            switch (Status)
            {
                case FS_WELCOME:
                    if( datalength != 2)
                    {
                        print("Invalid input. Please input again.\r\n");
                        break;
                    }
                    memcpy(temp, SerialBuf, 2);
                    if(memcmp(temp, "C1", 2) == 0)
                    {
                        // List all files
                        scan_files("0:");
                    }else if(memcmp(temp, "C2", 2) == 0)
                    {
                        // Create a new file
                        print("Enter complete path of the file to Create, like '0:/02-22/Test.TXT'\r\n");
                        Status = FS_NAME_FILE;
                    }else if(memcmp(temp, "C3", 2) == 0)
                    {
                        // Create a new directory
                        print("Enter name of the directory, like '02-22'\r\n");
                        Status = FS_NAME_PATH;
                    }else if(memcmp(temp, "C4", 2) == 0)
                    {
                        // Delete a existed file
                        print("Enter complete path of the file to Delete, like '0:/02-22/Test.TXT'\r\n");
                        Status = FS_DELETE;
                    }else if(memcmp(temp, "C5", 2) == 0)
                    {
                        // Modify a file
                        print("Enter complete path of the file to Modify, like '0:/02-22/Test.TXT'\r\n");
                        Status = FS_TEXT;
                    }else if(memcmp(temp, "C6", 2) == 0)
                    {
                        print("Enter complete path of the file to Read, like '0:/02-22/Test.TXT'\r\n");
                        Status = FS_READ;
                    }
                    break;
                case FS_NAME_FILE:
                {
                    // 1. Split the name, directory
                    TCHAR file_name[32]={0};
                    FRESULT res;
                    memcpy(file_name, SerialBuf, strlen((const char *)SerialBuf));
                    FIL file={0};
                    // 2. Create the new file
                    res = f_open(&file, file_name, FA_CREATE_NEW | FA_READ | FA_WRITE);
                    if(res == FR_OK)
                    {
                        print("File Created!\r\n");
                        f_close(&file);
                    } else WARNING(res);
                    // 3. Return to Welcome Page
                    Status = FS_WELCOME;
                    break;
                }
                case FS_NAME_PATH:  // Basically same with FS_NAME_FILE
                {
                    // 1. Get the path
                    char path[32]={0};
                    FRESULT res;
                    memcpy(path, SerialBuf, strlen((const char *)SerialBuf));
                    // 2. Create the new directory
                    res = f_mkdir(path);
                    if(res == FR_OK)
                    {
                        print("Create Directory Succeed!\r\n");
                    } else WARNING(res);
                    // 3. Return to Welcome Page
                    Status = FS_WELCOME;
                    break;
                }
                case FS_DELETE:
                {
                    // 1. Get the path
                    FRESULT res;
                    char path[32]={0};
                    memcpy(path, SerialBuf, strlen((const char *)SerialBuf));
                    // 2. Create the new directory
                    res = f_unlink(path);
                    if(res == FR_OK)
                    {
                        print("File Deleted!\r\n");
                    } else WARNING(res);
                    // 3. Return to Welcome Page
                    Status = FS_WELCOME;
                    break;
                }
                case FS_TEXT:
                {
                    static char path[32] = {0};
                    char writeBuf[BUFFERSIZE] = {0};
                    FIL file={0};

                    // 0. Get File Name at the first time
                    if((*path) == '\0')
                    {
                        memcpy(path, SerialBuf, strlen((const char *)SerialBuf));
                        print("Enter Your words row-wise: \r\n");
                        break;
                    }
                    // 1. Check if jump out
                    if(iLength == 1 && (SerialBuf[0] == 'q' || SerialBuf[0] == 'Q'))
                    {
                        print("Files Modified\r\n");
                        memset(path, 0, 32);
                        Status = FS_WELCOME;
                        break;
                    }
                    // 2. Open file in Append mode and write it
                    if( f_open(&file, path, FA_OPEN_APPEND | FA_READ | FA_WRITE) == FR_OK)
                    {
                        unsigned char len = strlen(SerialBuf);
                        // Make sure len does not cross the border
                        len = len < 254 ? len : len - 2;
                        memcpy(writeBuf, SerialBuf, len);
                        // To add a new line at the end of each sentence.
                        writeBuf[len] = '\r';
                        writeBuf[len+1] = '\n';
                        f_write(&file, writeBuf, len+2, NULL);
                        f_close(&file);
                    }
                    break;
                }
                case FS_READ:
                {
                    char len = 0 ;
                    char path[32] = {0};
                    char readBuf[BUFFERSIZE] = {0};
                    FIL file={0};

                    // 0. Get File Name at the first time
                    memcpy(path, SerialBuf, strlen((const char *)SerialBuf));

                    // 1. Open file in READ mode
                    if( f_open(&file, path, FA_READ) == FR_OK)
                    {
                        while(!f_eof(&file))
                        {
                            // f_gets() automatically moves the file pointer
                            // no need to increase the flseek
                            f_gets(readBuf, BUFFERSIZE, &file);
                            print(readBuf);
                            memset(readBuf, 0, BUFFERSIZE);
                        }
                    }
                    // 2. Return to Welcome Page
                    Status = FS_WELCOME;
                    break;
                }
                default:
                    break;
            }

            // 3. Clear Receive buff
            memset(SerialBuf, 0, iLength);
            iLength = 0;
            // 4. Return to receive uart data
            bRecFlag = 0;
            HAL_DMA_DeInit(&hdma_usart1_rx);
            HAL_DMA_Init(&hdma_usart1_rx);
            HAL_UART_Receive_DMA(&huart1, (uint8_t*)SerialBuf, BUFFERSIZE);
        }

    }
    /* USER CODE END StartSendTask */
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
