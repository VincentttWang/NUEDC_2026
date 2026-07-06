/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AllHeader.h"
#include "mode.h"
#include "visual.h"
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

/* USER CODE BEGIN PV */
// MS601M 传感器数据结构体
atk_ms601m_attitude_data_t attitude_dat;
atk_ms601m_gyro_data_t gyro_dat;
atk_ms601m_accelerometer_data_t accelerometer_dat;
char message[256]; 

//感为传感器数据变量
int Digtal[8];

MODE mode = {WAIT_MODE}; // 初始化模式为送药模式，位置为零
short drug_change = 1;
//用于标记是否需要进行药物模式的转化，
//只有在最开始的时候是1，以及在最后程序停止的时候是1，中间为0，即中间不需要检测是否装药
uint32_t mode_begin_t = 0;//记录模式开始的时间
char command_From_Bluetooth = 0; // 用于存储蓝牙接收到的命令

//测量过的转弯参数
const float r = 0.17;
const float tel = 26;

float current_yaw;//用于存储现在的yaw

const int back_delay = 300;//防止敲头，在后转之后停止一段时间
const float back_angle_cor = 10;//用于纠正陀螺仪的系统误差，当角度过小（逆时针不够很）

// 接收状态和缓冲区
uint8_t USART_RX_BUF[USART_RX_BUF_LEN] = {0}; 
// 真正接受数据的函数
uint8_t arr[25] = {0} ;
uint8_t USART_EYE_RX_BUF[USART_EYE_RX_LEN] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Initialize(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
bool CheckAndTurn(void);
bool CheckAndEnd(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
	//陀螺仪初始化
	atk_ms601m_init(115200);
	
	mode.loc.n = 0;
	// 电机和编码器初始化	
  Motor Left, Right;  
	MEInit(&Left, &Right); 
	
	// 接收中断初始化
  HAL_UARTEx_ReceiveToIdle_IT(&huart6, USART_RX_BUF, USART_RX_BUF_LEN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    IIC_Get_Digtal(Digtal); // 获取数字传感器数据,每一步都要执行以获取数据

    if(drug_change)
    {
      if(drugSet(&mode))// 进行药物模式的转化，如果转换成功进入if
      {
        drug_change = 0; // 转化完成后将标志位设为0
        mode_begin_t = HAL_GetTick(); // 记录模式开始的时间
      }
      continue; // 继续下一次循环
    }
    switch (mode.drug)
    {
      case WAIT_MODE:
        // 等待模式下的处理逻辑
        break;
      case PROPEL_MODE:
        if(command_From_Bluetooth != 'C')
        {
          if(!isWaitingLoc(mode.loc))
          {
            if(!CheckAndTurn())
              lineWalking(); // 进行循迹行走
          }
          else
          {
            if(!CheckAndEnd())
              lineWalking(); // 进行循迹行走
          }
        }
        else
        {
          if(isWaitingLoc(mode.loc))
          {
            if(!CheckAndTurn)
              Back(LocToTheta(mode.loc) + back_angle_cor); // 后退到上一个位置
          }
          else if(isEndOfWay(mode.loc)) //如果是除零点以外的终点          
          {
            if(!CheckAndEnd())
              lineWalking(); // 进行循迹行走
          }
          else // 如果不是终点
          {
            if(!CheckAndTurn()) // 检查是否需要结束当前模式
              lineWalking(); // 进行循迹行走
          }
        }
        break;
      default:
        // 未知模式的处理逻辑
        break;
    }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//用于给陀螺仪提供1us的定时
void delay_us_hal(uint16_t nus)
{
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    HAL_TIM_Base_Start(&htim6);       
    while (__HAL_TIM_GET_COUNTER(&htim6) < nus); 
    HAL_TIM_Base_Stop(&htim6);        
}


// 定时器中断回调函数，用于更新编码器速度
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 更新所有编码器的速度数据
    UpdateAllSpeed(htim);
}

bool CheckAndTurn()//所有check和turn函数都调用了dirget，在dirget中更新了mode.loc.trace和mode.loc.n
{
  if(HAL_GetTick() - mode_begin_t > 1000)
  {
    if(cross_Roads_Detect())
    {
      switch(DirGet(&mode)) // 获取下一个方向
      {
				case BACKWARD://在第二题会用到，目前不会用到
        case FORWARD:
          return true; // 继续前进
        case LEFT:
					current_yaw = getYaw();
          while(!isInTheYaw(sumTheta(current_yaw, 90), tel)) {runCircle(r, 0.5, 90, LEFT); HAL_Delay(10);}
          Break();
          return true; // 继续前进
        case RIGHT:
					current_yaw = getYaw();
          while(!isInTheYaw(sumTheta(current_yaw, -90), tel)){runCircle(r, 0.5, 90, RIGHT); HAL_Delay(10);}
          Break();
          return true; // 继续前进
      }
    }
  }
  return false; // 没有需要转向的情况
}
bool CheckAndEnd()
{
  if(half_Detect())
  {
    Break();
    drug_change = 1; // 设置标志位为1，表示需要进行药物模式的转化
    return true; // 需要结束当前模式
  }
  return false; // 没有需要结束的情况
}



void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART6)
  {
	  int j = 0 ;
	  for ( j = 0 ; j < Size ; j ++ )
	  {
		  arr[j] = USART_EYE_RX_BUF[j] ;
	  }
	  
	  
	  /*
	  核心函数,如下:
	  */
	  
	 // 设置房间存储数组
	  setRoomArray() ;
	  SampleGoalNumFromArr(arr) ;
	  control() ;
	  
	  
	  
    // 2.使用字符串匹配代替固定长度检查
 

	// 3. 发送回显（添加\r\n+正确长度+状态检查）
    const char *echo_str = "get!"; // 不换行的回显字符串
    uint16_t echo_len = strlen(echo_str); // 计算长度
	
    if (huart6.gState == HAL_UART_STATE_READY) 
	{ 
      HAL_UART_Transmit_IT(&huart6, (uint8_t *)echo_str, echo_len); // 检查UART是否就绪
    }

    // 4.重启接收前清除缓存区
    memset(USART_EYE_RX_BUF, 0, USART_EYE_RX_LEN);
	
	// 再次接收
    HAL_UARTEx_ReceiveToIdle_IT(&huart6, USART_EYE_RX_BUF, USART_EYE_RX_LEN);
  }
}


/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x24000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_128KB;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x30020000;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER3;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER4;
  MPU_InitStruct.BaseAddress = 0x38000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
