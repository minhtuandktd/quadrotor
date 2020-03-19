/* USER CODE BEGIN Header */
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu9255.h"
#include "IMU.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */





//Auto run parameter
#define 	AUTO_RUN														0				//1 = on, 0 = off
#define 	RUNNING_TIME												20			//seconds
							
							
//PID Parameters							
#define		KP_ROLL_PITCH												5.0f 		
#define		KI_ROLL_PITCH												0.0f  
#define		KD_ROLL_PITCH												0.0f 	
							
#define		KP_ROLL_PITCH_OMEGA									0.6 //1.1f 		//1.3f  
#define		KI_ROLL_PITCH_OMEGA									0.001f  //0.0052f;
#define		KD_ROLL_PITCH_OMEGA									5.0 //12.0f 	//12.6f;
#define 	ROLL_PITCH_MAX_INTEGRAL							200.0f
#define 	ROLL_PITCH_MAX_OUTPUT								400.0f


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

uint32_t count = 0, ii= 0 ,ga = 0, iii = 0, start = 0;
float channel_3 = 0, channel_5 = 0 , channel_1 = 0, channel_2 = 0;
uint16_t pid_roll_setpoint_base =0, pid_pitch_setpoint_base = 0;
float pid_roll_setpoint = 0, pid_pitch_setpoint = 0;
volatile uint16_t esc1 = 1000, esc2 = 1000 , esc3 = 1000, esc4 = 1000; // Cap PWM cho dong co : 1000-2000 
volatile float esc1_f = 0, esc2_f = 0, esc3_f =0, esc4_f =0, ga_f = 0;
extern uint8_t buffer3[21];
extern float roll, pitch, yaw, yaw_raw;
float yaw_setpoint = 0.0f;
extern float gyro_that[3];

/////////////////PID CONSTANTS/////////////////
/*----------ROLL------------*/
double kp_roll = KP_ROLL_PITCH;
double ki_roll = KI_ROLL_PITCH;
double kd_roll = KD_ROLL_PITCH;
float pid_output_roll = 0, pid_error_temp_roll = 0, pid_i_mem_roll = 0, pid_last_roll_d_error =0 ;
float pid_max_roll = 250.0f ;//25

float pid_output_roll_omega = 0, pid_error_temp_roll_omega = 0, pid_i_mem_roll_omega = 0, pid_last_roll_d_error_omega = 0 ;
double kp_roll_omega = KP_ROLL_PITCH_OMEGA;
double ki_roll_omega = KI_ROLL_PITCH_OMEGA;
double kd_roll_omega = KD_ROLL_PITCH_OMEGA ;

float pid_max_roll_omega_integral = ROLL_PITCH_MAX_INTEGRAL;
float pid_max_roll_omega = ROLL_PITCH_MAX_OUTPUT;

/*----------PITCH------------*/
double kp_pitch = KP_ROLL_PITCH;    
double ki_pitch = KI_ROLL_PITCH;  
double kd_pitch = KD_ROLL_PITCH;   
float pid_output_pitch = 0, pid_error_temp_pitch = 0, pid_i_mem_pitch = 0, pid_last_pitch_d_error = 0 ;
float pid_max_pitch = 250.0f ;

double kp_pitch_omega = KP_ROLL_PITCH_OMEGA;     
double ki_pitch_omega = KI_ROLL_PITCH_OMEGA;  
double kd_pitch_omega = KD_ROLL_PITCH_OMEGA;   
float pid_output_pitch_omega = 0, pid_error_temp_pitch_omega = 0, pid_i_mem_pitch_omega = 0, pid_last_pitch_d_error_omega = 0 ;

float pid_max_pitch_omega_integral = ROLL_PITCH_MAX_INTEGRAL;
float pid_max_pitch_omega = ROLL_PITCH_MAX_OUTPUT ;

/*----------YAW------------*/
float yaw_offset = 0, yaw_offset_tmp = 0;
uint8_t i_yaw = 0;
double kp_yaw = 0;
double ki_yaw = 0;
double kd_yaw = 0;
float pid_output_yaw = 0, pid_error_temp_yaw = 0, pid_i_mem_yaw = 0, pid_last_yaw_d_error = 0;
float pid_max_yaw = 100;

double kp_yaw_omega = 4.0f;
double ki_yaw_omega = 0.02f;
double kd_yaw_omega = 0.0f;
float pid_output_yaw_omega = 0, pid_error_temp_yaw_omega = 0, pid_i_mem_yaw_omega = 0, pid_last_yaw_d_error_omega = 0;
float pid_max_yaw_omega = 60.0f;
float pid_max_yaw_omega_integral = 40.0f;

//----------------------


uint16_t 	Gyroscope_X_offset=0,Gyroscope_Y_offset=0,Gyroscope_Z_offset=0,Accelerometer_X_offset=0,Accelerometer_Y_offset=0,Accelerometer_Z_offset=0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)
		{
			if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
			{
				if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9)==1)
				{
					__HAL_TIM_SetCounter(&htim1,0);
				}
				else
				{				
					channel_3 = __HAL_TIM_GetCompare(&htim1,TIM_CHANNEL_1);				
				}
			}
			
			if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
				{
				if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11)==1)
				{
					
					__HAL_TIM_SetCounter(&htim1,0);
				}
				else
				{
					channel_1 = __HAL_TIM_GetCompare(&htim1,TIM_CHANNEL_2);
				}
				}
			if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
				{
					if (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13)==1)
					{
						__HAL_TIM_SetCounter(&htim1,0);
					}
					else
					{
						channel_2 = __HAL_TIM_GetCompare(&htim1,TIM_CHANNEL_3);
					}
				}
			if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
				{
					if (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_14)==1)
					{
						__HAL_TIM_SetCounter(&htim1,0);
					}
					else
					{
						channel_5 = __HAL_TIM_GetCompare(&htim1,TIM_CHANNEL_4);
					}
				}
		}
}
/* PID CONTROLLER */
void PID_Controller_Angles(void)
{
	pid_roll_setpoint_base = channel_1;                                              //Normally channel_1 is the pid_roll_setpoint input.
  pid_pitch_setpoint_base = channel_2;                                             //Normally channel_2 is the pid_pitch_setpoint input.
	//We need a little dead band of 16us for better results. Channel_1 middle = 1485
  if (pid_roll_setpoint_base > 1493) pid_roll_setpoint = pid_roll_setpoint_base - 1493;
  else if (pid_roll_setpoint_base < 1477) pid_roll_setpoint = pid_roll_setpoint_base - 1477;
	pid_roll_setpoint = pid_roll_setpoint/15;
	
	//We need a little dead band of 16us for better results. Channel_2 middle = 1520
  if (pid_pitch_setpoint_base > 1528)pid_pitch_setpoint = pid_pitch_setpoint_base - 1528;
  else if (pid_pitch_setpoint_base < 1512)pid_pitch_setpoint = pid_pitch_setpoint_base - 1512;
	pid_pitch_setpoint = pid_pitch_setpoint/15;
	
	//Roll calculations///////////////////////////////////////////
	pid_error_temp_roll = pid_roll_setpoint - roll + 1.82f ;
			
	pid_i_mem_roll += ki_roll * pid_error_temp_roll;
	
	if (pid_i_mem_roll > pid_max_roll) pid_i_mem_roll = pid_max_roll;
	else 
			if (pid_i_mem_roll < pid_max_roll * -1) pid_i_mem_roll = pid_max_roll * -1;
	
	pid_output_roll = kp_roll * pid_error_temp_roll + pid_i_mem_roll + (-kd_roll)*(pid_error_temp_roll - pid_last_roll_d_error);
			
	if( pid_output_roll > pid_max_roll) pid_output_roll = pid_max_roll;
	else 
			if (pid_output_roll < pid_max_roll * -1) pid_output_roll = pid_max_roll * -1;
	
	pid_last_roll_d_error = pid_error_temp_roll;
	
	//Pitch calculations///////////////////////////////////////////
	pid_error_temp_pitch = pid_pitch_setpoint - pitch - 0.77f;
	
	pid_i_mem_pitch += ki_pitch * pid_error_temp_pitch;
	
	if (pid_i_mem_pitch > pid_max_pitch) pid_i_mem_pitch = pid_max_pitch;
	else 
			if (pid_i_mem_pitch < pid_max_pitch*-1) pid_i_mem_pitch = pid_max_pitch*-1;
		
	pid_output_pitch = kp_roll * pid_error_temp_pitch + pid_i_mem_pitch + (-kd_pitch)*(pid_error_temp_pitch - pid_last_pitch_d_error);
	
	if (pid_output_pitch > pid_max_pitch) pid_output_pitch = pid_max_pitch;
	else
			if (pid_output_pitch < pid_max_pitch*-1) pid_output_pitch = pid_max_pitch * -1;
	
	pid_last_pitch_d_error = pid_error_temp_pitch;
	
	//Yaw calculations/////////////////////////////////////////////
//	pid_error_temp_yaw = yaw_setpoint - yaw  ;
//	
//	pid_i_mem_yaw += ki_yaw*pid_error_temp_yaw;
//	
//	if (pid_i_mem_yaw > pid_max_yaw) pid_i_mem_yaw = pid_max_yaw;
//	else if (pid_i_mem_yaw < pid_max_yaw*(-1)) pid_i_mem_yaw = pid_max_yaw*(-1);
//	
//	pid_output_yaw = kp_yaw*pid_error_temp_yaw + pid_i_mem_yaw + (-kd_yaw)*(pid_error_temp_yaw - pid_last_yaw_d_error);
//	
//	if (pid_output_yaw > pid_max_yaw) pid_output_yaw = pid_max_yaw;
//	else if (pid_output_yaw < pid_max_yaw*(-1)) pid_output_yaw = pid_max_yaw*(-1);
//	
//	pid_last_yaw_d_error = pid_error_temp_yaw;



}

int sign_output_roll, pre_sign_output_roll = 1;
int sign_output_pitch, pre_sign_output_pitch = 1;
int sign_output_yaw, pre_sign_output_yaw = 1;

void PID_Controller_Omega(void)
{
	//Omega roll calculation//////////////////////////////////////////
	pid_error_temp_roll_omega = pid_output_roll - gyro_that[0] ;
	
	pid_i_mem_roll_omega += ki_roll_omega * pid_error_temp_roll_omega;
	
	if (pid_i_mem_roll_omega > pid_max_roll_omega_integral) pid_i_mem_roll_omega = pid_max_roll_omega_integral;
	else if (pid_i_mem_roll_omega < pid_max_roll_omega_integral * (-1)) pid_i_mem_roll_omega = pid_max_roll_omega_integral* (-1);
	
	pid_output_roll_omega = kp_roll_omega*pid_error_temp_roll_omega + pid_i_mem_roll_omega + kd_roll_omega * (pid_error_temp_roll_omega - pid_last_roll_d_error_omega);
	
	if (pid_output_roll_omega > pid_max_roll_omega) pid_output_roll_omega = pid_max_roll_omega;
	else if (pid_output_roll_omega < pid_max_roll_omega * (-1)) pid_output_roll_omega = pid_max_roll_omega * (-1);
	
	pid_last_roll_d_error_omega = pid_error_temp_roll_omega;
	pre_sign_output_roll = sign_output_roll;
	
	//Omega pitch calculation//////////////////////////////////////////
	pid_error_temp_pitch_omega = pid_output_pitch - gyro_that[1] ;
	
	pid_i_mem_pitch_omega += ki_pitch_omega * pid_error_temp_pitch_omega;

	if (pid_i_mem_pitch_omega > pid_max_pitch_omega_integral) pid_i_mem_pitch_omega = pid_max_pitch_omega_integral;
	else if (pid_i_mem_pitch_omega < pid_max_pitch_omega_integral * (-1)) pid_i_mem_pitch_omega = pid_max_pitch_omega_integral* (-1);
	
	pid_output_pitch_omega = kp_pitch_omega * pid_error_temp_pitch_omega + pid_i_mem_pitch_omega + kd_pitch_omega * (pid_error_temp_pitch_omega - pid_last_pitch_d_error_omega);
	
	if (pid_output_pitch_omega > pid_max_pitch_omega) pid_output_pitch_omega = pid_max_pitch_omega;
	else if (pid_output_pitch_omega < pid_max_pitch_omega * (-1)) pid_output_pitch_omega = pid_max_pitch_omega * (-1);
	
	pid_last_pitch_d_error_omega = pid_error_temp_pitch_omega;
	pre_sign_output_pitch = sign_output_pitch;

	//Omega yaw calculation//////////////////////////////////////////
	//pid_error_temp_yaw_omega = pid_output_yaw - gyro_that[2];
	
	pid_error_temp_yaw_omega = 0 - gyro_that[2];
	
	pid_i_mem_yaw_omega += ki_yaw_omega * pid_error_temp_yaw_omega;
	
	if (pid_i_mem_yaw_omega > pid_max_yaw_omega_integral) pid_i_mem_yaw_omega = pid_max_yaw_omega_integral;
	else if (pid_i_mem_yaw_omega < pid_max_yaw_omega_integral * (-1)) pid_i_mem_yaw_omega = pid_max_yaw_omega_integral * (-1);
	
	pid_output_yaw_omega = kp_yaw_omega * pid_error_temp_yaw_omega + pid_i_mem_yaw_omega + kd_yaw_omega * (pid_error_temp_yaw_omega - pid_last_yaw_d_error_omega);
	
	if (pid_output_yaw_omega > pid_max_yaw_omega) pid_output_yaw_omega = pid_max_yaw_omega;
	else if (pid_output_yaw_omega < pid_max_yaw_omega * (-1)) pid_output_yaw_omega = pid_max_yaw_omega * (-1);
	
	pid_last_yaw_d_error_omega = pid_error_temp_yaw_omega;
}
void Reset_PID(void)
{
	pid_i_mem_roll_omega = pid_i_mem_pitch_omega = pid_i_mem_yaw_omega = 0.0f;
	pid_last_roll_d_error_omega = pid_last_pitch_d_error_omega = pid_last_yaw_d_error_omega = 0.0f;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t auto_run_counter_temp = 0, auto_run_counter = 0;
uint8_t control_counter = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{	
		MPU9255_Get_Data();
		IMU_GetYawPitchRoll();

#if AUTO_RUN
		if (start == 1)
		{
			auto_run_counter_temp++;
			if (auto_run_counter_temp == 250){
				auto_run_counter++;
				if (auto_run_counter == RUNNING_TIME){
					start = 0;
					break;
				}
				auto_run_counter_temp = 0;
			}
#endif
			
			if ( channel_5 > 1500)
				{
					control_counter++;
					
					if (control_counter == 4)  //Change the compared value to change the speed of angle control loop
					{								
						PID_Controller_Angles();
						control_counter = 0;
					}
					
					PID_Controller_Omega();
					ga = channel_3 ;	
					if (ga > 1500) ga = 1500;
					
					esc1 = ga - pid_output_roll_omega - pid_output_pitch_omega - pid_output_yaw_omega;   //MPU dat giua 2 truc
					esc2 = ga - pid_output_roll_omega + pid_output_pitch_omega + pid_output_yaw_omega;
					esc3 = ga + pid_output_roll_omega + pid_output_pitch_omega - pid_output_yaw_omega;
					esc4 = ga + pid_output_roll_omega - pid_output_pitch_omega + pid_output_yaw_omega;
					
					if (esc1 < 1100) esc1 = 1100;                                                 //Keep the motors running.
					if (esc2 < 1100) esc2 = 1100;                                                 //Keep the motors running.
					if (esc3 < 1100) esc3 = 1100;                                                 //Keep the motors running.
					if (esc4 < 1100) esc4 = 1100;                                                 //Keep the motors running.

					if (esc1 > 1800) esc1 = 1800;                                                 //Limit the esc-1 pulse to 2000us.
					if (esc2 > 1800) esc2 = 1800;                                                 //Limit the esc-2 pulse to 2000us.
					if (esc3 > 1800) esc3 = 1800;                                                 //Limit the esc-3 pulse to 2000us.
					if (esc4 > 1800) esc4 = 1800;                                                 //Limit the esc-4 pulse to 2000us.
					
				}
			else 
				{
					Reset_PID();
					esc1 = 1000;
					esc2 = 1000;
					esc3 = 1000;
					esc4 = 1000;
					
				}
#if AUTO_RUN
		}
		else {
			Reset_PID();
			esc1 = 1000;
			esc2 = 1000;
			esc3 = 1000;
			esc4 = 1000;
		}
#endif

	/*----Xuat PWM ra dong co -----*/
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1 , esc3 ); //PA0
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2 , esc1 ); //PA1
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3 , esc2 ); //PA2
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4 , esc4 ); //PA3				

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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	MPU9255_Init();
	InitGyrOffset();
	
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	
	/*----Xuat PWM ra dong co -----*/
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1 , 1000 ); //PA0
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2 , 1000 ); //PA1
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3 , 1000 ); //PA2
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4 , 1000 ); //PA3
	HAL_Delay(2500);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
	
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim3);
	
	 
  /* USER CODE END 2 */

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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 29999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
