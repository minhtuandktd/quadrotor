/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "dwt_stm32_delay.h"
#include "adns3080.h"
#include <math.h>
//#include "PWM_to_F.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define 	HCSR04_NUMBER					((float)0.0171821)

//User defined parameters
#define		TX_CONTROLLER					1 				//1 = Use controller to start, 0 = Don't use controller
#define		RUNNING_TIME					20				//Run time before auto stop on auto run mode
#define		AUTO_RUN							1					//1 = Turn on auto run, 0 = Turn off auto run
#define 	YAW_CONTROLLER				1
#define 	ALTITUDE_CONTROLLER		1

#if				!YAW_CONTROLLER
#define 	YAW_OFFSET_VALUE			0.0f
#endif

//PID Parameters
#define		KP_ROLL_PITCH					2.8f

#define		KP_ROLL_PITCH_OMEGA		2.5f
#define		KI_ROLL_PITCH_OMEGA		0.0112f
#define		KD_ROLL_PITCH_OMEGA		12.3f
#define		PID_MAX								250.0f
#define		PID_MAX_INTEGRAL			200.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;

/* USER CODE BEGIN PV */
uint32_t count = 0, ga = 0, start = 0;
float channel_1 = 0, channel_2 = 0, channel_3 = 0, channel_4, channel_5 = 0, channel_6 = 0;
volatile uint16_t esc1 = 1000, esc2 = 1000 , esc3 = 1000, esc4 = 1000; // Cap PWM cho dong co : 1000-2000 
volatile float esc1_f = 0, esc2_f = 0, esc3_f = 0, esc4_f = 0, ga_f = 0;
extern uint8_t buffer3[21];
extern float roll, pitch, yaw, yaw_raw;
float yaw_setpoint = 0.0f;
extern float gyro_that[3];
int32_t m_position_x = 0, m_position_y = 0;
float distance = 0;

/////////////////PID CONSTANTS/////////////////
/*----------ROLL------------*/
double kp_roll = KP_ROLL_PITCH;//1.8
double ki_roll = 0.0f;
double kd_roll = 0.0f;
float pid_output_roll = 0, pid_error_temp_roll = 0, pid_i_mem_roll = 0, pid_last_roll_d_error =0 ;
float pid_max_roll = 250.0f ;//25

float pid_output_roll_omega = 0, pid_error_temp_roll_omega = 0, pid_i_mem_roll_omega = 0, pid_last_roll_d_error_omega = 0 ;
double kp_roll_omega = KP_ROLL_PITCH_OMEGA;//1.1
double ki_roll_omega = KI_ROLL_PITCH_OMEGA;//0.0052f;
double kd_roll_omega = KD_ROLL_PITCH_OMEGA;//12.6f;

float pid_max_roll_omega_integral = PID_MAX_INTEGRAL;
float pid_max_roll_omega = PID_MAX;

/*----------PITCH------------*/
double kp_pitch = KP_ROLL_PITCH;    
double ki_pitch = 0.0f;  
double kd_pitch = 0.0f;   
float pid_output_pitch = 0, pid_error_temp_pitch = 0, pid_i_mem_pitch = 0, pid_last_pitch_d_error =0 ;
float pid_max_pitch = 250.0f ;

double kp_pitch_omega = KP_ROLL_PITCH_OMEGA;     
double ki_pitch_omega = KI_ROLL_PITCH_OMEGA;  
double kd_pitch_omega = KD_ROLL_PITCH_OMEGA;   
float pid_output_pitch_omega = 0, pid_error_temp_pitch_omega = 0, pid_i_mem_pitch_omega = 0, pid_last_pitch_d_error_omega =0 ;

float pid_max_pitch_omega_integral = PID_MAX_INTEGRAL;
float pid_max_pitch_omega = PID_MAX;

/*----------YAW------------*/
float yaw_offset = 0, yaw_offset_tmp = 0;
uint8_t i_yaw = 0;
double kp_yaw = 0.5;
double ki_yaw = 0;
double kd_yaw = 0;
float pid_output_yaw = 0, pid_error_temp_yaw = 0, pid_i_mem_yaw = 0, pid_last_yaw_d_error = 0;
float pid_max_yaw = 100;

double kp_yaw_omega = 1.5f;
double ki_yaw_omega = 0.005f;
double kd_yaw_omega = 6.0f;
float pid_output_yaw_omega = 0, pid_error_temp_yaw_omega = 0, pid_i_mem_yaw_omega = 0, pid_last_yaw_d_error_omega = 0;
float pid_max_yaw_omega = 200.0f;
float pid_max_yaw_omega_integral = 100.0f;

//----------------------
//extern TM_MPU6050_t	MPU6050_Data0;
//extern TM_HMC_t			HMC_Data;

uint16_t 	Gyroscope_X_offset=0,Gyroscope_Y_offset=0,Gyroscope_Z_offset=0,Accelerometer_X_offset=0,Accelerometer_Y_offset=0,Accelerometer_Z_offset=0;

uint8_t controller_start = 0;
uint32_t running_timer = 0, running_timer_counter = 0, start_running = 0;
uint32_t time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
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
					channel_3 = __HAL_TIM_GetCompare(&htim1,TIM_CHANNEL_1);		//Throttle			
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
					channel_1 = __HAL_TIM_GetCompare(&htim1,TIM_CHANNEL_2);		//Roll control channel
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
					channel_2 = __HAL_TIM_GetCompare(&htim1,TIM_CHANNEL_3);		//Pitch control channel
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
					channel_5 = __HAL_TIM_GetCompare(&htim1,TIM_CHANNEL_4);		//Start channel
				}
			}
		}
		else if (htim->Instance == TIM9){
			if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
				if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == 1){
					__HAL_TIM_SetCounter(&htim9, 0);
				}
				else{
					time = __HAL_TIM_GetCompare(&htim9, TIM_CHANNEL_1);
				}
			}
		}
}
/* PID CONTROLLER */
uint16_t pid_roll_setpoint_base = 0, pid_pitch_setpoint_base = 0;
float pid_roll_setpoint = 0, pid_pitch_setpoint = 0;
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
	pid_error_temp_roll = pid_roll_setpoint - roll ;
			
	pid_i_mem_roll += ki_roll * pid_error_temp_roll;
	
	if (pid_i_mem_roll > pid_max_roll) pid_i_mem_roll = pid_max_roll;
	else 
			if (pid_i_mem_roll < pid_max_roll * -1) pid_i_mem_roll = pid_max_roll * -1;
	
	pid_output_roll = kp_roll * pid_error_temp_roll + pid_i_mem_roll + (-kd_roll)*(pid_error_temp_roll - pid_last_roll_d_error);
			
//	if( pid_output_roll > pid_max_roll) pid_output_roll = pid_max_roll;
//	else 
//			if (pid_output_roll < pid_max_roll * -1) pid_output_roll = pid_max_roll * -1;
	
	pid_last_roll_d_error = pid_error_temp_roll;
	
	//Pitch calculations///////////////////////////////////////////
	pid_error_temp_pitch = pid_pitch_setpoint - pitch ;
	
	pid_i_mem_pitch += ki_pitch * pid_error_temp_pitch;
	
	if (pid_i_mem_pitch > pid_max_pitch) pid_i_mem_pitch = pid_max_pitch;
	else 
			if (pid_i_mem_pitch < pid_max_pitch*-1) pid_i_mem_pitch = pid_max_pitch*-1;
		
	pid_output_pitch = kp_roll * pid_error_temp_pitch + pid_i_mem_pitch + (-kd_pitch)*(pid_error_temp_pitch - pid_last_pitch_d_error);
	
//	if (pid_output_pitch > pid_max_pitch) pid_output_pitch = pid_max_pitch;
//	else
//			if (pid_output_pitch < pid_max_pitch*-1) pid_output_pitch = pid_max_pitch * -1;
	
	pid_last_pitch_d_error = pid_error_temp_pitch;
	
	//Yaw calculations/////////////////////////////////////////////
	pid_error_temp_yaw = 0 - yaw  ;
	
	pid_i_mem_yaw += ki_yaw*pid_error_temp_yaw;
	
	if (pid_i_mem_yaw > pid_max_yaw) pid_i_mem_yaw = pid_max_yaw;
	else if (pid_i_mem_yaw < pid_max_yaw*(-1)) pid_i_mem_yaw = pid_max_yaw*(-1);
	
	pid_output_yaw = kp_yaw*pid_error_temp_yaw + pid_i_mem_yaw + (-kd_yaw)*(pid_error_temp_yaw - pid_last_yaw_d_error);
	
	if (pid_output_yaw > pid_max_yaw) pid_output_yaw = pid_max_yaw;
	else if (pid_output_yaw < pid_max_yaw*(-1)) pid_output_yaw = pid_max_yaw*(-1);
	
	pid_last_yaw_d_error = pid_error_temp_yaw;

	count++;

}

void PID_Controller_Omega(void)
{
	//Omega roll calculation//////////////////////////////////////////
	pid_error_temp_roll_omega = pid_output_roll - gyro_that[0] ;
	
//	sign_output_roll = getsign(pid_output_roll);
//	if (sign_output_roll == pre_sign_output_roll){
		pid_i_mem_roll_omega += ki_roll_omega * pid_error_temp_roll_omega;
//	}
//	else pid_i_mem_roll_omega = 0;
	
	if (pid_i_mem_roll_omega > pid_max_roll_omega_integral) pid_i_mem_roll_omega = pid_max_roll_omega_integral;
	else if (pid_i_mem_roll_omega < pid_max_roll_omega_integral * (-1)) pid_i_mem_roll_omega = pid_max_roll_omega_integral * (-1);
	
	pid_output_roll_omega = kp_roll_omega*pid_error_temp_roll_omega + pid_i_mem_roll_omega + kd_roll_omega * (pid_error_temp_roll_omega - pid_last_roll_d_error_omega);
	
	if (pid_output_roll_omega > pid_max_roll_omega) pid_output_roll_omega = pid_max_roll_omega;
	else if (pid_output_roll_omega < pid_max_roll_omega * (-1)) pid_output_roll_omega = pid_max_roll_omega * (-1);
	
	pid_last_roll_d_error_omega = pid_error_temp_roll_omega;
//	pre_sign_output_roll = sign_output_roll;
	
	//Omega pitch calculation//////////////////////////////////////////
	pid_error_temp_pitch_omega = pid_output_pitch - gyro_that[1] ;
	
//	sign_output_pitch = getsign(pid_output_pitch);
//	if (sign_output_pitch == pre_sign_output_pitch){
		pid_i_mem_pitch_omega += ki_pitch_omega * pid_error_temp_pitch_omega;
//	}
//	else pid_i_mem_pitch_omega = 0;
	
	if (pid_i_mem_pitch_omega > pid_max_pitch_omega_integral) pid_i_mem_pitch_omega = pid_max_pitch_omega_integral;
	else if (pid_i_mem_pitch_omega < pid_max_pitch_omega_integral * (-1)) pid_i_mem_pitch_omega = pid_max_pitch_omega_integral * (-1);
	
	pid_output_pitch_omega = kp_pitch_omega * pid_error_temp_pitch_omega + pid_i_mem_pitch_omega + kd_pitch_omega * (pid_error_temp_pitch_omega - pid_last_pitch_d_error_omega);
	
	if (pid_output_pitch_omega > pid_max_pitch_omega) pid_output_pitch_omega = pid_max_pitch_omega;
	else if (pid_output_pitch_omega < pid_max_pitch_omega * (-1)) pid_output_pitch_omega = pid_max_pitch_omega * (-1);
	
	pid_last_pitch_d_error_omega = pid_error_temp_pitch_omega;
//	pre_sign_output_pitch = sign_output_pitch;

	//Omega yaw calculation//////////////////////////////////////////
	pid_error_temp_yaw_omega = pid_output_yaw - gyro_that[2];
	
//	sign_output_yaw = getsign(pid_output_yaw);
//	if (sign_output_yaw == pre_sign_output_yaw){
		pid_i_mem_yaw_omega += ki_yaw_omega * pid_error_temp_yaw_omega;
//	}
//	else pid_i_mem_yaw_omega = 0;
	
	if (pid_i_mem_yaw_omega > pid_max_yaw_omega_integral) pid_i_mem_yaw_omega = pid_max_yaw_omega;
	else if (pid_i_mem_yaw_omega < pid_max_yaw_omega_integral * (-1)) pid_i_mem_yaw_omega = pid_max_yaw_omega * (-1);
	
	pid_output_yaw_omega = kp_yaw_omega * pid_error_temp_yaw_omega + pid_i_mem_yaw_omega + kd_yaw_omega * (pid_error_temp_yaw_omega - pid_last_yaw_d_error_omega);
	
	if (pid_output_yaw_omega > pid_max_yaw_omega) pid_output_yaw_omega = pid_max_yaw_omega;
	else if (pid_output_yaw_omega < pid_max_yaw_omega * (-1)) pid_output_yaw_omega = pid_max_yaw_omega * (-1);
	
	pid_last_yaw_d_error_omega = pid_error_temp_yaw_omega;
//	pre_sign_output_yaw = sign_output_yaw; 

}

float altitude_error = 0.0f, altitude_error_pre = 0.0f, throttle_output = 0.0f, throttle_output_max = 1000.0f;
float kp_altitude = 0.0f, ki_altitude = 0.0f, kd_altitude = 0.0f;
float pid_altitude_integral, pid_altitude_integral_max;

void PID_Controller_Altitude(void){
	altitude_error = (channel_3 * 0.225f - 250.0f) - distance;
	
	pid_altitude_integral += ki_altitude * altitude_error;
	if (pid_altitude_integral > pid_altitude_integral_max) pid_altitude_integral = pid_altitude_integral_max;
	else if (pid_altitude_integral < pid_altitude_integral_max * (-1)) pid_altitude_integral = pid_altitude_integral_max * (-1);
	
	throttle_output = kp_altitude * altitude_error + pid_altitude_integral + kd_altitude * (altitude_error - altitude_error_pre);
	if (throttle_output > throttle_output_max) throttle_output = throttle_output_max;
	else if (throttle_output < 0) throttle_output = 0.0f;
	
	altitude_error_pre = altitude_error;
}
		
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t control_delay;
uint32_t w_control_timer, w_control_timer_raw, w_control_timer_pre;
uint16_t inner_controller_counter = 0, outer_controller_counter = 0, outer_outer_controller_counter = 0;
uint8_t position_counter = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		position_counter++;
		if (position_counter == 10){
			adns_get_data(hspi1);
			m_position_x = get_position_x();
			m_position_y = get_position_y();
			position_counter = 0;
		}
#if AUTO_RUN
		if (start == 1){
#endif			
			MPU9255_Get_Data();
			IMU_GetYawPitchRoll(); 
#if AUTO_RUN			
			running_timer_counter++;
			if (running_timer_counter == 500){
				running_timer++;
				running_timer_counter = 0;
			}
			if (running_timer == RUNNING_TIME){
				start = 0;
			}
#endif

#if TX_CONTROLLER	
		if (channel_5 > 1500)
		{
#endif
			inner_controller_counter++;
			if (inner_controller_counter == 10){
				inner_controller_counter = 0;
				outer_controller_counter++;
				if (outer_controller_counter == 3){
					PID_Controller_Angles();
					outer_controller_counter = 0;
					outer_outer_controller_counter++;
					if (outer_outer_controller_counter == 2){
						outer_outer_controller_counter = 0;
						PID_Controller_Altitude();
					}
				}
				PID_Controller_Omega();
#if !ALTITUDE_CONTROLLER						
#if TX_CONTROLLER
				if (channel_3 > 1500) 
					ga = 1500;
				else 
					ga = channel_3;
#else
				ga = 1350;
#endif
#else
				if (channel_3 > 1200){
					ga = 1000 + throttle_output;
				}
				else ga = 0;
#endif
				
#if YAW_CONTROLLER				
				esc1 = ga - pid_output_roll_omega - pid_output_pitch_omega - pid_output_yaw_omega;   //MPU dat giua 2 truc
				esc2 = ga - pid_output_roll_omega + pid_output_pitch_omega + pid_output_yaw_omega;
				esc3 = ga + pid_output_roll_omega + pid_output_pitch_omega - pid_output_yaw_omega;
				esc4 = ga + pid_output_roll_omega - pid_output_pitch_omega + pid_output_yaw_omega;
#else
				esc1 = ga - pid_output_roll_omega - pid_output_pitch_omega - YAW_OFFSET_VALUE;   //MPU dat giua 2 truc
				esc2 = ga - pid_output_roll_omega + pid_output_pitch_omega + YAW_OFFSET_VALUE;
				esc3 = ga + pid_output_roll_omega + pid_output_pitch_omega - YAW_OFFSET_VALUE;
				esc4 = ga + pid_output_roll_omega - pid_output_pitch_omega + YAW_OFFSET_VALUE;	
#endif
				
				if (esc1 < 1100) esc1 = 1100;                                                //Keep the motors running.
				if (esc2 < 1100) esc2 = 1100;                                                //Keep the motors running.
				if (esc3 < 1100) esc3 = 1100;                                                //Keep the motors running.
				if (esc4 < 1100) esc4 = 1100;                                                //Keep the motors running.
                             
				if (esc1 > 1750) esc1 = 1750;                                                 //Limit the esc-1 pulse to 2000us.
				if (esc2 > 1750) esc2 = 1750;                                                 //Limit the esc-2 pulse to 2000us.
				if (esc3 > 1750) esc3 = 1750;                                                 //Limit the esc-3 pulse to 2000us.
				if (esc4 > 1750) esc4 = 1750;                                                 //Limit the esc-4 pulse to 2000us.
			}
#if TX_CONTROLLER							
		}
		else 
			{
				Reset_PID();
				esc1 = 1000;
				esc2 = 1000;
				esc3 = 1000;
				esc4 = 1000;
				
			}
#endif

#if AUTO_RUN
		}
		else {
			Reset_PID();
			esc1 = esc2 = esc3 = esc4 = 1000;
		}
#endif
		/*----Xuat PWM ra dong co -----*/
				
		esc1 = esc3 = 1000;
		//esc2 = esc4 = 1000;
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
  MX_SPI1_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
	Init_button_and_led();
	DWT_Delay_Init();
//	MPU9255_Init();
//	InitGyrOffset();
	adns_init(hspi1);
	
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	
	/*----Xuat PWM ra dong co -----*/
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1 , 1000 ); //PA0
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2 , 1000 ); //PA1
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3 , 1000 ); //PA2
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4 , 1000 ); //PA3
	HAL_Delay(2000);
	
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
	HAL_TIM_IC_Start_IT(&htim9, TIM_CHANNEL_1);
//	control_delay = TIM1->CNT;
//	yaw_setpoint = yaw_raw;
//	while ((TIM1->CNT - control_delay) < 500) {};
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
	start = 1;
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
		DWT_Delay_us(1);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		DWT_Delay_us(10);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);		
		
		distance = (float)time * HCSR04_NUMBER; // * 1.226891;
		
			
		DWT_Delay_ms(50);
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 0;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 83;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 9999;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 4;
  if (HAL_TIM_IC_ConfigChannel(&htim9, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void Init_button_and_led(void){
		GPIO_InitTypeDef GPIO_Init;
	
	__HAL_RCC_GPIOD_CLK_ENABLE();
	
	GPIO_Init.Pin = GPIO_PIN_12;
	GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_Init.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_Init.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOD, &GPIO_Init);
	
	GPIO_Init.Pin = GPIO_PIN_13;
	HAL_GPIO_Init(GPIOD, &GPIO_Init);
	
	GPIO_Init.Pin = GPIO_PIN_14;
	HAL_GPIO_Init(GPIOD, &GPIO_Init);

	GPIO_Init.Pin = GPIO_PIN_15;
	HAL_GPIO_Init(GPIOD, &GPIO_Init);

}

int getsign(float c){
	if (c >= 0) return 1;
	else return -1;
}

float wrap_to_range_f(const float low, const float high, float x)
{
  /* Wrap x into interval [low, high) */
  /* Assumes high > low */

  const float range = high - low;

  if (range > 0.0f) {
    while (x >= high) {
      x -= range;
    }

    while (x < low) {
      x += range;
    }
  } else {
    x = low;
  }

  return x;
}

void Reset_PID(void)
{
	pid_i_mem_roll_omega = pid_i_mem_pitch_omega = pid_i_mem_yaw_omega = 0.0f;
	pid_last_roll_d_error_omega = pid_last_pitch_d_error_omega = pid_last_yaw_d_error_omega = 0.0f;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
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
