/**
  ******************************************************************************
  * @file    IMU.c
  * @author  Waveshare Team
  * @version V1.0
  * @date    29-August-2014
  * @brief   This file provides all the IMU firmware functions.

  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, WAVESHARE SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  ******************************************************************************
  */
#include "IMU.h"
#include "mpu9255.h"

#define LPF_cutoff_freq_angles	1

float roll, pitch, yaw;
float roll_raw, pitch_raw, yaw_raw;
float roll_pre, pitch_pre, yaw_pre;

volatile float q0=1, q1, q2, q3; 
float GYRO_FACTOR = 131.072f;

/**
  * @brief  invSqrt
  * @param  
  * @retval 
  */

float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	
	long i = *(long*)&y;                //get bits for floating value
	i = 0x5f3759df - (i >> 1);          //gives initial guss you
	y = *(float*)&i;                    //convert bits back to float
	y = y * (1.5f - (halfx * y * y));   //newtop step, repeating increases accuracy
	
	return y;
}


/**
  * @brief  initializes IMU
  * @param  None
  * @retval None
  */
void IMU_Init(void)
{	 	
  	q0 = 1.0f;  
  	q1 = 0.0f;
  	q2 = 0.0f;
  	q3 = 0.0f;
}

#define Kp 4.50f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 1.0f    // integral gain governs rate of convergence of gyroscope biases

/**
  * @brief  Updata attitude and heading 
  * @param  ax: accelerometer X
  * @param  ay: accelerometer Y
  * @param  az: accelerometer Z
  * @param  gx: gyroscopes X
  * @param  gy: gyroscopes Y
  * @param  gz: gyroscopes Z
  * @param  mx: magnetometer X
  * @param  my: magnetometer Y
  * @param  mz: magnetometer Z
  * @retval None
  */
void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
	float ex, ey, ez, halfT = 0.024f;

	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;   
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;          

	norm = invSqrt(ax * ax + ay * ay + az * az);       
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	norm = invSqrt(mx * mx + my * my + mz * mz);          
	mx = mx * norm;
	my = my * norm;
	mz = mz * norm;

	// compute reference direction of flux
	hx = 2.0f * mx * (0.5f - q2q2 - q3q3) + 2.0f * my * (q1q2 - q0q3) + 2.0f * mz * (q1q3 + q0q2);
	hy = 2.0f * mx * (q1q2 + q0q3) + 2.0f * my * (0.5f - q1q1 - q3q3) + 2.0f * mz * (q2q3 - q0q1);
	hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);         
	bx = sqrt((hx * hx) + (hy * hy));
	bz = hz;     

	// estimated direction of gravity and flux (v and w)
	vx = 2.0f * (q1q3 - q0q2);
	vy = 2.0f * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
	wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
	wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);  

	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		exInt = exInt + ex * Ki * halfT;
		eyInt = eyInt + ey * Ki * halfT;	
		ezInt = ezInt + ez * Ki * halfT;

		gx = gx + Kp * ex + exInt;
		gy = gy + Kp * ey + eyInt;
		gz = gz + Kp * ez + ezInt;
	}

	q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
	q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
	q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
	q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;  

	norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
}


/**
  * @brief  Get quaters
  * @param  None
  * @retval None
  */
void IMU_GetQuater(void)
{
	float MotionVal[9];
		MotionVal[0]=gyro[0]/GYRO_FACTOR;
    MotionVal[1]=gyro[1]/GYRO_FACTOR;
    MotionVal[2]=gyro[2]/GYRO_FACTOR;
    MotionVal[3]=accel[0];
    MotionVal[4]=accel[1];
    MotionVal[5]=accel[2];
    MotionVal[6]=mag[0];
    MotionVal[7]=mag[1];
    MotionVal[8]=mag[2];
    
 	IMU_AHRSupdate((float)MotionVal[0] * 0.0175f, (float)MotionVal[1] * 0.0175f, (float)MotionVal[2] * 0.0175f,
   	(float)MotionVal[3], (float)MotionVal[4], (float)MotionVal[5], (float)MotionVal[6], (float)MotionVal[7], MotionVal[8]);
}


/**
  * @brief  Get Yaw Pitch Roll
  * @param  None
  * @retval None
  */
void IMU_GetYawPitchRoll(void)
{
  IMU_GetQuater(); 
//	pitch_raw = asin(-2.0f * q1 * q3 + 2.0f * q0* q2)* 57.3f; // pitch
//	roll_raw = atan2(2.0f * q2 * q3 + 2.0f * q0 * q1, -2.0f * q1 * q1 - 2.0f * q2* q2 + 1.0f) * 57.3f; // roll
//	yaw_raw = atan2(-2.0f * q1 * q2 - 2.0f * q0 * q3, 2.0f * q2 * q2 + 2.0f * q3 * q3 - 1.0f) * 57.3f; // yaw

//	pitch = LPF(pitch_raw, pitch_pre, LPF_cutoff_freq_angles, 0.002f);
//	roll  = LPF(roll_raw, roll_pre, LPF_cutoff_freq_angles, 0.002f);
//	yaw		= LPF(yaw_raw, yaw_pre, LPF_cutoff_freq_angles, 0.002f);
	
	pitch = asin(-2.0f * q1 * q3 + 2.0f * q0* q2)* 57.3f; // pitch
	roll = atan2(2.0f * q2 * q3 + 2.0f * q0 * q1, -2.0f * q1 * q1 - 2.0f * q2* q2 + 1.0f) * 57.3f; // roll
	yaw = atan2(-2.0f * q1 * q2 - 2.0f * q0 * q3, 2.0f * q2 * q2 + 2.0f * q3 * q3 - 1.0f) * 57.3f; // yaw
	
//	pitch_pre = pitch;
//	roll_pre 	= roll;
//	yaw_pre 	= yaw;

}

/******************* (C) COPYRIGHT 2014 Waveshare *****END OF FILE*******************/

