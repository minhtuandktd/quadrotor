#ifndef _mpu9255_h_
#define _mpu9255_h_

#include "main.h"
#include "stm32f4xx_hal.h"

// Define MPU9255 register address
//****************************************
#define 	MPU6050_DEVICE_ADRESS				0xD0 // 0x68<<1
#define 	AK8963_DEVICE_ADRESS				0x18 // 0x0C<<1

#define 	THROTTLE_P													1465

#define	PWR_MGMT_1		0x6B	//Power Management. Typical values:0x00(run mode)
#define	SMPLRT_DIV		0x19	//Sample Rate Divider. Typical values:0x07(125Hz) 1KHz internal sample rate
#define	CONFIG				0x1A	//Low Pass Filter.Typical values:0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//Gyro Full Scale Select. Typical values:0x10(1000dps)
#define	ACCEL_CONFIG	0x1C	//Accel Full Scale Select. Typical values:0x01(2g)
#define INT_PIN_CFG   0x37  
#define AK8963_CNTL1          0x0A 
#define AK8963_CNTL2          0x0B 
#define USER_CTRL     0x6A
#define I2C_MST_CTRL  0x24
#define PWR_MGMNT_2   0x6C
#define ACCEL_CONFIG2 0x1D
#define AK8963_WHO_AM_I 0x00
#define I2C_SLV0_ADDR   0x25
#define AK8963_I2C_ADDR 0x0C
#define I2C_SLV0_REG    0x26
#define I2C_SLV0_DO     0x63
#define I2C_SLV0_CTRL   0x27
#define I2C_SLV0_EN     0x80
#define I2C_READ_FLAG   0x80
#define AK8963_PWR_DOWN 0x00
#define AK8963_RESET    0x01
#define EXT_SENS_DATA_00 0x49
#define AK8963_FUSE_ROM  0x0F
#define AK8963_CNT_MEAS2 0x16
#define CLOCK_SEL_PLL  0x01
#define PWR_RESET     0x80

#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
 //gyroscope offset
#define    XG_OFFSET_H       0x13
#define    XG_OFFSET_L       0x14
#define    YG_OFFSET_H       0x15
#define    YG_OFFSET_L       0x16
#define    ZG_OFFSET_H       0x17
#define    ZG_OFFSET_L       0x18
		
#define MAG_XOUT_L		0x03
#define MAG_XOUT_H		0x04
#define MAG_YOUT_L		0x05
#define MAG_YOUT_H		0x06
#define MAG_ZOUT_L		0x07
#define MAG_ZOUT_H		0x08
#define    ASAX       0x10
#define    ASAY       0x11
#define    ASAZ       0x12


#define		WHO_AM_I		  	0x75	//Identity of the device

#define 	WHO_AM_I_VAL		0x73 //Identity of MPU6050 is 0x68
#define 	WHO_AM_I_VAL_MAG		0x48





void MPU9255_Init(void);
void InitGyrOffset(void);
void MPU9255_Get_Data(void);
void Format_Data_Accel_Gyro_Temp_Mag (void);
void Average_Filter(int16_t ax,int16_t ay,int16_t az,int16_t mx,int16_t my, int16_t mz,int16_t gx,int16_t gy, int16_t gz);
void Average_Filter_2(int16_t ax,int16_t ay,int16_t az,int16_t mx,int16_t my, int16_t mz);
void writeRegisterMPU9250(uint8_t subAddress, uint8_t data);
void writeAK8963Register(uint8_t subAddress, uint8_t data);
void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest);
void readRegisterMPU9250(uint8_t subAddress, uint8_t count, uint8_t* dest);
void writeRegisterMPU9250(uint8_t subAddress, uint8_t data);
float LPF(float sample, float pre_value, float cut_off, float dt);

#endif
