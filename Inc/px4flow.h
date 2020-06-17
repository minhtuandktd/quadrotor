#ifndef __PX4FLOW_H__
#define __PX4FLOW_H__

#include "main.h"
#include "stm32f4xx_hal.h"

#define DEG_TO_RAD M_PI/180.0f
#define RAD_TO_DEG 180.0f/M_PI

#define PX4_DEVICE_ADDRESS  0x42
#define DELTA_SONAR_SIZE 5
#define DELTA_SONAR_DISTANCE 5
#define PX4FLOW_VEL_SIZE 13
#define DELTA_PX4FLOWVEL_SIZE 5
#define PX4FLOW_PID_D_BUFFER_SIZE 10
#define POS_CONTROLLER_T 5

struct px4_i2c_integral_frame
{
    uint16_t frame_count_since_last_readout;//number of flow measurements since last I2C readout [#frames]
    int16_t pixel_flow_x_integral;//accumulated flow in radians*10000 around x axis since last I2C readout [rad*10000]
    int16_t pixel_flow_y_integral;//accumulated flow in radians*10000 around y axis since last I2C readout [rad*10000]
    int16_t gyro_x_rate_integral;//accumulated gyro x rates in radians*10000 since last I2C readout [rad*10000] 
    int16_t gyro_y_rate_integral;//accumulated gyro y rates in radians*10000 since last I2C readout [rad*10000] 
    int16_t gyro_z_rate_integral;//accumulated gyro z rates in radians*10000 since last I2C readout [rad*10000] 
    uint32_t integration_timespan;//accumulation timespan in microseconds since last I2C readout [microseconds]
    uint32_t sonar_timestamp;// time since last sonar update [microseconds]
    int16_t ground_distance;// Ground distance in meters*1000 [meters*1000]
    int16_t gyro_temperature;// Temperature * 100 in centi-degrees Celsius [degcelsius*100]
    uint8_t quality;// averaged quality of accumulated flow values [0:bad quality;255: max quality]
};

void readRegisterPX4(uint8_t subAddress, uint8_t count, uint8_t* dest);
uint8_t PX4Flow_update_integral(void);
void PX4Flow_get_data(void);
void PX4Flow_Init(void);
float average_filter(float buffer[], uint8_t buffer_size, float sample, uint8_t *count);
float LPF(float sample, float pre_value, float cut_off, float dt);
void px4flow_position_pid(float vx_setpoint, float vy_setpoint, float vx_input, float vy_input);
float calculate_relative_yaw(float yaw_absolute,float yaw_offset);
float Math_fConstrain(float value, float min, float max);
float invSqrt(float x);
float safe_asin(float v);
void PX4Flow_get_angle_setpoint(float* roll_sp, float* pitch_sp);
void PX4Flow_get_sp_vel(float * v_out_x, float * v_out_y, float px_pf, float py_pf, float px_sp, float py_sp);
float PX4Flow_get_distance(void);
#endif
