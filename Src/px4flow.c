#include "px4flow.h"
#include "IMU.h"
#include "mpu9255.h"
/*
	5V
	PB11 - SDA
	PB10 - SCL
	GND
*/





struct px4_i2c_integral_frame iframe;
extern I2C_HandleTypeDef hi2c2;
extern float yaw, pitch, roll;

extern float channel_1, channel_2, channel_3;
extern float px_setpoint, py_setpoint;
extern uint8_t hovering_controller ;
uint8_t ground_distance_count = 0, ground_distance_fail_count = 0;
uint8_t px4flow_error ;


uint8_t data_integral[26];
float pixel_x, pixel_y;
uint8_t vel_x_cnt, vel_y_cnt;
float vel_x_avr, vel_y_avr;
float vel_x_buffer[PX4FLOW_VEL_SIZE + 1], vel_y_buffer[PX4FLOW_VEL_SIZE + 1];
double px4flow_timespan;
double yaw_px4flow, yaw_px4flow_offset;
double vel_x_vip, vel_y_vip;
float real_delta_px4flow_vel_x, real_delta_px4flow_vel_y;
float real_delta_sonar, delta_sonar_buffer[DELTA_SONAR_SIZE + 1];
float real_distance_buffer[DELTA_SONAR_DISTANCE + 1];
uint8_t delta_px4flow_vel_cnt_x, delta_px4flow_vel_cnt_y;
float delta_px4flow_vel_x_buffer[DELTA_PX4FLOWVEL_SIZE + 1], delta_px4flow_vel_y_buffer[DELTA_PX4FLOWVEL_SIZE + 1];
float pre_velocity_x, pre_velocity_y;
double px, py;
double px_bf, py_bf, vel_x_original_frame, vel_y_original_frame;
float pid_px, pid_py, lpf_pid_px_out, lpf_pid_py_out;
uint8_t slow_ctrler_cnt = 0;
float pid_vel_filter_hz = 17.0f, d_filter_hz = 15.0f;
float vx_sp, vy_sp;
float velpid_x_i, velpid_y_i;
float pid_d_buffer_px[PX4FLOW_PID_D_BUFFER_SIZE + 1], pid_d_buffer_py[PX4FLOW_PID_D_BUFFER_SIZE + 1];
uint8_t px4flow_pid_d_cnt_y, px4flow_pid_d_cnt_x;
float px4flow_pid_avr_dx, px4flow_pid_avr_dy;
float velpid_x_p, Dx, velpid_y_p, Dy;
  // static float Dx_lpf, Dy_lpf;
float temp_pid_rx, temp_pid_ry;
  //	  static float pre_vel_x_error, pre_vel_y_error;
float vel_x_error, vel_y_error;
float p_pos_max = 6.579f, i_pos_max = 4.53f, d_pos_max = 3.719f, poshold_pid_max = 13.5f;
float velocity_sp_max = 0.85f;
float kp_hovering = 0.20588f, ki_hovering = 0.05f, kd_hovering = 0.02735f;
float angle_compensate_roll, angle_compensate_pitch;
float _kp_pos_of = 0.00055f;
float velocity_sp_lpf_hz = 12.0f;
extern float px_setpoint, py_setpoint, vx_setpoint, vy_setpoint;

void readRegisterPX4(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
	uint8_t * tmp = &subAddress;
	HAL_I2C_Master_Transmit(&hi2c2, PX4_DEVICE_ADDRESS<<1, tmp, 1, 10); 
	HAL_I2C_Master_Receive(&hi2c2, PX4_DEVICE_ADDRESS<<1, dest, count, 10);
}

uint8_t PX4Flow_update_integral(void)
{
	readRegisterPX4(0x16, 26, data_integral);
	//HAL_I2C_Mem_Read(&hi2c2, PX4_DEVICE_ADDRESS<<1, 0x16, I2C_MEMADD_SIZE_8BIT, data_integral, 26, 10);

	// read the data
  iframe.frame_count_since_last_readout = data_integral[0] + (data_integral[1] << 8);
  iframe.pixel_flow_x_integral  = data_integral[2] + (data_integral[3] << 8);
  iframe.pixel_flow_y_integral  = data_integral[4] + (data_integral[5] << 8);
  iframe.gyro_x_rate_integral   = data_integral[6] + (data_integral[7] << 8);
  iframe.gyro_y_rate_integral   = data_integral[8] + (data_integral[9] << 8);
  iframe.gyro_z_rate_integral   = data_integral[10] + (data_integral[11] << 8);
  iframe.integration_timespan   = data_integral[12] + (data_integral[13] << 8) + (data_integral[14] << 16) + (data_integral[15]<<24);
  iframe.sonar_timestamp        = data_integral[16] + (data_integral[17] << 8) + (data_integral[18] << 16) + (data_integral[19]<<24);
  iframe.ground_distance    		= data_integral[20] + (data_integral[21]<<8);
  iframe.gyro_temperature       = data_integral[22] + (data_integral[23]<<8) ;
  iframe.quality              	= data_integral[24];
  return 1;
}

int gyro_x_rate, gyro_y_rate, gyro_z_rate, flow_x, flow_y, timespan, ground_distance, quality;
float ground_distance_cm, pre_ground_distance_cm = 0.0f;
float velocity_x, velocity_y;
uint32_t last_px4flow_timer, px4flow_read_timer, PX4read_timer;
float real_delta_sonar, delta_sonar_buffer[DELTA_SONAR_SIZE + 1];
uint8_t delta_sonar_cnt = 0, real_distance_counter = 0;
float flowScaler_x = 0.0f, flowScaler_y = 0.0f;
float flowScaleFactorX, flowScaleFactorY;
float pid_poshold_max;
float w_cos_yaw, w_sin_yaw;
float delta_x = 0.0f, delta_y = 0.0f;

void PX4Flow_Init(void){
    flowScaleFactorX = 1.0f + 0.001f * flowScaler_x;
    flowScaleFactorY = 1.0f + 0.001f * flowScaler_y;
    pid_poshold_max = sin(poshold_pid_max * DEG_TO_RAD);
    _kp_pos_of = 0.00055f;
    slow_ctrler_cnt = 0;
		kp_hovering = 0.08f;///0.1f; //0.20588
		ki_hovering = 0.03f ;//0.05f;
		kd_hovering = 0.105f;//0.08f ;//0.02735f;
		px = py = 0.0f;
}

/*
	ALTITUDE CONTROLLER VARIABLES
*/
float altitude_setpoint = 0.0f;
float altitude_error = 0.0f, altitude_error_pre = 0.0f, throttle_output = 0.0f, throttle_output_max = 275.0f;
//float kp_altitude = 0.5f, ki_altitude = 0.5f, kd_altitude = 10.0f;
float kp_altitude = 0.75f, ki_altitude = 0.07f, kd_altitude = 5.5f;
float pid_altitude_integral, pid_altitude_d = 0.0f, pre_pid_altitude_d = 0.0f, pid_altitude_integral_max = 95.0f;
float distance = 0.0f, distance_rep = 0.0f, distance_real = 0.0f, pre_distance = 0.0f, delta_us015_buffer[6],distance_buffer[20];


void PID_Controller_Altitude(uint16_t channel_3, float distance)
{
	if (channel_3 > THROTTLE_P)
	{
		altitude_setpoint = 65.0f;
	}
	else 
		altitude_setpoint = distance;

		altitude_error = altitude_setpoint - distance;
		
		pid_altitude_integral += ki_altitude * altitude_error;	
		if (pid_altitude_integral > pid_altitude_integral_max) pid_altitude_integral = pid_altitude_integral_max;
		else if (pid_altitude_integral < pid_altitude_integral_max * (-1)) pid_altitude_integral = pid_altitude_integral_max * (-1);	
		pid_altitude_d = kd_altitude * (altitude_error - altitude_error_pre);
		pid_altitude_d = LPF(pid_altitude_d, pre_pid_altitude_d, 50, 0.112f);//anpha=0.972
		pid_altitude_d = Math_fConstrain(pid_altitude_d, -45.0f, 45.0f);
		pre_pid_altitude_d = pid_altitude_d;
	
		throttle_output = kp_altitude * altitude_error + pid_altitude_integral + pid_altitude_d ;
		
		if (throttle_output > 125.0f)  throttle_output = 125.0f;
		else if (throttle_output < -125.0f)  throttle_output = -125.0f;
		
		altitude_error_pre = altitude_error;
		
}

float real_ground_distance, ground_distance_filtered, delta_sonar;
uint32_t distance_error_counter = 0, count_error = 0;

void PX4Flow_get_data(void){
  gyro_x_rate = iframe.gyro_x_rate_integral / 10.0f; // mrad
  gyro_y_rate = iframe.gyro_y_rate_integral / 10.0f; // mrad
  gyro_z_rate = iframe.gyro_z_rate_integral / 10.0f;
  flow_x = iframe.pixel_flow_x_integral * flowScaleFactorX / 10.0f; // mrad
  flow_y = iframe.pixel_flow_y_integral * flowScaleFactorY / 10.0f; // mrad  
  timespan = iframe.integration_timespan; // microseconds
  px4flow_timespan = timespan * 0.000001f;
  ground_distance = iframe.ground_distance; // mm
  ground_distance_cm = ground_distance*0.1f;
  quality = iframe.quality;

	if (ground_distance_cm == 0) {
		distance_error_counter++;
	}
	//---Distance ground process-------
	ground_distance_count++;
	if (ground_distance_count >= 7)
	{
		ground_distance_count = 0;
		if (fabs(ground_distance_cm - pre_ground_distance_cm) >= 20.0f) {
      ground_distance_fail_count++;
      if (ground_distance_fail_count < 5) {
        ground_distance_cm = pre_ground_distance_cm;
      } else {
        ground_distance_fail_count = 0;
      }
    } else {
      ground_distance_fail_count = 0;
    } 
    real_delta_sonar = average_filter(delta_sonar_buffer, DELTA_SONAR_SIZE, pre_ground_distance_cm - ground_distance_cm, & delta_sonar_cnt);
    pre_ground_distance_cm = ground_distance_cm;
		distance_real = ground_distance_cm* cos(roll * M_PI/180.0f) * cos(pitch * M_PI/180.0f);
		if( distance_real < 30) distance_real = 30.0f;
		PID_Controller_Altitude(channel_3, distance_real);
	}
		
//  real_delta_sonar = average_filter(delta_sonar_buffer, DELTA_SONAR_SIZE, pre_ground_distance_cm - ground_distance_cm, &delta_sonar_cnt);
	
//	ground_distance_filtered = LPF(ground_distance_cm, pre_ground_distance_cm, 50.0f, 0.048f);
  //ground_distance_filtered = average_filter(real_distance_buffer, DELTA_SONAR_DISTANCE, ground_distance_cm, &real_distance_counter);
//	ground_distance_filtered = ground_distance_filtered*0.5f + ground_distance_cm*0.5f;
	
	
  if (quality > 150) {
    // Update flow rate with gyro rate
    pixel_x = flow_x - gyro_x_rate; // mrad
    pixel_y = flow_y - gyro_y_rate; // mrad
    // Scale based on ground distance and compute speed
    // (flow/1000) * (ground_distance/1000) / (timespan/1000000)
    // velocity in body frame
    velocity_x = pixel_x * ground_distance_cm * 10.0f / timespan; // m/s
    velocity_y = pixel_y * ground_distance_cm * 10.0f / timespan; // m/s 
    vel_x_avr = average_filter(vel_x_buffer, PX4FLOW_VEL_SIZE, velocity_x, & vel_x_cnt);
    vel_y_avr = average_filter(vel_y_buffer, PX4FLOW_VEL_SIZE, velocity_y, & vel_y_cnt);
    yaw_px4flow = calculate_relative_yaw(yaw, yaw_px4flow_offset);
		
//		yaw_px4flow = 0.0f;
    
		double cos_yaw, sin_yaw;
    cos_yaw = cos(yaw_px4flow * DEG_TO_RAD);
    sin_yaw = sin(yaw_px4flow * DEG_TO_RAD);
		w_cos_yaw = cos_yaw;
		w_sin_yaw = sin_yaw;
    // choose velocity for calculate px,py and pid position
    vel_x_vip = vel_x_avr;
    vel_y_vip = vel_y_avr;
    real_delta_px4flow_vel_x = average_filter(delta_px4flow_vel_x_buffer, DELTA_PX4FLOWVEL_SIZE, pre_velocity_x - vel_x_vip, & delta_px4flow_vel_cnt_x);
    real_delta_px4flow_vel_y = average_filter(delta_px4flow_vel_y_buffer, DELTA_PX4FLOWVEL_SIZE, pre_velocity_y - vel_y_vip, & delta_px4flow_vel_cnt_y);
    pre_velocity_x = vel_x_vip;
    pre_velocity_y = vel_y_vip;
    //convert velocity_x,y in current frame to original frame.
    vel_x_original_frame = (-vel_x_vip) * cos_yaw + vel_y_vip * sin_yaw;
    vel_y_original_frame = -(-vel_x_vip) * sin_yaw + vel_y_vip * cos_yaw;
    vel_x_original_frame = -vel_x_original_frame;
    // Integrate velocity to get pose estimate. dt in mili second

    px = px + vel_x_original_frame * (float)timespan * 0.001f; // mili m 
    py = py + vel_y_original_frame * (float)timespan * 0.001f;
    // convert px,py in original frame to body frame
    px_bf = (-px) * cos_yaw + py * sin_yaw;
    py_bf = -(-px) * sin_yaw + py * cos_yaw;
    px_bf = -px_bf;
  }
<<<<<<< HEAD
	if (quality < 150)
	{
		count_error++;
	}
	else
	{
		count_error = 0;
	}
	if (count_error == 100)
	{
		px4flow_error = 1;
	}
	
=======
#if POSITION_CONTROL	
>>>>>>> 48bab97780b20d97282318c2d937fa7923ad4fcd
  if (slow_ctrler_cnt == (POS_CONTROLLER_T - 1)){
    slow_ctrler_cnt = 0;
//		PX4Flow_get_sp_vel(&vx_sp, &vy_sp, px, py, px_setpoint, py_setpoint);
		PX4Flow_get_sp_vel(&vx_sp, &vy_sp, px_bf, py_bf, px_setpoint, py_setpoint);
  }
  slow_ctrler_cnt++;
	
  px4flow_position_pid(vx_sp, vy_sp, vel_x_vip, vel_y_vip);
#else
  px4flow_position_pid(vx_setpoint, vy_setpoint, vel_x_vip, vel_y_vip);
	
#endif
  /*
  Convert angle to rc
  */
  pid_px = angle_compensate_roll;
  lpf_pid_px_out = LPF(pid_px, lpf_pid_px_out, pid_vel_filter_hz, px4flow_timespan);
  pid_py = angle_compensate_pitch;
  lpf_pid_py_out = LPF(pid_py, lpf_pid_py_out, pid_vel_filter_hz, px4flow_timespan);

}

void px4flow_position_pid(float vx_setpoint, float vy_setpoint, float vx_input, float vy_input) {

  vel_x_error = (vx_setpoint - vx_input);
  vel_y_error = (vy_setpoint - vy_input);

  velpid_x_p = vel_x_error * kp_hovering;
  velpid_x_p = Math_fConstrain(velpid_x_p, -sin(p_pos_max * DEG_TO_RAD), sin(p_pos_max * DEG_TO_RAD));
  velpid_x_i += vel_x_error * ki_hovering * px4flow_timespan;
  velpid_x_i = Math_fConstrain(velpid_x_i, -sin(i_pos_max * DEG_TO_RAD), sin(i_pos_max * DEG_TO_RAD));
  //		Dx = (vel_x_error - pre_vel_x_error)*velocity.d/px4flow_timespan;
  //	  Dx = Math_fConstrain(Dx, -sin(d_pos_max*DEG_TO_RAD), sin(d_pos_max*DEG_TO_RAD));
  Dx = real_delta_px4flow_vel_x * kd_hovering / px4flow_timespan;
//	Dx_lpf = LPF(Dx, Dx_lpf, d_filter_hz, px4flow_timespan);
  px4flow_pid_avr_dx = average_filter(pid_d_buffer_px, PX4FLOW_PID_D_BUFFER_SIZE, Dx, & px4flow_pid_d_cnt_x);
  //	temp_pid_rx =  velpid_x_p + velpid_x_i + Dx;
  //	  temp_pid_rx =  velpid_x_p + velpid_x_i + Dx_lpf;
  temp_pid_rx = velpid_x_p + velpid_x_i + px4flow_pid_avr_dx;
  //		pre_vel_x_error = vel_x_error;

  velpid_y_p = vel_y_error * kp_hovering;
  velpid_y_p = Math_fConstrain(velpid_y_p, -sin(p_pos_max * DEG_TO_RAD), sin(p_pos_max * DEG_TO_RAD));
  velpid_y_i += vel_y_error * ki_hovering * px4flow_timespan;
  velpid_y_i = Math_fConstrain(velpid_y_i, -sin(i_pos_max * DEG_TO_RAD), sin(i_pos_max * DEG_TO_RAD));
  //		Dy = (vel_y_error - pre_vel_y_error) * velocity_param.d/px4flow_timespan;;
  //		Dy = Math_fConstrain(Dy, -sin(d_pos_max*DEG_TO_RAD), sin(d_pos_max*DEG_TO_RAD));
  Dy = real_delta_px4flow_vel_y * kd_hovering / px4flow_timespan;
  //	Dy_lpf = LPF(Dy, Dy_lpf, d_filter_hz, px4flow_timespan);	
  px4flow_pid_avr_dy = average_filter(pid_d_buffer_py, PX4FLOW_PID_D_BUFFER_SIZE, Dy, & px4flow_pid_d_cnt_y);
  //		temp_pid_ry = velpid_y_p + velpid_y_i + Dy_lpf ;
  // sin(pitch)_setpoint
  temp_pid_ry = velpid_y_p + velpid_y_i + px4flow_pid_avr_dy;
  //		pre_vel_y_error = vel_y_error;
  // RC roll pitch compesate value.
  // this value must be limit at least [-1;1] to be from -90;90 
  /*
  asin radian unit
  */
  temp_pid_rx = Math_fConstrain(temp_pid_rx, -pid_poshold_max, pid_poshold_max);
  temp_pid_ry = Math_fConstrain(temp_pid_ry, -pid_poshold_max, pid_poshold_max);
  // safe asin and convert to degree
  // roll
  /*
	Based on following paper:
  Design and Implementation of a Mini Quadrotor Control System in GPS Denied Environments		
  */
  /* compensate for T_total
  hover_throttle
  Motors[0]
  it shouldn't have hover_throttle, only total_thurst, because I have always neglect total_thrust ->
  so I have to implement it this way in order to not mess the code up.
  */
//   if (flight_mode == 3) {
//     //thrust_average = (Motors[0]+Motors[1]+Motors[2]+Motors[3])/4.0f;
//     thrust_average = (float) Motor_throttle;
//     velocity_compensate_factor = (float) hover_throttle / thrust_average;
//   } else {
//     velocity_compensate_factor = 1.0f;
//   }
//   // u_vx*m/T_total
//   temp_pid_ry = temp_pid_ry * velocity_compensate_factor;
  // compesate complete!!
  float temp_pid_rx_2;
  temp_pid_rx_2 = temp_pid_rx  * invSqrt(1.0f - temp_pid_ry * temp_pid_ry);
  //	temp_pid_rx_2 = temp_pid_rx_2*;
  angle_compensate_roll = -safe_asin(temp_pid_rx_2) * RAD_TO_DEG;
  angle_compensate_pitch = safe_asin(temp_pid_ry) * RAD_TO_DEG;
	
//   pidvelocity_x_rc.pout = px4flow_pid_to_rc(velpid_x_p);
//   pidvelocity_x_rc.dout = px4flow_pid_to_rc(px4flow_pid_avr_dx);
//   pidvelocity_x_rc.iout = px4flow_pid_to_rc(velpid_x_i);
//   pidvelocity_y_rc.pout = px4flow_pid_to_rc(velpid_y_p);
//   pidvelocity_y_rc.dout = px4flow_pid_to_rc(px4flow_pid_avr_dy);
//   pidvelocity_y_rc.iout = px4flow_pid_to_rc(velpid_y_i);
}


float average_filter(float buffer[], uint8_t buffer_size, float sample, uint8_t *count) {
	buffer[buffer_size]-= buffer[*count];
	buffer[*count] = sample;
	buffer[buffer_size]+= buffer[*count];
	*count = (*count+1)%buffer_size;
	return buffer[buffer_size]/(float)buffer_size;
}

float calculate_relative_yaw(float yaw_absolute,float yaw_offset) {
    float yaw_temp;
    if (yaw_offset >= 0.0f){
				if (yaw_absolute < (yaw_offset - 180) && yaw_absolute > -180){
					yaw_temp = 360 - (- yaw_absolute + yaw_offset);
				}
				else {
					yaw_temp = yaw_absolute - yaw_offset;
				}
		}
// offset < 0.0f
     else {
			if (yaw_absolute < 180 && yaw_absolute > (180 + yaw_offset)){
					yaw_temp = - yaw_offset + yaw_absolute - 360;
				}
				else {
					yaw_temp = yaw_absolute - yaw_offset;
				}
    }
    return yaw_temp;
}

float Math_fConstrain(float value, float min, float max)
{
	if(value > max)value = max;
	else if(value < min)value = min;
	return value;
}

float safe_asin(float v)
{
	if (isnan(v))
	{
		return 0.0f;
	}
	if (v >= 1.0f)
	{
		return M_PI / 2;
	}
	if (v <= -1.0f)
	{
		return -M_PI / 2;
	}
	return asin(v);
}
uint32_t test_counter = 0;

void PX4Flow_get_angle_setpoint(float* roll_sp, float* pitch_sp){
	test_counter++;
	PX4Flow_update_integral();
	PX4Flow_get_data();
	*roll_sp = lpf_pid_px_out;
	*pitch_sp = lpf_pid_py_out;
}
float w_px_sp, w_py_sp;

void PX4Flow_get_sp_vel(float * v_out_x, float * v_out_y, float px_pf, float py_pf, float px_sp, float py_sp){
	float vx_sp_out, vy_sp_out;
	
	vx_sp_out = 0.0f;
	vy_sp_out = 0.0f;
	
//	if (channel_1 > 1493) vx_sp_out = -((channel_1) - 1493)*1.0/507;
//	else if (channel_1 <1477) vx_sp_out =  (1477 - (channel_1))*1.0/477;
//					else vx_sp_out = 0.0f;
//	
//	if (channel_2 > 1528) vy_sp_out =  ((channel_2) - 1528)*1.0/472;
//	else if (channel_2 <1512) vy_sp_out =  -(1512 - (channel_2))*1.0/512;
//					else vy_sp_out = 0.0f;

	
	if (channel_1 > 1500) px_setpoint = px_setpoint - ((channel_1) - 1500)*100/500;
	else if (channel_1 <1470) px_setpoint = px_setpoint + (1470 - (channel_1))*100/470;
	
	if (channel_2 > 1535) py_setpoint = py_setpoint + ((channel_2) - 1535)*100/465;
	else if (channel_2 <1505) py_setpoint = py_setpoint - (1505 - (channel_2))*100/505;

	
//	vx_sp_out = _kp_pos_of * (0.0f - px_pf);
//	vy_sp_out = _kp_pos_of * (0.0f - py_pf);

//	w_px_sp = px_sp;
//	w_py_sp = py_sp;
	vx_sp_out = _kp_pos_of * (px_setpoint - px_pf);
	vy_sp_out = _kp_pos_of * (py_setpoint - py_pf);
	yaw_px4flow_offset = yaw;
	
	vx_sp_out = LPF(vx_sp_out, *v_out_x, velocity_sp_lpf_hz, POS_CONTROLLER_T*px4flow_timespan);
  vy_sp_out = LPF(vy_sp_out, *v_out_y, velocity_sp_lpf_hz, POS_CONTROLLER_T*px4flow_timespan);
	
  *v_out_x = Math_fConstrain(vx_sp_out, -velocity_sp_max, velocity_sp_max);
  *v_out_y = Math_fConstrain(vy_sp_out, -velocity_sp_max, velocity_sp_max);	
  yaw_px4flow_offset = yaw;	
}

float PX4Flow_get_distance(void){
//	if (ground_distance_filtered == 30.0f) return 0.0f;
//	else
//		return ground_distance_filtered;
	return ground_distance_filtered;
}


