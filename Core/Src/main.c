/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************cccc************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp280.h"
#include "HMC5883L.h"
#include "stdbool.h"
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0xD0

#define DLPF_CFG 0x1A
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */


BMP280_HandleTypedef bmp280;
float pressure, temperature, humidity, altitude_sea, altitude, altitude_cal;
int32_t pressure_rotating_mem[50], pressure_total_avarage;
uint8_t pressure_rotating_mem_location;
float pressure_rotating_mem_actual;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Ax, Ay, Az, Gx, Gy, Gz;


bool set_gyro_angle = true;

volatile long ch[8];
volatile long tick;
volatile uint8_t pulse;

int receiver_input_channel_1;
int receiver_input_channel_2;
int receiver_input_channel_3;
int receiver_input_channel_4;
int receiver_input_channel_5;
int receiver_input_channel_6;

int throttle;

long gyro_x_cal;
long gyro_y_cal;
long gyro_z_cal;

float angle_pitch_acc_cal;
float angle_roll_acc_cal;

int loop_timer;

long gyro_x_man = 278;
long gyro_y_man = -2898;
long gyro_z_man = 352;

long gyro_x;
long gyro_y;
long gyro_z;


long acc_x;
long acc_y;
long acc_z;
long acc_total_vector;

float angle_roll;
float angle_pitch;

//float angle_roll_output;
//float angle_pitch_output;

float angle_roll_acc;
float angle_pitch_acc;

float roll_level_adjust;
float pitch_level_adjust;

float gyro_roll_input;
float gyro_pitch_input;
float gyro_yaw_input;

float pid_roll_setpoint;
float pid_pitch_setpoint;
float pid_yaw_setpoint;


float pid_p_gain_roll		= 0.4;			//0.4
float pid_i_gain_roll		= 0.0015;			//0.0015
float pid_d_gain_roll		= 0.7;			//0.7

float pid_p_gain_pitch		= 0.4;			//0.4
float pid_i_gain_pitch		= 0.0015;			//0.0015
float pid_d_gain_pitch		= 0.7;			//0.7

float pid_p_gain_yaw		= 2;
float pid_i_gain_yaw		= 0.02;
float pid_d_gain_yaw		= 0;

float pid_i_mem_roll;
float pid_i_mem_pitch;
float pid_i_mem_yaw;

float pid_last_roll_d_error;
float pid_last_pitch_d_eroor;
float pid_last_yaw_d_error;

float pid_error_temp;

float pid_roll_output;
float pid_pitch_output;
float pid_yaw_output;

int pid_max_roll			= 400;
int pid_max_pitch			= 400;
int pid_max_yaw				= 400;

int esc_1 = 1000;
int esc_2= 1000;
int esc_3= 1000;
int esc_4= 1000;

int start;

int min_throthle = 1070;
int max_throthle = 2000;
int disable_motor = 1000;


float turning_speed = 3.0;
bool auto_level = true;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */

void MPU6050_Init (void)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x03;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, DLPF_CFG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 8g
		Data = 0x10;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 500 �/s
		Data = 0x08;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}

}

void gyro_get_data(){

	uint8_t Accel_Val_Raw[6];
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Accel_Val_Raw, 6, 1000);

	acc_y = (int16_t) (Accel_Val_Raw[0] << 8 | Accel_Val_Raw [1]);
	acc_x = (int16_t) (Accel_Val_Raw[2] << 8 | Accel_Val_Raw [3]);
	acc_z = (int16_t) (Accel_Val_Raw[4] << 8 | Accel_Val_Raw [5]);

	uint8_t Gyro_Val_Raw[6];
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Gyro_Val_Raw, 6, 1000);

	gyro_y = (int16_t) (Gyro_Val_Raw[0] << 8 | Gyro_Val_Raw [1]);
	gyro_x = (int16_t) (Gyro_Val_Raw[2] << 8 | Gyro_Val_Raw [3]);
	gyro_z = (int16_t) (Gyro_Val_Raw[4] << 8 | Gyro_Val_Raw [5]);

	gyro_x *= -1;
	//gyro_y *= -1;
	gyro_z *= -1;

}

void mpu6050_cal(){

	for( int i = 0; i < 2000; i++){
		if ( i % 15 == 0 ) HAL_GPIO_TogglePin(GPIOC, led_cal_Pin);
		gyro_get_data();

		gyro_x_cal += gyro_x;
		gyro_y_cal += gyro_y;
		gyro_z_cal += gyro_z;


		acc_total_vector = sqrt( ( acc_x*acc_x ) + ( acc_y * acc_y) + ( acc_z * acc_z ) );

		  if ( abs(acc_y) < acc_total_vector ){
			  angle_pitch_acc = asin( (float) acc_y / acc_total_vector ) * 57.296;
		  }

		  if ( abs(acc_x) < acc_total_vector ){
			  angle_roll_acc = asin( (float) acc_x / acc_total_vector ) * 57.296;
		  }

		  angle_pitch_acc_cal += angle_pitch_acc;
		  angle_roll_acc_cal += angle_roll_acc;

		HAL_Delay(4);

	}

	gyro_x_cal /= 2000;
	gyro_y_cal /= 2000;
	gyro_z_cal /= 2000;

	angle_pitch_acc_cal /= 2000;
	angle_roll_acc_cal /= 2000;
}

//void read_bmp280(){
//	bmp280_read_float(&bmp280, &temperature, &pressure, &humidity);
//	//altitude_sea=145442.2f*(1-powf(10,log(pressure/101325.0f)*(1.0f/5.255f)));
//}
//
//void do_altitude_cal(){
//	for( int i = 0; i < 2000; i++){
//			if ( i % 15 == 0 ) HAL_GPIO_TogglePin(GPIOC, led_cal_Pin);
//			read_bmp280();
//			altitude_cal += altitude;
//	}
//	altitude_cal /= 2000;
//}

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  MX_TIM11_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  HAL_TIM_Base_Start(&htim10);
  HAL_TIM_Base_Start(&htim11);


  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
  HAL_Delay(100);

  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,0);
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,0);
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,0);
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,0);

  HAL_Delay(100);

  HMC5883L_setRange (HMC5883L_RANGE_1_3GA);
  HMC5883L_setMeasurementMode (HMC5883L_CONTINOUS);
  HMC5883L_setDataRate (HMC5883L_DATARATE_15HZ);
  HMC5883L_setSamples (HMC5883L_SAMPLES_1);
  HMC5883L_setOffset (0, 0);

  bmp280_init_default_params(&bmp280.params);
  bmp280.addr = BMP280_I2C_ADDRESS_0;
  bmp280.i2c = &hi2c1;
  bmp280_init(&bmp280, &bmp280.params);

  MPU6050_Init();
  HAL_Delay(1000);
  mpu6050_cal();
  HAL_Delay(1000);
//  do_altitude_cal();

  loop_timer = __HAL_TIM_GET_COUNTER(&htim10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	HMC5883L_readNormalize ();
//
//	read_bmp280();
//	//altitude = abs(altitude - altitude_cal);
//    pressure_total_avarage -= pressure_rotating_mem[pressure_rotating_mem_location];                          //Subtract the current memory position to make room for the new value.
//    pressure_rotating_mem[pressure_rotating_mem_location] = pressure;                                                //Calculate the new change between the actual pressure and the previous measurement.
//    pressure_total_avarage += pressure_rotating_mem[pressure_rotating_mem_location];                          //Add the new value to the long term avarage value.
//    pressure_rotating_mem_location++;                                                                         //Increase the rotating memory location.
//    if (pressure_rotating_mem_location == 20)pressure_rotating_mem_location = 0;                              //Start at 0 when the memory location 20 is reached.
//    actual_pressure_fast = (float)pressure_total_avarage / 20.0;                                              //Calculate the average pressure of the last 20 pressure readings.
//
//    //To get better results we will use a complementary fillter that can be adjusted by the fast average.
//    actual_pressure_slow = actual_pressure_slow * (float)0.985 + actual_pressure_fast * (float)0.015;
//    actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;                                       //Calculate the difference between the fast and the slow avarage value.
//    if (actual_pressure_diff > 8)actual_pressure_diff = 8;                                                    //If the difference is larger then 8 limit the difference to 8.
//    if (actual_pressure_diff < -8)actual_pressure_diff = -8;                                                  //If the difference is smaller then -8 limit the difference to -8.
//    //If the difference is larger then 1 or smaller then -1 the slow average is adjuste based on the error between the fast and slow average.
//    if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0;
//    //actual_pressure = actual_pressure_slow;


	  receiver_input_channel_1 = ch[0];
	  receiver_input_channel_2 = ch[1];
	  receiver_input_channel_3 = ch[2];
	  receiver_input_channel_4 = ch[3];
	  receiver_input_channel_5 = ch[4];
	  receiver_input_channel_6 = ch[5];

	  gyro_get_data();

	  gyro_x -= gyro_x_cal;
	  gyro_y -= gyro_y_cal;
	  gyro_z -= gyro_z_cal;

//	  acc_x -= acc_x_cal;
//	  acc_y -= acc_y_cal;
//	  acc_z -= acc_z_cal;

	  gyro_pitch_input 	= ( gyro_pitch_input * 0.7 ) + ((float)( gyro_x / 65.5) * 0.3);
	  gyro_roll_input 	= ( gyro_roll_input * 0.7 ) + ((float)( gyro_y / 65.5) * 0.3);
	  gyro_yaw_input 	= ( gyro_yaw_input * 0.7 ) + ((float)( gyro_z / 65.5) * 0.3);

	  angle_pitch += gyro_x * 0.0000611;
	  angle_roll += gyro_y * 0.0000611;


	  angle_pitch -= angle_roll * sin(gyro_z * 0.000001066);
	  angle_roll += angle_pitch * sin(gyro_z * 0.000001066);

	  acc_total_vector = sqrt( ( acc_x*acc_x ) + ( acc_y * acc_y) + ( acc_z * acc_z ) );

	  if ( abs(acc_y) < acc_total_vector ){
		  angle_pitch_acc = asin( (float) acc_y / acc_total_vector ) * 57.296;
		  angle_pitch_acc -= angle_pitch_acc_cal;
	  }

	  if ( abs(acc_x) < acc_total_vector ){
		  angle_roll_acc = asin( (float) acc_x / acc_total_vector ) * 57.296;
		  angle_roll_acc -= angle_roll_acc_cal;
	  }

	  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
	  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;

	  //angle_pitch_acc -= 0.0;		// -1
	  //angle_roll_acc -= 0.0;		// -2.5



//	  if ( set_gyro_angle ) {
//
//	  }
//	  else{
//		  angle_pitch = angle_pitch_acc;
//		  angle_roll = angle_roll_acc;
//		  set_gyro_angle = true;
//	  }


		  pitch_level_adjust = angle_pitch * 15;
		  roll_level_adjust = angle_roll * 15;

//	  if ( !auto_level ){
//		  pitch_level_adjust =0;
//		  roll_level_adjust =0;
//	  }


//	  if ( receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050 ) start =1;

	  if ( receiver_input_channel_6 > 1550 ){
		  start = 2;
		  HAL_GPIO_WritePin(led_arm_GPIO_Port, led_arm_Pin, 1);
		  HAL_GPIO_WritePin(led_disarm_GPIO_Port, led_disarm_Pin, 0);

		  //angle_pitch = angle_pitch_acc;
		  //angle_roll = angle_roll_acc;

		  pid_i_mem_roll = 0;
		  pid_last_roll_d_error = 0;
		  pid_i_mem_pitch = 0;
		  pid_last_pitch_d_eroor = 0;
		  pid_i_mem_yaw = 0;
		  pid_last_yaw_d_error = 0;
	  }

	  if ( start == 2 && receiver_input_channel_6 < 1450){
		  start =0;
		  HAL_GPIO_WritePin(led_arm_GPIO_Port, led_arm_Pin, 0);
		  HAL_GPIO_WritePin(led_disarm_GPIO_Port, led_disarm_Pin, 1);
	  }


	  pid_roll_setpoint =0;
	  if ( receiver_input_channel_1 > 1508 ) {
		  pid_roll_setpoint = (receiver_input_channel_1 - 1508);
	  }
	  else if ( receiver_input_channel_1  < 1492 ){
		  pid_roll_setpoint = ( receiver_input_channel_1  - 1492 );
	  }
	  else{
		  receiver_input_channel_1 = 1500;
	  }

	  pid_roll_setpoint -= roll_level_adjust;
	  pid_roll_setpoint /= turning_speed;

	  pid_pitch_setpoint =0;
	  if ( receiver_input_channel_2 > 1508 ) {
		  pid_pitch_setpoint = ( receiver_input_channel_2 - 1508 );
	  }
	  else if ( receiver_input_channel_2 < 1492 ) {
		  pid_pitch_setpoint = ( receiver_input_channel_2 - 1492 );
	  }
	  else{
	  	receiver_input_channel_2 = 1500;
	  }

	  pid_pitch_setpoint -= pitch_level_adjust;
	  pid_pitch_setpoint /= turning_speed;


	  pid_yaw_setpoint =0;
	  if ( receiver_input_channel_3 > 1050 ){
		  if ( receiver_input_channel_4 > 1508 ) {
			  pid_yaw_setpoint = ( receiver_input_channel_4 - 1508 ) / turning_speed;
		  }
		  else if ( receiver_input_channel_4 < 1492 ){
			  pid_yaw_setpoint = ( receiver_input_channel_4 - 1492 ) / turning_speed;
		  }
	  }
	  else{
	  	  receiver_input_channel_4 = 1500;
	  }

	  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
	  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;

	  if ( pid_i_mem_roll > pid_max_roll ) pid_i_mem_roll = pid_max_roll;
	  else if ( pid_i_mem_roll < pid_max_roll * -1 ) pid_i_mem_roll = pid_max_roll * -1;

	  pid_roll_output = ( pid_p_gain_roll * pid_error_temp ) + pid_i_mem_roll + ( pid_d_gain_roll * ( pid_error_temp - pid_last_roll_d_error));

	  if ( pid_roll_output > pid_max_roll ) pid_roll_output = pid_max_roll;
	  else if ( pid_roll_output < pid_max_roll * -1) pid_roll_output = pid_max_roll * -1;

	  pid_last_roll_d_error = pid_error_temp;


	  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
	  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;

	  if ( pid_i_mem_pitch > pid_max_pitch ) pid_i_mem_pitch = pid_max_pitch;
	  else if ( pid_i_mem_pitch < pid_max_pitch * -1 ) pid_i_mem_pitch = pid_max_pitch * -1;

	  pid_pitch_output = ( pid_p_gain_pitch * pid_error_temp ) + pid_i_mem_pitch + ( pid_d_gain_pitch * ( pid_error_temp - pid_last_pitch_d_eroor));

	  if ( pid_pitch_output > pid_max_pitch ) pid_pitch_output = pid_max_pitch;
	  else if ( pid_pitch_output < pid_max_pitch * -1 ) pid_pitch_output = pid_max_pitch * -1;

	  pid_last_pitch_d_eroor = pid_error_temp;


	  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
	  pid_i_mem_yaw += pid_p_gain_yaw * pid_error_temp;

	  if ( pid_i_mem_yaw > pid_max_yaw ) pid_i_mem_yaw = pid_max_yaw;
	  else if ( pid_i_mem_yaw < pid_max_yaw * -1 ) pid_i_mem_yaw = pid_max_yaw * -1;

	  pid_yaw_output = ( pid_p_gain_yaw * pid_error_temp ) + pid_i_mem_yaw + ( pid_d_gain_yaw * ( pid_error_temp - pid_last_yaw_d_error ));

	  if ( pid_yaw_output > pid_max_yaw ) pid_yaw_output = pid_max_yaw;
	  else if ( pid_yaw_output < pid_max_yaw * -1 ) pid_yaw_output = pid_max_yaw * -1;

	  pid_last_yaw_d_error = pid_error_temp;

	  throttle = receiver_input_channel_3;


	  if ( start == 2 ){
		  if ( throttle > 1800 ) throttle = 1800;

		  esc_1 = throttle - pid_pitch_output + pid_roll_output - pid_yaw_output;
		  esc_2 = throttle + pid_pitch_output + pid_roll_output + pid_yaw_output;
		  esc_3 = throttle + pid_pitch_output - pid_roll_output - pid_yaw_output;
		  esc_4 = throttle - pid_pitch_output - pid_roll_output + pid_yaw_output;

		  if ( esc_1 < min_throthle ) esc_1 = min_throthle;
		  if ( esc_2 < min_throthle ) esc_2 = min_throthle;
		  if ( esc_3 < min_throthle ) esc_3 = min_throthle;
		  if ( esc_4 < min_throthle ) esc_4 = min_throthle;

		  if ( esc_1 > max_throthle ) esc_1 = max_throthle;
		  if ( esc_2 > max_throthle ) esc_2 = max_throthle;
		  if ( esc_3 > max_throthle ) esc_3 = max_throthle;
		  if ( esc_4 > max_throthle ) esc_4 = max_throthle;


	  }else{
		  esc_1 = disable_motor;
		  esc_2 = disable_motor;
		  esc_3 = disable_motor;
		  esc_4 = disable_motor;
	  }

	  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,esc_1);
	  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,esc_2);
	  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,esc_3);
	  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,esc_4);

	//	  if ( __HAL_TIM_GET_COUNTER(&htim13) - loop_timer > 4070 ){
	//		  HAL_GPIO_TogglePin(led_status_GPIO_Port, led_status_Pin);
	//	  }

	  while ( __HAL_TIM_GET_COUNTER(&htim10) - loop_timer < 4000 );
	  loop_timer = __HAL_TIM_GET_COUNTER(&htim10);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 100-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 2000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 100-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 100-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 0xffff;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */


  /* USER CODE END TIM11_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_cal_GPIO_Port, led_cal_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, led_arm_Pin|led_disarm_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : led_cal_Pin */
  GPIO_InitStruct.Pin = led_cal_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_cal_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RC_Pin */
  GPIO_InitStruct.Pin = RC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : led_arm_Pin led_disarm_Pin */
  GPIO_InitStruct.Pin = led_arm_Pin|led_disarm_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if ( GPIO_Pin == RC_Pin){
		tick = __HAL_TIM_GET_COUNTER(&htim11);
		__HAL_TIM_SET_COUNTER(&htim11,0);

		if ( tick < 2100){
			ch[pulse] = tick;
			pulse++;
		}
		else{
			__HAL_TIM_SET_COUNTER(&htim11,0);
			pulse =0;
		}

	}

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
