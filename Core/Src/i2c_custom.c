/*
 * TU Berlin - Control Systems - AviGA3
 *
 * I2C_CUSTOM:
 * Handle I2C communication between Mainboard and Sensorboard
 */

/*********************************************************************/
/*Includes*/
#include <i2C_custom.h>
#include <meas.h>

/*********************************************************************/
/*Private variables*/


/*********************************************************************/
/*External*/

//SYNC: Timer overflow (restart timer every 10 s)
extern uint16_t sync_timer_overflow;
extern TIM_HandleTypeDef htim16;

//I2C
extern I2C_HandleTypeDef hi2c1;
extern uint8_t i2c_buffer[];

//IMU: Device and FIFO-Frame
extern uint8_t fifo_data[];
extern struct bmi2_dev bmi2_dev;
extern struct bmi2_fifo_frame fifoframe;

//MEAS: Sensors
extern imu_sensor sensor;
extern uint8_t meas_state_flag;

/*********************************************************************/
/*Callbacks*/

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	//Wake up system after stop mode
	if(sensor.transfer_mode == MODE_BLE)
	{
		HAL_ResumeTick();
	}

	if(i2c_buffer[0] == I2C_SETTINGS)
	{
		i2c_setSettings();
	}

	if(i2c_buffer[0] == I2C_MODE)
	{
		i2c_setMode();
	}

	if((i2c_buffer[0] == I2C_MEAS_START))
	{
		//Start Measurement (Activate IMU and FIFO-Interrupt)
		meas_state_flag = ON;

		//I2C: Listen for new commands
		HAL_I2C_Slave_Receive_IT(&hi2c1, i2c_buffer, I2C_HEADER_SIZE);
	}

	if(i2c_buffer[0] == I2C_MEAS_STOP)
	{
		//Stop measurement
		meas_state_flag = ON;

		//I2C: Listen for new commands
		HAL_I2C_Slave_Receive_IT(&hi2c1, i2c_buffer, I2C_HEADER_SIZE);
	}

	if(i2c_buffer[0] == I2C_DATA_REQUEST)
	{

		//SYNC: Get current timestamp from mainboard
		TIM16->CNT 				= (uint32_t)(i2c_buffer[5] | (i2c_buffer[6] << 8));
		sync_timer_overflow 	= (uint16_t)(i2c_buffer[7] | (i2c_buffer[8] << 8));

		i2c_sendDataHeader();

		i2c_sendData();
	}
}

/*********************************************************************/
/*Functions*/

//INTERRUPT: Request communication
void i2c_requestCommunication()
{
	HAL_GPIO_WritePin(I2C_INT_GPIO_Port, I2C_INT_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(I2C_INT_GPIO_Port, I2C_INT_Pin, GPIO_PIN_RESET);
}

//Fill buffer with data header information
uint8_t i2c_sendDataHeader()
{
	//Wait for I2C-Interface
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);

	//Configure DATA_HEADER (at the end to keep copied measurement data safe)
	i2c_buffer[0] = I2C_DATA_HEADER;
	i2c_buffer[1] = sensor.settings.odr;
	i2c_buffer[2] = sensor.settings.acc_range;
	i2c_buffer[3] = sensor.settings.gyr_range;
	i2c_buffer[4] = sensor.accel_frame_length;
	i2c_buffer[5] = (sensor.time_arrival & 0xff); 						//Time_arrival_LSB
	i2c_buffer[6] = ((sensor.time_arrival >> 8) & 0xff); 				//Time_arrival_MSB
	i2c_buffer[7] = (sensor.time_arrival_overflow & 0xff); 				//Time_arrival__overflowLSB
	i2c_buffer[8] = ((sensor.time_arrival_overflow >> 8) & 0xff); 		//Time_arrival_overflow_MSB

	//Transmit Data-Header
	HAL_I2C_Slave_Transmit(&hi2c1, i2c_buffer, I2C_HEADER_SIZE, 100);

	return 0;
}

//SendFIFO-Data
uint8_t i2c_sendData()
{
	//Wait for I2C-Interface
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);

	//Copy ACCEL-Data in I2C-Buffer
	for(uint16_t i = 0; i < BMI2_FIFO_ACCEL_FRAME_COUNT; i++)
	{
		i2c_buffer[(i*6)+0] 	= ((sensor.accel[i].x) & 0xff);
		i2c_buffer[(i*6)+1] 	= ((sensor.accel[i].x >> 8) & 0xff);

		i2c_buffer[(i*6)+2] 	= ((sensor.accel[i].y) & 0xff);
		i2c_buffer[(i*6)+3] 	= ((sensor.accel[i].y >> 8) & 0xff);

		i2c_buffer[(i*6)+4] 	= ((sensor.accel[i].z) & 0xff);
		i2c_buffer[(i*6)+5] 	= ((sensor.accel[i].z >> 8) & 0xff);
	}

	//Copy GYRO-Data in I2C-Buffer
	for(uint16_t i = 0; i < BMI2_FIFO_GYRO_FRAME_COUNT; i++)
	{
		i2c_buffer[(i*6) + 0 + BMI2_FIFO_ACCEL_FRAME_COUNT*6] 	= ((sensor.gyro[i].x) & 0xff);
		i2c_buffer[(i*6) + 1 + BMI2_FIFO_ACCEL_FRAME_COUNT*6] 	= ((sensor.gyro[i].x >> 8) & 0xff);

		i2c_buffer[(i*6) + 2 + BMI2_FIFO_ACCEL_FRAME_COUNT*6] 	= ((sensor.gyro[i].y) & 0xff);
		i2c_buffer[(i*6) + 3 + BMI2_FIFO_ACCEL_FRAME_COUNT*6] 	= ((sensor.gyro[i].y >> 8) & 0xff);

		i2c_buffer[(i*6) + 4 + BMI2_FIFO_ACCEL_FRAME_COUNT*6] 	= ((sensor.gyro[i].z) & 0xff);
		i2c_buffer[(i*6) + 5 + BMI2_FIFO_ACCEL_FRAME_COUNT*6] 	= ((sensor.gyro[i].z >> 8) & 0xff);
	}

	//Transmit Data
	HAL_I2C_Slave_Transmit(&hi2c1, i2c_buffer, sensor.accel_frame_length*12, 100);

	//Wait for I2C-Interface
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);

	//I2C: Listen for new commands
	HAL_I2C_Slave_Receive_IT(&hi2c1, i2c_buffer, I2C_HEADER_SIZE);

	return 0;
}

//Store new settings for IMU
uint8_t i2c_setSettings()
{
	//Wait for I2C-Interface
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);

	sensor.settings.odr 		= i2c_buffer[1];
	sensor.settings.acc_range 	= i2c_buffer[2];
	sensor.settings.gyr_range 	= i2c_buffer[3];

	//I2C: Listen for new commands
	HAL_I2C_Slave_Receive_IT(&hi2c1, i2c_buffer, I2C_HEADER_SIZE);

	return 0;
}

//Get new transfer mode (BLE/I2C) and configure system
uint8_t i2c_setMode()
{
	//Mode I2C
	if(i2c_buffer[1] == MODE_I2C)
	{
		sensor.transfer_mode = MODE_I2C;

		//LED: Indicator
		setLED(GREEN, OFF);
		blinkLED(BLUE, ON, 1000, 1);
	}

	if(i2c_buffer[1] == MODE_BLE)
	{
		sensor.transfer_mode = MODE_BLE;

		//LED: Indicator
		setLED(GREEN, OFF);
		blinkLED(BLUE, ON, 1000, 2);
	}


	//Wait for I2C-Interface
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);

	//I2C: Listen for new commands
	HAL_I2C_Slave_Receive_IT(&hi2c1, i2c_buffer, I2C_HEADER_SIZE);

	return 0;
}
