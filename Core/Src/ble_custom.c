/*
 * TU Berlin - Control Systems - AviGA3
 *
 * BLE_CUSTOM:
 * Handle BLE communication between PC and Sensorboard
 */

/*********************************************************************/
/*Includes*/
#include <ble_custom.h>


/*********************************************************************/
/*Private variables*/
//Conversion buffer
uint8_t sensor_data_raw[2048] = {0};


/*********************************************************************/
/*External*/

extern imu_sensor sensor;

extern uint8_t ble_flag;
extern uint8_t ble_buffer[];

//DEBUG
extern int16_t acc_x;

/*********************************************************************/
/*Callbacks*/


/*********************************************************************/
/*FLAG Handler*/


/*********************************************************************/
/*Functions*/

void ble_sendData()
{

	//Copy ACCEL-Data in DATA-Buffer
	for(uint16_t i = 0; i < BMI2_FIFO_ACCEL_FRAME_COUNT; i++)
	{
		sensor_data_raw[(i*6)+0] 	= ((sensor.accel[i].x) & 0xff);
		sensor_data_raw[(i*6)+1] 	= ((sensor.accel[i].x >> 8) & 0xff);

		sensor_data_raw[(i*6)+2] 	= ((sensor.accel[i].y) & 0xff);
		sensor_data_raw[(i*6)+3] 	= ((sensor.accel[i].y >> 8) & 0xff);

		sensor_data_raw[(i*6)+4] 	= ((sensor.accel[i].z) & 0xff);
		sensor_data_raw[(i*6)+5] 	= ((sensor.accel[i].z >> 8) & 0xff);
	}

	//Copy GYRO-Data in DATA-Buffer
	for(uint16_t i = 0; i < BMI2_FIFO_GYRO_FRAME_COUNT; i++)
	{
		sensor_data_raw[(i*6) + 0 + BMI2_FIFO_ACCEL_FRAME_COUNT*6] 	= ((sensor.gyro[i].x) & 0xff);
		sensor_data_raw[(i*6) + 1 + BMI2_FIFO_ACCEL_FRAME_COUNT*6] 	= ((sensor.gyro[i].x >> 8) & 0xff);

		sensor_data_raw[(i*6) + 2 + BMI2_FIFO_ACCEL_FRAME_COUNT*6] 	= ((sensor.gyro[i].y) & 0xff);
		sensor_data_raw[(i*6) + 3 + BMI2_FIFO_ACCEL_FRAME_COUNT*6] 	= ((sensor.gyro[i].y >> 8) & 0xff);

		sensor_data_raw[(i*6) + 4 + BMI2_FIFO_ACCEL_FRAME_COUNT*6] 	= ((sensor.gyro[i].z) & 0xff);
		sensor_data_raw[(i*6) + 5 + BMI2_FIFO_ACCEL_FRAME_COUNT*6] 	= ((sensor.gyro[i].z >> 8) & 0xff);
	}

	//Set header
	ble_buffer[0] = 0;													//Frame-Number
	ble_buffer[1] = sensor.settings.odr;								//Output data rate
	ble_buffer[2] = sensor.settings.acc_range;							//ACCEL sensitivity
	ble_buffer[3] = sensor.settings.gyr_range;							//GYRO sensitivity
	ble_buffer[4] = sensor.accel_frame_length;							//Amount of transfered data sets per axis (less than 170 because of FIFO-watermark level)
	ble_buffer[5] = (sensor.time_arrival & 0xff); 						//Time_arrival_LSB
	ble_buffer[6] = ((sensor.time_arrival >> 8) & 0xff); 				//Time_arrival_MSB
	ble_buffer[7] = (sensor.time_arrival_overflow & 0xff); 				//Time_arrival__overflowLSB
	ble_buffer[8] = ((sensor.time_arrival_overflow >> 8) & 0xff); 		//Time_arrival_overflow_MSB

	//	//ACC: All frames
	for(uint8_t j = 0; j <= 7; j++)
	{
		//Copy DATA buffer to BLE-Buffer (burst write) (144 bytes/frame = 6 bytes/set * 24 sets/frame )
		for(uint8_t i = 0; i < 144; i++)
		{
			ble_buffer[i+9] = sensor_data_raw[(i) + (j * 144)];
		}

		//Frame Number
		ble_buffer[0] = j;

		//Wait for transmission
		HAL_Delay(50);

		//Send buffer
		ble_flag = ON;
		ble_updateChar(BLE_CHAR_DATA);
	}


	//	//GYR: All frames
	for(uint8_t j = 8; j <= 15; j++)
	{
		//Copy DATA buffer to BLE-Buffer (burst write) (144 bytes/frame = 6 bytes/set * 24 sets/frame )
		for(uint8_t i = 0; i < 144; i++)
		{
			ble_buffer[i+9] = sensor_data_raw[(i) + (j * 144) - 132];	//Offset for new Frame: -132
		}

		//Frame Number
		ble_buffer[0] = j;

		//Wait for transmission
		HAL_Delay(50);

		//Send buffer
		ble_flag = ON;
		ble_updateChar(BLE_CHAR_DATA);
	}

}
