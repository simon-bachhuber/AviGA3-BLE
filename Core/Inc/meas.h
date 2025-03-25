/*
 * TU Berlin - Control Systems - AviGA3
 *
 * The "Integrated Measurement Unit" BMI270 is used in this project.
 * The communication is using SPI.
 * In bmi270.c, the initialization, setting,... for the IMU are performed.
 */

/*********************************************************************/

#ifndef MEAS_H
#define MEAS_H

/*********************************************************************/
/*Includes*/

//BMI270
#include <bmi270_custom.h>

//I2C
#include <i2c_custom.h>

//BLE
#include "ble_custom.h"

/*********************************************************************/
/*Defines*/

//STATES:
#define MODE_I2C             	UINT8_C(0x0)
#define MODE_BLE             	UINT8_C(0x1)

/*       Structs              */
/******************************************************************************/

//Editable settings for IMU-Sensor
typedef struct
{
	//Output Data Rate
	uint8_t odr;

	//Accelerometer range
	uint8_t acc_range;

	//Gyroscope range
	uint8_t gyr_range;
}imu_settings;

typedef struct
{
	uint8_t sensor_nr;

	uint8_t active;

	uint8_t transfer_mode;

	imu_settings settings;

	uint8_t i2c_address;

	uint8_t sd_flag;

	uint16_t time_arrival;
	uint16_t time_arrival_overflow;

	uint16_t accel_frame_length;

	struct bmi2_sens_axes_data accel[BMI2_FIFO_ACCEL_FRAME_COUNT];
	struct bmi2_sens_axes_data gyro[BMI2_FIFO_GYRO_FRAME_COUNT];

} imu_sensor;

extern imu_sensor sensor;

/*********************************************************************/
/*Functions*/
void meas_Handler();

imu_sensor meas_initSensor(imu_sensor sensor,uint8_t sensor_nr);

void meas_startMeasurement(struct bmi2_dev *bmi2_dev,  struct bmi2_fifo_frame *fifoframe);
void meas_stopMeasurement(struct bmi2_dev *bmi2_dev,  struct bmi2_fifo_frame *fifoframe);

void meas_fifoWTM_intRoutine();

#endif /* MEAS_H */
