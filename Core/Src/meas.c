/*
 * TU Berlin - Control Systems - AviGA3
 *
 * Measurement:
 * In measurement.c/.h, all functions to start / stop a measurement routine are performed.
 */

/*********************************************************************/
/*Includes*/

//COMMON
#include <common.h>

//MEAS
#include <meas.h>

#include  <bmi270_custom.h>

#include  <bmi2_defs.h>

/*********************************************************************/
/*Private variables*/

//Measurement Interrupt flag
uint8_t meas_state_flag = OFF;
uint8_t meas_data_flag	= OFF;

//Sensor
imu_sensor sensor;

//IMU: FLAGS & MARKERS
uint16_t int_status = 0;
uint16_t watermark = 0;
uint16_t fifo_length;
uint16_t accel_frame_length;
uint16_t gyro_frame_length;

/*********************************************************************/
/*External*/

//SYNC: Timer overflow (restart timer every 10 s)
uint16_t sync_timer_overflow = 0;
extern TIM_HandleTypeDef htim16;

//I2C
extern I2C_HandleTypeDef hi2c1;
extern uint8_t i2c_buffer[] ; // DATA to receive

//SYNC: Timer overflow (restart timer every 10 s)
extern uint16_t sync_timer_overflow;
extern TIM_HandleTypeDef htim16;

//IMU: Device and FIFO-Frame
extern uint8_t fifo_data[];
extern struct bmi2_dev bmi2_dev;
extern struct bmi2_fifo_frame fifoframe;


/*********************************************************************/
/*Functions*/

//Measurement-Interrupt Handler
void meas_Handler()
{
	//Check for START/STOP-Interrupt
	if(meas_state_flag == ON)
	{
		//Set measurement ON
		if(sensor.active == OFF)
		{
			//LED: Deactivate all LED indications (for energy saving)
			blinkLED(GREEN, OFF,0, 0);
			setLED(GREEN, OFF);

			//Indicate data transfer mode
			if(sensor.transfer_mode == MODE_I2C)
			{
				//LED: Blink blue LED x 1
				blinkLED(BLUE, ON, 500, 1);
			}

			if(sensor.transfer_mode == MODE_BLE)
			{
				//LED: Blink blue LED x 2
				blinkLED(BLUE, ON, 500, 2);
			}

			//Start Measurement (Activate IMU and FIFO-Interrupt)
			meas_startMeasurement(&bmi2_dev, &fifoframe);
		}

		//Set measurement OFF
		else
		{
			//LED: Blink blue LED
			blinkLED(BLUE, ON, 1500, 1);

			//Stop measurement
			meas_stopMeasurement(&bmi2_dev, &fifoframe);
		}

		//RESET Flag
		meas_state_flag = OFF;
	}

	//Check for new DATA Interrupt
	if(meas_data_flag == ON)
	{
		meas_fifoWTM_intRoutine();

		//Reset Flag
		meas_data_flag = OFF;
	}
}

/*********************************************************************/
//Initialize connected Sensorboards
imu_sensor meas_initSensor(imu_sensor sensor,uint8_t sensor_nr)
{
	sensor.sensor_nr = sensor_nr;

	//MEASUREMENT: OFF
	sensor.active = OFF;

	//TRANFER MODE: I2C (standard)
	sensor.transfer_mode = MODE_I2C;

	//SETTINGS: 100 HZ, 2G, 250 dps
	sensor.settings.odr = BMI2_GYR_ODR_100HZ;
	sensor.settings.acc_range = BMI2_ACC_RANGE_2G;
	sensor.settings.gyr_range = BMI2_GYR_RANGE_250;

	//SYNC Times
	sensor.time_arrival 			= 0;
	sensor.time_arrival_overflow	= 0;

	//SD: FLAG
	sensor.sd_flag = OFF;

	//I2C: FLAG & ADDRESS
	sensor.i2c_address 	= sensor_nr<<1;

	return sensor;
}

/*********************************************************************/

//Start new measurement (Activate BMI20 and FIFO Interrupts)
void meas_startMeasurement(struct bmi2_dev *bmi2_dev,  struct bmi2_fifo_frame *fifoframe)
{
	//Status-Byte
	uint8_t rslt;

	/* Accel and gyro sensor are listed in array. */
	uint8_t sensor_sel[2] = { BMI2_ACCEL, BMI2_GYRO };

	/* Configuration settings for accel and gyro. */
	rslt = set_accel_gyro_config(bmi2_dev, sensor.settings.odr, sensor.settings.acc_range, sensor.settings.gyr_range);

	/* Accel and Gyro enable must be done after setting configurations */
	rslt = bmi270_sensor_enable(sensor_sel, 2, bmi2_dev);

	/* Before setting FIFO, disable the advance power save mode. */
	rslt = bmi2_set_adv_power_save(BMI2_DISABLE, bmi2_dev);

	/* Initially disable all configurations in fifo. */
	rslt = bmi2_set_fifo_config(BMI2_FIFO_ALL_EN, BMI2_DISABLE, bmi2_dev);

	/* Set FIFO configuration by enabling accel, gyro.*/
	rslt = bmi2_set_fifo_config(BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN, BMI2_ENABLE, bmi2_dev);

	/* To enable headerless mode, disable the header. */
	if (rslt == BMI2_OK)
	{
		rslt = bmi2_set_fifo_config(BMI2_FIFO_HEADER_EN, BMI2_DISABLE, bmi2_dev);
	}

	/* FIFO water-mark interrupt is enabled. */
	fifoframe->data_int_map = BMI2_FWM_INT;

	/* Map water-mark interrupt to the required interrupt pin. */
	rslt = bmi2_map_data_int(fifoframe->data_int_map, BMI2_INT1, bmi2_dev);

	/* Set water-mark level. */
	fifoframe->wm_lvl = BMI2_FIFO_WATERMARK_LEVEL;
	fifoframe->length = BMI2_FIFO_RAW_DATA_USER_LENGTH;

	/* Set the water-mark level if water-mark interrupt is mapped. */
	rslt = bmi2_set_fifo_wm(fifoframe->wm_lvl, bmi2_dev);

	//Configure INT1 for FIFO_WATERMARK_INT as Output
	uint8_t data[1];
	data[0] = 0b00001010;
	bmi270_writeRegister(BMI2_INT1_IO_CTRL_ADDR, data , 1);

	//Set measurement ON
	sensor.active = ON;

	//Start Sync Timer
	HAL_TIM_Base_Start_IT(&htim16);
}

//MEAS: Stop measurement
void meas_stopMeasurement(struct bmi2_dev *bmi2_dev,  struct bmi2_fifo_frame *fifoframe)
{
	/* Accel and gyro sensor are listed in array. */
	uint8_t sensor_sel[2] = {BMI2_ACCEL, BMI2_GYRO};

	//Disable ACC & GYR
	bmi270_sensor_disable(sensor_sel, 2, bmi2_dev);

	//Disable everything in FIFO config
	bmi2_set_fifo_config(BMI2_FIFO_ALL_EN, BMI2_DISABLE, bmi2_dev);

	//Set measurement OFF
	sensor.active = OFF;

	//Stop Sync Timer
	HAL_TIM_Base_Stop_IT(&htim16);
	TIM16->CNT = 0;
	sync_timer_overflow = 0;
}

//MEAS: FIFO Watermark Interrupt routine
void meas_fifoWTM_intRoutine()
{
	/* Read FIFO data on interrupt. */
	uint8_t rslt;
	rslt = bmi2_get_int_status(&int_status, &bmi2_dev);

	/* To check the status of FIFO watermark interrupt. */
	if ((rslt == BMI2_OK) && (int_status & BMI2_FWM_INT_STATUS_MASK))
	{
		rslt = bmi2_get_fifo_wm(&watermark, &bmi2_dev);
		rslt = bmi2_get_fifo_length(&fifo_length, &bmi2_dev);

		accel_frame_length = BMI2_FIFO_ACCEL_FRAME_COUNT;
		gyro_frame_length = BMI2_FIFO_GYRO_FRAME_COUNT;

		/* Updating FIFO length to be read based on availa%ble length and dummy byte updation */
		fifoframe.length = fifo_length + bmi2_dev.dummy_byte;

		/* Read FIFO data. */
		rslt = bmi2_read_fifo_data(&fifoframe, &bmi2_dev);

		if (rslt == BMI2_OK)
		{
			/* Parse the FIFO data to extract accelerometer data from the FIFO buffer. */
			rslt = bmi2_extract_accel(sensor.accel, &accel_frame_length, &fifoframe, &bmi2_dev);

			/* Parse the FIFO data to extract gyro data from the FIFO buffer. */
			rslt = bmi2_extract_gyro(sensor.gyro, &gyro_frame_length, &fifoframe, &bmi2_dev);

			//Set ACCEL/GYRO Frame length
			sensor.accel_frame_length = accel_frame_length;

			//SYNC: Get Sensor arrival time
			sensor.time_arrival = TIM16->CNT;
			sensor.time_arrival_overflow = sync_timer_overflow;

			//Data transfer to main system (I2C / BLE)
			if(sensor.transfer_mode == MODE_I2C)
			{
				//I2C: Send data using i2c;
				i2c_requestCommunication();
			}

			if(sensor.transfer_mode == MODE_BLE)
			{
				ble_sendData();
			}
		}
	}
}

