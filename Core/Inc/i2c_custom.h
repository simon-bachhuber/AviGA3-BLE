/*
 * TU Berlin - Control Systems - AviGA3
 *
 * I2C_CUSTOM:
 * Handle I2C communication between Mainboard and Sensorboard
 */


/*********************************************************************/
#ifndef I2C_CUSTOM_H
#define I2C_CUSTOM_H

/*********************************************************************/
/*Includes*/
#include <common.h>
#include <bmi270_custom.h>
#include <meas.h>
/*********************************************************************/
/*Defines*/

//BUFFER SIZE
#define I2C_BUFF_SIZE       	UINT8_C(2048) //Max 2048 bytes of IMU-FIFO-BUFFER

//HEADER SIZE
#define I2C_HEADER_SIZE       	UINT8_C(9) //I2C_HEADER_SIZE

//STATES
#define I2C_DATA_NOT_READY     	UINT8_C(0)
#define I2C_DATA_READY       	UINT8_C(1)

//REQUEST CODES:
#define I2C_SETTINGS        	UINT8_C(0)
#define I2C_MODE        		UINT8_C(1)
#define I2C_MEAS_START        	UINT8_C(2)
#define I2C_MEAS_STOP       	UINT8_C(3)
#define I2C_DATA_REQUEST    	UINT8_C(4)
#define I2C_DATA_HEADER        	UINT8_C(5)

/*********************************************************************/
/*Functions*/

void i2c_requestCommunication();

uint8_t i2c_sendDataHeader();
uint8_t i2c_sendData();

uint8_t i2c_setSettings();
uint8_t i2c_setMode();

#endif /* I2C_CUSTOM_H */
