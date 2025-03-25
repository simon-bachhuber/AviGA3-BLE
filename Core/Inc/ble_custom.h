/*
 * TU Berlin - Control Systems - AviGA3
 *
 * BLE_CUSTOM:
 * Handle BLE communication between PC and Sensorboard
 */


/*********************************************************************/
#ifndef BLE_CUSTOM_H
#define BLE_CUSTOM_H

/*********************************************************************/
/*Includes*/
#include <common.h>
#include <meas.h>

/*********************************************************************/
/*Defines*/

//BUFFER SIZE
#define BLE_BUFF_SIZE       	UINT8_C(153) //BLE Max capability: 247 bytes

//HEADER SIZE
#define BLE_HEADER_SIZE       	UINT8_C(9) //I2C_HEADER_SIZE

//CHARACTERISTIC IDENTIFIERS
#define BLE_CHAR_DATA       	UINT8_C(0)
#define BLE_CHAR_BATTERY       	UINT8_C(1)

//STATES:

//REQUEST CODES:


/*********************************************************************/
/*Functions*/

void ble_sendData();


#endif /* BLE_CUSTOM_H */
