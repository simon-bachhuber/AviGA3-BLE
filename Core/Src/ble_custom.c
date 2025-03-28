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
    // We're only sending one set of data now, not multiple frames
    
    // Set header
    ble_buffer[0] = 0;                             // Frame-Number
    ble_buffer[1] = sensor.settings.odr;           // Output data rate
    ble_buffer[2] = sensor.settings.acc_range;     // ACCEL sensitivity
    ble_buffer[3] = sensor.settings.gyr_range;     // GYRO sensitivity
    ble_buffer[4] = 1;                             // Single data point
    ble_buffer[5] = (sensor.time_arrival & 0xff);  // Time_arrival_LSB
    ble_buffer[6] = ((sensor.time_arrival >> 8) & 0xff); // Time_arrival_MSB
    ble_buffer[7] = (sensor.time_arrival_overflow & 0xff); // Time_overflow_LSB
    ble_buffer[8] = ((sensor.time_arrival_overflow >> 8) & 0xff); // Time_overflow_MSB
    
    // Pack accelerometer data
    ble_buffer[9]  = (sensor.accel[0].x & 0xff);
    ble_buffer[10] = ((sensor.accel[0].x >> 8) & 0xff);
    ble_buffer[11] = (sensor.accel[0].y & 0xff);
    ble_buffer[12] = ((sensor.accel[0].y >> 8) & 0xff);
    ble_buffer[13] = (sensor.accel[0].z & 0xff);
    ble_buffer[14] = ((sensor.accel[0].z >> 8) & 0xff);
    
    // Pack gyroscope data
    ble_buffer[15] = (sensor.gyro[0].x & 0xff);
    ble_buffer[16] = ((sensor.gyro[0].x >> 8) & 0xff);
    ble_buffer[17] = (sensor.gyro[0].y & 0xff);
    ble_buffer[18] = ((sensor.gyro[0].y >> 8) & 0xff);
    ble_buffer[19] = (sensor.gyro[0].z & 0xff);
    ble_buffer[20] = ((sensor.gyro[0].z >> 8) & 0xff);
    
    // Send the combined data in a single package
    ble_flag = ON;
    ble_updateChar(BLE_CHAR_DATA);
}