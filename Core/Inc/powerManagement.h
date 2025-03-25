/*
 * TU Berlin - Control Systems - AviGA3
 *
 * Power Management:
 * General power management functions including battery charging, status,..
 * - Check battery voltage
 * - Check charging state
 */




#ifndef POWERMANAGEMENT_H
#define POWERMANAGEMENT_H

/*********************************************************************/
/*Includes*/

#include <common.h>
#include <powerManagement.h>

/*********************************************************************/
/*Defines*/
#define CHG_Pin GPIO_PIN_2
#define CHG_GPIO_Port GPIOA

/*********************************************************************/
/*Functions*/

uint8_t powerManagement_getBatteryLevel();
uint8_t powerManagement_getChargerState();


#endif /* POWERMANAGEMENT_H */
