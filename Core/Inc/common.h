/*
 * TU Berlin - Control Systems - AviGA3
 *
 * Common:
 * Common defines and functions to use basic interfaces of the system.
 * - Status Defines
 * - LED-Control
 * - Button-Control
 */

#ifndef COMMON_H
#define COMMON_H


/*********************************************************************/
/*Includes*/

#include "main.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

/*********************************************************************/
/*Defines*/

//STATES:
#define ON             		UINT8_C(0x1)
#define OFF             	UINT8_C(0x0)

//BUFFER: Measurement-Buffer-Size (Size of measurement-data buffers for ACC & GYR. Filled with FIFO data)
#define measurement_buffer_size	4096

//LED:
#define BLUE             	UINT8_C(0x1)
#define GREEN             	UINT8_C(0x2)

/*********************************************************************/
/*Functions*/

//LED: Set LED states
void setLED(uint8_t led, uint8_t state);
void blinkLED(uint8_t led, uint8_t state, uint16_t ms, int8_t count);
void blinkLED_intRoutine();

void common_Handler();

//BUTTON: Get Button state
uint8_t getButton();


#endif /* COMMON_H */



