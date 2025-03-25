/*
 * TU Berlin - Control Systems - AviGA3
 *
 * Power Management:
 * General power management functions including battery charging, status,..
 * - Check battery voltage
 * - Check charging state
 */


/*********************************************************************/
/*Includes*/

//Math
#include <math.h>

//Power Management
#include <powerManagement.h>

//Measurement
#include <meas.h>

/*********************************************************************/
/*Private variables*/


/*********************************************************************/
/*Extern*/

//MEASUREMENT
extern imu_sensor sensor;

//LED
extern uint8_t led_blink_active[];

//ADC
extern ADC_HandleTypeDef hadc1;

//TIMER1
extern TIM_HandleTypeDef htim1;

/*********************************************************************/
/*Functions*/

uint8_t powerManagement_getBatteryLevel()
{
	//Battery Voltage
	float battery_voltage = 0;

	//Start ADC
	HAL_ADC_Start(&hadc1);

	//Get current battery voltage
	HAL_ADC_PollForConversion(&hadc1, 100);
	battery_voltage = HAL_ADC_GetValue(&hadc1);

	//Calculate voltage (3V3 * 2 = 6V6)
	battery_voltage = (battery_voltage/4096)*6.6;

	//Get difference to empty battery (empty = 3V5)
	battery_voltage = battery_voltage - 3.5;

	//calculate percentage (Umax = 4V2: 4V2 - 3V5 = 0V7)
	battery_voltage = round((battery_voltage/0.7) * 100);

	//Check if battery is higher than 100% or lower than 0%
	if(battery_voltage >= 100)
	{
		battery_voltage = 100;
	}

	if(battery_voltage <= 0)
	{
		battery_voltage = 0;
	}

	//Stop ADC
	HAL_ADC_Stop(&hadc1);

	return (uint8_t)battery_voltage;
}

uint8_t powerManagement_getChargerState()
{
	//Charger state
	uint8_t charger_state = HAL_GPIO_ReadPin(CHG_GPIO_Port, CHG_Pin);;

	//Check State (0 == Charging; 1 == Not Charging)
	if(charger_state == OFF)
	{
		//Check if a measurement is active (no blinking for energy saving)
		if(sensor.active == OFF && led_blink_active[0] == 0)
		{
			//Indication LED Charging(BLINKING)
			blinkLED(GREEN,ON, 1000,-1);
		}
	}

	else if(charger_state == ON)
	{
		//Check if a measurement is active (no blinking for energy saving)
		if(sensor.active == OFF && led_blink_active[0] == 0)
		{
			//Indication LED Full (ON)
			blinkLED(GREEN,OFF, 0,0);
			setLED(GREEN,ON);
		}
	}


	return charger_state;
}
