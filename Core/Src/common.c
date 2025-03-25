/*
 * TU Berlin - Control Systems - AviGA3
 *
 * Common:
 * Common defines and functions to use basic interfaces of the system.
 * - Status Defines
 * - External Interrupt Handlers
 * - LED-Control
 * - Button-Control
 */

/*********************************************************************/
/*Includes*/
#include <common.h>
#include <meas.h>


/*********************************************************************/
/*Private variables*/

//BUTTON: State of button debounce
uint8_t btn_debounce_state = ON;
uint16_t btn_count = 0;

//LED: States
uint8_t led_state[] 		= {OFF,OFF};
uint8_t led_blink_state[] 	= {OFF,OFF};
uint8_t led_blink_active[] 	= {OFF,OFF};
uint8_t led_blink_count[] 	= {0,0};

/*********************************************************************/
/*External*/
//TIM1: LED blinking
extern TIM_HandleTypeDef htim1;

//TIM2: BUTTON debounce
extern TIM_HandleTypeDef htim2;

//TIM16: Sync timer
extern TIM_HandleTypeDef htim16;

//Power Management: Trigger new state
extern charger_state_trigger;

//MEAS: Measurement Interrupt flag
extern uint8_t meas_state_flag;
extern uint8_t meas_data_flag;

//SYNC: Timer overflow (restart timer every 10 s)
extern uint16_t sync_timer_overflow;

//MEAS: Sensors
extern imu_sensor sensor;

/*********************************************************************/
/*INTERRUPT CALLBACKS*/

//EXTI: External Interrupt events
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//Wake up system after stop mode
	if(sensor.transfer_mode == MODE_BLE)
	{
		HAL_ResumeTick();
	}

	//BUTTON: Input Button
	if(GPIO_Pin == BUTTON_Pin)
	{
		//BUTTON: Debounce routine using TIM2
		if(btn_debounce_state == ON)
		{
			//BUTTON: Start debounce Timer
			HAL_TIM_Base_Start_IT(&htim2);

			//BUTTON: Block new input
			btn_debounce_state = OFF;
		}
	}

	//IMU: FIFO data ready
	if(GPIO_Pin == BMI_INT1_Pin)
	{

		meas_data_flag = ON;
	}
}

//TIM: Timer interrupts
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
	//Wake up system after stop mode
	if(sensor.transfer_mode == MODE_BLE)
	{
		HAL_ResumeTick();
	}

	//Timer 2 interrupt for button debounce
	if (htim->Instance == htim2.Instance)
	{
		//Check if BUTTON is still activated
		if((HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET))
		{
			//Counter up
			btn_count++;
		}

		//Check if Button was released
		if((HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_SET) | (btn_count > 50))
		{
			//Start Measurement: Less than 3 seconds
			if((btn_count >= 1) & (btn_count <= 50))
			{
				//MEAS: Change measurement state
				meas_state_flag = ON;
			}

			//Change Transmit Mode: I2C / BLE
			if(btn_count > 50)
			{

				sensor.transfer_mode =  !sensor.transfer_mode;

				//Mode I2C
				if(sensor.transfer_mode == MODE_I2C)
				{

					//LED: Indicator
					setLED(GREEN, OFF);
					blinkLED(BLUE, ON, 1000, 1);
				}

				if(sensor.transfer_mode == MODE_BLE)
				{
					//LED: Indicator
					setLED(GREEN, OFF);
					blinkLED(BLUE, ON, 1000, 2);
				}
			}

			btn_count = 0;

			//BUTTON: Reset debounce state
			btn_debounce_state = ON;

			//BUTTON: Stop debounce Timer
			HAL_TIM_Base_Stop_IT(&htim2);
		}
	}

	//Timer 16 interrupt for Sync timer overflow
	if (htim->Instance == htim16.Instance)
	{
		//SYNC: Timer overflow (restart timer every 10 s)
		sync_timer_overflow++;
	}
}

/*********************************************************************/
/*Functions*/

//LED: Set LED states
void setLED(uint8_t led, uint8_t state)
{
	//Set LED 1
	if(led == 1)
	{
		led_state[0] = state;
	}

	//Set LED 2
	if(led == 2)
	{
		led_state[1] = state;
	}
}

//LED: Blink LED (count = -1: infinity)
void blinkLED(uint8_t led, uint8_t state, uint16_t ms, int8_t count)
{
	//Turn blinking ON
	if(state == ON)
	{
		//Change ARR of TIM1 according to blinking speed
		double arr = (0.5*ms)-1;
		TIM1->ARR = arr;

		//Start timer using interrupt
		HAL_TIM_Base_Start_IT(&htim1);

		//Activate desired LED
		led_blink_active[led-1] = ON;

		//Set counter
		led_blink_count[led-1] = count;
	}

	//Turn blinking OFF
	else
	{
		//Deactivate desired LED
		led_blink_active[led-1] = OFF;

		//Reset counter
		led_blink_count[led-1] = 0;

		//Check if timer is still needed and stop
		if(led_blink_active[0]+led_blink_active[1] == 0)
		{
			HAL_TIM_Base_Stop_IT(&htim1);
		}
	}
}

//LED: Blink LED by timer interrupt
void blinkLED_intRoutine()
{
	//Scan both LEDs
	for(uint8_t i = 0; i<=1; i++)
	{
		//Check for amount of blinks
		if(led_blink_count[i] != 0)
		{
			//Check if color should blink
			if(led_blink_active[i] == ON)
			{
				//Toggle  - ON
				if(led_blink_state[i] == OFF)
				{
					setLED(i+1, ON);
					led_blink_state[i] = ON;
				}

				//Toggle  - OFF
				else
				{
					setLED(i+1, OFF);
					led_blink_state[i] = OFF;

					//Check for counter
					if(led_blink_count[i] != -1)
					{
						led_blink_count[i]--;

						//Deactivate LED when done
						if(led_blink_count[i] == 0)
						{
							led_blink_active[i] = 0;

							if(led_blink_active[0]+led_blink_active[1] == 0)
							{
								//Stop Blinking timer
								HAL_TIM_Base_Stop_IT(&htim1);
							}
						}
					}
				}
			}
		}

	}
}


//Handle LED state
void common_Handler()
{
	if(led_state[0] == ON)
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	}

	if(led_state[0] == OFF)
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	}

	//Turn green only on when blue is off
	if((led_state[1] == ON) & (led_state[0] == OFF) & (led_blink_active[0] == OFF))
	{
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	}

	if(led_state[1] == OFF)
	{
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	}
}

