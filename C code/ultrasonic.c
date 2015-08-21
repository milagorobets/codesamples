 #include <delay.h>
 #include "ultrasonic.h"
 
 //External Globals
 volatile int ultrasonic_2;
 volatile int ultrasonic_3;
 volatile int ultrasonic_4;
 volatile int ultrasonic_5;
 volatile int ultrasonic_6;
 volatile int ultrasonic_7;
 volatile extern float des_roll;
 volatile extern float des_pitch;
 volatile extern float des_yaw;
 volatile extern float des_z;
 
 volatile int adc_2, adc_3, adc_4, adc_5, adc_6, adc_7;
 	 
 volatile int val = 10;
 volatile int val_vert = 5;
 
 volatile avr32_adc_t *adc = &AVR32_ADC;
 
 volatile uint8_t adc_2_distance = 0;
 volatile uint8_t adc_3_distance = 0;
 volatile uint8_t adc_4_distance = 0;
 volatile uint8_t adc_5_distance = 0;
 //Up pointed sensor
 volatile uint8_t adc_6_distance = 0;
 //Down pointed sensor
 volatile uint8_t adc_7_distance = 0;
 
 void enable_Ultrasonic()
 {
	 // Init delay module
	 delay_init(FOSC0);
	 
	 // GPIO pin/adc-function map.
	 static const gpio_map_t ADC_GPIO_MAP =
	 {
	 {ADC_ULTRA_SONIC_PIN_2, ADC_ULTRA_SONIC_FUNCTION_2},
	 {ADC_ULTRA_SONIC_PIN_3, ADC_ULTRA_SONIC_FUNCTION_3},
	 {ADC_ULTRA_SONIC_PIN_4, ADC_ULTRA_SONIC_FUNCTION_4},
	 {ADC_ULTRA_SONIC_PIN_5, ADC_ULTRA_SONIC_FUNCTION_5},
	 {ADC_ULTRA_SONIC_PIN_6, ADC_ULTRA_SONIC_FUNCTION_6},
	 {ADC_ULTRA_SONIC_PIN_7, ADC_ULTRA_SONIC_FUNCTION_7}
	 };
	 
	 // Assign and enable GPIO pins to the ADC function.
	 gpio_enable_module(ADC_GPIO_MAP, sizeof(ADC_GPIO_MAP) / sizeof(ADC_GPIO_MAP[0]));
	 
	 // configure ADC
	 // Lower the ADC clock to match the ADC characteristics (because we configured
	 // the CPU clock to 12MHz, and the ADC clock characteristics are usually lower;
	 // cf. the ADC Characteristic section in the data sheet).
	 AVR32_ADC.mr |= 0xa << AVR32_ADC_MR_PRESCAL_OFFSET;
	 adc_configure(adc);
	 
	 // Enable the ADC channels.
	 adc_enable(adc, ADC_ULTRA_SONIC_CHANNEL_2);
	 adc_enable(adc, ADC_ULTRA_SONIC_CHANNEL_3);
	 adc_enable(adc, ADC_ULTRA_SONIC_CHANNEL_4);
	 adc_enable(adc, ADC_ULTRA_SONIC_CHANNEL_5);
	 adc_enable(adc, ADC_ULTRA_SONIC_CHANNEL_6);
	 adc_enable(adc, ADC_ULTRA_SONIC_CHANNEL_7);

	// launch conversion on all enabled channels

} 

void update_Ultrasonic()
{
		adc_start(adc);
		// get value for the ULTRA_SONIC adc channel
		adc_2_distance = adc_get_value(adc, ADC_ULTRA_SONIC_CHANNEL_2);
		adc_3_distance = adc_get_value(adc, ADC_ULTRA_SONIC_CHANNEL_3);
		adc_4_distance = adc_get_value(adc, ADC_ULTRA_SONIC_CHANNEL_4);
		adc_5_distance = adc_get_value(adc, ADC_ULTRA_SONIC_CHANNEL_5);
		adc_6_distance = adc_get_value(adc, ADC_ULTRA_SONIC_CHANNEL_6);
		adc_7_distance = adc_get_value(adc, ADC_ULTRA_SONIC_CHANNEL_7);

		//Convert adc values to decimeters
		ultrasonic_2 = .127 * adc_2_distance;
		ultrasonic_3 = .127 * adc_3_distance;
		ultrasonic_4 = .127 * adc_4_distance;
		ultrasonic_5 = .127 * adc_5_distance;
		ultrasonic_6 = .127 * adc_6_distance;
		ultrasonic_7 = .127 * adc_7_distance;
		
		//Check the values read into the ADC's
		adc_2 = check_adc_values(adc_2_distance);	// Left Sensor
		adc_3 = check_adc_values(adc_3_distance);	// Right Sensor
		adc_4 = check_adc_values(adc_4_distance);	//
		adc_5 = check_adc_values(adc_5_distance);	//
		adc_6 = check_adc_values(adc_6_distance);	// Front Sensor
		adc_7 = check_adc_values(adc_7_distance);	// Back Sensor
		
		//Determine Roll,Pitch and Yaw commands
		//des_roll = (adc_3*val) - (adc_5*val);
		//des_pitch = (adc_2*val) - (adc_4*val);
		//des_z = (adc_6*val_vert) - (adc_7*val_vert);	
}

void disable_Ultrasonic()
{
		// Disable the ADC channels
		adc_disable(adc, ADC_ULTRA_SONIC_CHANNEL_2);
		adc_disable(adc, ADC_ULTRA_SONIC_CHANNEL_3);
		adc_disable(adc, ADC_ULTRA_SONIC_CHANNEL_4);
		adc_disable(adc, ADC_ULTRA_SONIC_CHANNEL_5);
		adc_disable(adc, ADC_ULTRA_SONIC_CHANNEL_6);
		adc_disable(adc, ADC_ULTRA_SONIC_CHANNEL_7);

}
 
 int check_adc_values(signed short adc_value)
{
	 int check_val = 0;
	 
	 //Check ADC value
	 if(adc_value <39)
	 {
		 check_val = 1;
	 }
	 else
	 {
		 check_val = 0;
	 }
	 return check_val;
}

	  