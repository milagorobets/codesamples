#ifndef _ULTRASONIC_H_
#define _ULTRASONIC_H_

#include <asf.h>
#include <string.h>
#include "freq.h"

// Externs
volatile extern int ultrasonic_2;
volatile extern int ultrasonic_3;
volatile extern int ultrasonic_4;
volatile extern int ultrasonic_5;
volatile extern int ultrasonic_6;
volatile extern int ultrasonic_7;
volatile extern float des_roll;
volatile extern float des_pitch;
volatile extern float des_yaw;
volatile extern float des_z;

volatile extern int adc_2, adc_3, adc_4, adc_5, adc_6, adc_7;

volatile extern int val;
volatile extern int val_vert;

volatile extern uint8_t adc_2_distance;
volatile extern uint8_t adc_3_distance;
volatile extern uint8_t adc_4_distance;
volatile extern uint8_t adc_5_distance;
volatile extern uint8_t adc_6_distance;
volatile extern uint8_t adc_7_distance;


 // Connection of the ULTRA_SONIC sensor
 #define ADC_ULTRA_SONIC_CHANNEL_2     2
 #define ADC_ULTRA_SONIC_PIN_2         AVR32_ADC_AD_2_PIN
 #define ADC_ULTRA_SONIC_FUNCTION_2    AVR32_ADC_AD_2_FUNCTION
 
 #define ADC_ULTRA_SONIC_CHANNEL_3     3
 #define ADC_ULTRA_SONIC_PIN_3         AVR32_ADC_AD_3_PIN
 #define ADC_ULTRA_SONIC_FUNCTION_3    AVR32_ADC_AD_3_FUNCTION
 
 #define ADC_ULTRA_SONIC_CHANNEL_4     4
 #define ADC_ULTRA_SONIC_PIN_4         AVR32_ADC_AD_4_PIN
 #define ADC_ULTRA_SONIC_FUNCTION_4    AVR32_ADC_AD_4_FUNCTION
 
 #define ADC_ULTRA_SONIC_CHANNEL_5     5
 #define ADC_ULTRA_SONIC_PIN_5         AVR32_ADC_AD_5_PIN
 #define ADC_ULTRA_SONIC_FUNCTION_5    AVR32_ADC_AD_5_FUNCTION
 
 #define ADC_ULTRA_SONIC_CHANNEL_6     6
 #define ADC_ULTRA_SONIC_PIN_6         AVR32_ADC_AD_6_PIN
 #define ADC_ULTRA_SONIC_FUNCTION_6    AVR32_ADC_AD_6_FUNCTION
 
 #define ADC_ULTRA_SONIC_CHANNEL_7     7
 #define ADC_ULTRA_SONIC_PIN_7         AVR32_ADC_AD_7_PIN
 #define ADC_ULTRA_SONIC_FUNCTION_7    AVR32_ADC_AD_7_FUNCTION
 
 //Prototypes
 int check_adc_values(signed short adc_value);
 void enable_Ultrasonic(void);
 void disable_Ultrasonic(void);
 void update_Ultrasonic(void); 
 
 #endif