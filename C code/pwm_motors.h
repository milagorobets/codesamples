/*
 * pwm_motors.h
 *
 * Created: 3/21/2013 11:50:23 PM
 *  Author: Kerrigan
 */ 


#ifndef PWM_MOTORS_H_
#define PWM_MOTORS_H_


#define PWM_MOTOR1_PIN			AVR32_TC1_A2_0_PIN			// PA11 - Jumper 4, pin 7
#define PWM_MOTOR1_FUNCTION		AVR32_TC1_A2_0_FUNCTION
#define PWM_MOTOR1_CHANNEL		2

#define PWM_MOTOR2_PIN			AVR32_TC0_B2_0_PIN			// PX19 - Jumper 3, pin 8
#define PWM_MOTOR2_FUNCTION		AVR32_TC0_B2_0_FUNCTION
#define PWM_MOTOR2_CHANNEL		2

#define PWM_MOTOR3_PIN			AVR32_TC1_B0_0_PIN			// PA10 - Jumper 4, pin 6
#define PWM_MOTOR3_FUNCTION		AVR32_TC1_B0_0_FUNCTION
#define PWM_MOTOR3_CHANNEL		0

#define PWM_MOTOR4_PIN			AVR32_TC1_B1_0_PIN			// PA08 - Jumper 4, pin 8
#define PWM_MOTOR4_FUNCTION		AVR32_TC1_B1_0_FUNCTION
#define PWM_MOTOR4_CHANNEL		1

#define ESC_FREQ		350
#define TC_PERIOD		(((16500000UL)/8)/ESC_FREQ)
#define TC_MULTIPLE		(16500000UL)/8/1000

void initialize_PWM(void);
void update_Motors(void);

// Motor speed variables
extern volatile float motor1_speed;
extern volatile float motor2_speed;
extern volatile float motor3_speed;
extern volatile float motor4_speed;

extern volatile avr32_tc_t	 *tc0;	// PWM timer
extern volatile avr32_tc_t	 *tc1;	// PWM timer

#endif /* PWM_MOTORS_H_ */