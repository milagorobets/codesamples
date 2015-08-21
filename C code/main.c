/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */


#include <asf.h>
#include <delay.h>
#include <stdint.h>
#include <math.h>
#include <inttypes.h>
#include <rtc.h>
#include <pm.h>
#include <tc.h>
#include <usart.h>
#include <adc.h>
#include "pwm_motors.h"
#include "sensors.h"
#include "ultrasonic.h"
#include "BT.h"

/*~~~~~~~~~~~~ DEFINES ~~~~~~~~~~~~~~~~~~~~~~~~~~*/

// Constants for control system
#define KP_Z  0
#define KI_Z  0
#define KD_Z  0

#define KP_ROLL  0.95	//
#define KI_ROLL  1.2	//
#define KD_ROLL  0.35		//

#define KP_PITCH  0.95	
#define KI_PITCH  1.2
#define KD_PITCH  0.35

#define KPYAW  0
#define KIYAW  1
#define KDYAW  .01

#define TRUST_GAIN	0.1   // Trust in ACC


// Clocks
#define F_CPU 66000000UL
#define FCPU_HZ 66000000UL
#define FPBA_HZ 16500000UL


// Averaging filter
#define FILTER_LENGTH	20


/*~~~~~~~~~~~~ GLOBALS ~~~~~~~~~~~~~~~~~~~~~~~~~~*/

// Variables for control system
volatile float roll;
volatile float pitch;
volatile float yaw;
volatile float z;

volatile float g_roll = 0;
volatile float g_pitch = 0;
volatile float g_yaw = 0;

volatile float a_roll = 0;
volatile float a_pitch = 0;

// Desired values for control system
volatile float des_roll = 0;
volatile float des_pitch = 0;
volatile float des_yaw = 0;
volatile float des_z = 0;

// Sensor related variables
volatile int g_x, g_y, g_z, a_x, a_y, a_z, m_x, m_y, m_z;
volatile int RAW[6];						// Stores raw acc and gyro values
volatile int RAW_OFFSET[6] = {0};			// Stores offset for sensors
volatile int SENSOR_SIGN[9] = {1, 1, 1, -1, -1, -1, 1, 1, 1};
volatile int gyro_x, gyro_y, gyro_z;
volatile int accel_x, accel_y, accel_z;
volatile int magnetom_x, magnetom_y, magnetom_z;

// Keeping track of time
volatile unsigned long timer_current = 0;
volatile unsigned long timer_old = 0;
volatile avr32_tc_t	 *tc0 = (&AVR32_TC0);	// PWM timer
volatile avr32_tc_t	 *tc1 = (&AVR32_TC1);	// PWM timer
volatile float g_dt = 0;

// Motor speed variables
volatile float motor1_speed = 0;
volatile float motor2_speed = 0;
volatile float motor3_speed = 0;
volatile float motor4_speed = 0;

// Derivative term filter
volatile float FIFO_deriv_roll[FILTER_LENGTH] = {0};
	
// Communication system data
volatile double meas_roll = 0;
volatile double meas_pitch= 0;
volatile double meas_yaw = 0;
volatile double meas_z = 0;
volatile int global_data[5];
volatile char global_command;			//current command being saved
volatile int global_data_counter = 0;	//data counter for data being read
volatile int global_is_command = 0;		//determines if command or not
volatile int global_ready = 0;			//check this flag if data/command is ready for us in control system

/*~~~~~~~~~~~~ CLOCK CONFIGURATION ~~~~~~~~~~~~~~~~~~~~~*/
pcl_freq_param_t pcl_freq_param =
{
	.cpu_f=FCPU_HZ,
	.pba_f=FPBA_HZ,
	.osc0_f=FOSC0,
	.osc0_startup=OSC0_STARTUP,
};

/*~~~~~~~~~~~~ TWIM CONFIGURATION ~~~~~~~~~~~~~~~~~~~~~~*/
const static twim_options_t twi_opt_GYRO = {
	//! The PBA clock frequency.
	.pba_hz=FPBA_HZ,
	//! The baud rate of the TWI bus.
	.speed=9600,
	//! The desired address.
	.chip=GYRO_ADDR,
	//! SMBUS mode
	.smbus=0, //Do not use SMBus
};

const static twim_options_t twi_opt_ACC = {
	//! The PBA clock frequency.
	.pba_hz=FPBA_HZ,
	//! The baud rate of the TWI bus.
	.speed=9600,
	//! The desired address.
	.chip=ACC_ADDR,
	//! SMBUS mode
	.smbus=0, //Do not use SMBus
};

const static twim_options_t twi_opt_MAGN = {
	//! The PBA clock frequency.
	.pba_hz=FPBA_HZ,
	//! The baud rate of the TWI bus.
	.speed=9600,
	//! The desired address.
	.chip=MAGN_ADDR,
	//! SMBUS mode
	.smbus=0, //Do not use SMBus
};

//~~~~~~~~~~~~~~~~~~~USART CONFIGURATION ~~~~~~~~~~~~~~~~//
const usart_options_t usart_opt = {
	.baudrate    = 9600,
	.channelmode = AVR32_USART_MR_CHMODE_NORMAL,
	.charlength  = 8,
	.paritytype  = AVR32_USART_MR_PAR_NONE,
	.stopbits    = AVR32_USART_MR_NBSTOP_1,
};

const usart_options_t usart_opt_2 = {
	.baudrate    = 115200,
	.channelmode = AVR32_USART_MR_CHMODE_NORMAL,
	.charlength  = 8,
	.paritytype  = AVR32_USART_MR_PAR_NONE,
	.stopbits    = AVR32_USART_MR_NBSTOP_1,
};

void get_Angles(void);
void initialize_IMU(void);
static void twim_init(void);
float atan2new(float x, float y);

/*~~~~~~~~~~~~ ISR ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
static void usart_int_handler(void)
{
    int rx_character;
     /*
     * In the code line below, the interrupt priority level does not need to
     * be explicitly masked as it is already because we are within the
     * interrupt handler.
     * The USART Rx interrupt flag is cleared by side effect when reading
     * the received character.
     * Waiting until the interrupt has actually been cleared is here useless
     * as the call to usart_write_char will take enough time for this before
     * the interrupt handler is left and the interrupt priority level is
     * unmasked by the CPU.
     */
    usart_read_char(USART, &rx_character);

	if(global_is_command == 0)
	{
		switch(rx_character)
		{
			case 'a':
			{
				//Get Roll, Pitch, Yaw, Z
				//request data
				//usart_write_char(USART, rx_character);
				//usart_write_line(USART,"\r\nGet Motor Speed #1\r\n");
				global_command = rx_character;
				global_is_command = 1;
				global_ready = 0;
				break;
			}
			case 'R':
			{
				//End transmission
				//end ISR
				//usart_write_line(USART,"\r\nEnding Transmission\r\n");
				global_is_command = 0;
				global_ready = 0;
				break;

			}
			default:
			{
				//error
				//end ISR
				//usart_write_line(USART,"\r\nBad Command\r\n");
				global_is_command = 0;
				global_ready = 0;
				break;
			}
		}
	}
	else //received character is data
	{
		if(global_data_counter < 4)		//if data 
		{
			//usart_write_char(USART, rx_character);
			global_data[global_data_counter] = rx_character;	//save char as data
			global_data_counter++;		//increment data counter
		}
		else
		{
			if(global_data_counter == 4)
			{
				//usart_write_char(USART, rx_character);
				global_data[global_data_counter] = rx_character;	//save last received char
				global_is_command = 0;	//next char should be a command
				global_ready = 1;		//data is ready
				global_data_counter =0;	//reset data counter
				//usart_write_line(USART,"\r\nData Saved\r\n");
			}
		}			
	}	
}
/*~~~~~~~~~~~~ BEGINNING OF CODE ~~~~~~~~~~~~~~~~~~~~~~*/

int main (void)
{	
	// Variables -- Misc
	const twim_options_t twi_option_GYRO = twi_opt_GYRO;
	const twim_options_t twi_option_ACC = twi_opt_ACC;
	const twim_options_t twi_option_MAGN = twi_opt_MAGN;
	
	// Variables -- Control system
	float prev_e_z = 0;
	float prev_e_roll = 0;
	float prev_e_pitch = 0;
	float prev_e_yaw = 0;
	float eint_z = 0;
	float eint_roll = 0;
	float eint_pitch = 0;
	float eint_yaw = 0;
	float pitch_ederiv = 0; // Error derivative
	float roll_ederiv = 0;
	float prev_deriv_e_roll = 0;
	float yaw_ederiv = 0;
	float z_ederiv = 0;
	float e = 0;
	float dt;					// Time between samples
	unsigned long t1 = 0;
	unsigned long t2 = 0;
	float u1, u2, u3, u4 = 0;
	float m1, m2, m3, m4 = 0;
	float g_force = 240;
	int xcount = 0;
	float gain = 1;
	int time_to_update_data = 0;
	
	int deriv_count=0;
	
	const usart_options_t usart_option =  usart_opt;
	const usart_options_t usart_option_2 =  usart_opt_2;
	
	// Setting up the board		
	pcl_configure_clocks(&pcl_freq_param);
	board_init();
	
	// Initialize Bluetooth
	configure_AT(FPBA_HZ, usart_option);
	
	irq_initialize_vectors();
	cpu_irq_enable();
	
	Disable_global_interrupt();
	
	// Initialize interrupt vectors.
	INTC_init_interrupts();		
	INTC_register_interrupt(&usart_int_handler, USART_IRQ, AVR32_INTC_INT0);
	
	USART->ier = AVR32_USART_IER_RXRDY_MASK;
	
	// Enable all interrupts.
	Enable_global_interrupt();		
	
	// Enable Ultrasonic sensors
	enable_Ultrasonic();
	
	// Initialize motors
	initialize_PWM();
	

	
	tc_start(tc1, PWM_MOTOR1_CHANNEL);
	tc_start(tc0, PWM_MOTOR2_CHANNEL);
	tc_start(tc1, PWM_MOTOR3_CHANNEL);
	tc_start(tc1, PWM_MOTOR4_CHANNEL);
	
	// Waits for button press here after battery is connected
	while (gpio_get_pin_value(GPIO_PUSH_BUTTON_0));
	delay_ms(1000);
	/*
	while(1){
		while (motor2_speed<60)
		{
			motor2_speed += 1;	
			update_Motors();
			delay_ms(300);
		}
		while (motor2_speed>0)
		{
			motor2_speed -= 1;
			update_Motors();
			delay_ms(300);
		}
	}	
	while (gpio_get_pin_value(GPIO_PUSH_BUTTON_0));*/
	
	//delay_ms(3000);
	
	// Initialize RTC
	//AVR32_RTC->ctrl & AVR32_RTC_CTRL_BUSY_MASK = 0;
	rtc_init(&AVR32_RTC, RTC_OSC_32KHZ, 1);
	rtc_enable(&AVR32_RTC);
	
	// Initialize IMU parts
	initialize_IMU();
	
	// TESTING ONLY

	float test_array[2000];
	float test_array_2[2000];
	float test_array_3[2000];
	
	for (int k = 0; k < FILTER_LENGTH; k++){
		FIFO_deriv_roll[k] = roll;
	}
	
	time_to_update_data = 0;
	
	// Control system code begins
	while(1)
	{
		t1 = rtc_get_value(&AVR32_RTC); // Current time in ms
		dt = ((float)(t1-t2))*0.001*0.125;	// dt in seconds
		if (t1 < t2) // Timer overflowed
		{
			dt = ((float)(t1 + rtc_get_top_value(&AVR32_RTC))*0.001*0.125);		
		}
		t2 = t1;
		
		get_Angles();
		//pitch = 0;
		yaw = 0;
		
		e = des_z-z;											//calculating height error
		eint_z = eint_z + ((e+prev_e_z)/2)*dt;					//calculate error integral term
		z_ederiv = (e - prev_e_z)/dt;								//calculate error derivative term
		u1 =(KP_Z*e) + (KI_Z*eint_z) + (KD_Z*z_ederiv)+g_force;	//calculating control output
		prev_e_z=e;
		
		//ROLL
		e = des_roll-roll;										//calculating roll error
		eint_roll = eint_roll + ((e+prev_e_roll)/2)*dt;			//calculate error integral term
		prev_e_roll=e;
		
		for (int i = 0; i < (FILTER_LENGTH - 1); i++)
		{
			FIFO_deriv_roll[i] = FIFO_deriv_roll[i+1];	
		}
		FIFO_deriv_roll[FILTER_LENGTH-1] = e;
		
		roll_ederiv = (FIFO_deriv_roll[FILTER_LENGTH-1] - FIFO_deriv_roll[0])/(FILTER_LENGTH*dt);
	
		u2 =(KP_ROLL*e) + (KI_ROLL*eint_roll) + (KD_ROLL*roll_ederiv); //calculating control output

		if(xcount < 2000){
			test_array[xcount] = roll_ederiv;
			test_array_2[xcount] = roll;
			test_array_3[xcount] = g_roll;
		}		
		xcount++;
		
		//PITCH
		e = des_pitch-pitch;									//calculating pitch error
		eint_pitch = eint_pitch + ((e+prev_e_pitch)/2)*dt;		//calculate error integral term
		pitch_ederiv = (e - prev_e_pitch)/dt;							//calculate error derivative term
		u3 =(KP_PITCH*e) + (KI_PITCH*eint_pitch) + (KD_PITCH*pitch_ederiv);	//calculating control output
		prev_e_pitch=e;
		
		//YAW
		e = des_yaw-yaw;										//calculating yaw error
		eint_yaw = eint_yaw + ((e+prev_e_yaw)/2)*dt;			//calculate error integral term
		yaw_ederiv = (e - prev_e_yaw)/dt;							//calculate error derivative term
		u4 =(KPYAW*e) + (KIYAW*eint_yaw) + (KDYAW*yaw_ederiv);		//calculating control output
		prev_e_yaw=e;
		
		//MOTOR SPEEDS
		
		m1=(0.25*u1+0.5*u2+0.25*u4)*gain;
		m2=(0.25*u1+0.5*u3-0.25*u4)*gain;
		m3=(0.25*u1-0.5*u2+0.25*u4)*gain;
		m4=(0.25*u1-0.5*u3-0.25*u4)*gain;
		
		if (m1 > 95)
		m1 = 95;
		else if (m1 < 5)
		m1 = 5;
		
		if (m2 > 95)
		{
			m2 = 95;
		}
		else if (m2 < 5)
		{
			m2 = 5;
		}		
				
		if (m3 > 95)
		{
			m3 = 95;
		}
		else if (m3 < 5)
		{
			m3 = 5;
		}
		
		if (m4 > 95)
		{
			m4 = 95;
		}
		else if (m4 < 5)
		{
			m4 = 5;
		}
		
		motor1_speed = m2; //m2
		motor2_speed = m1; //m3
		motor3_speed = m4; //m4
		motor4_speed = m3; //m1..... the imu was turned 90 degrees....
		
		update_Motors();
		

		//Bluetooth Send
		if (time_to_update_data > 3)
		{
			// Get Ultrasonic data
			update_Ultrasonic();
					
			// Send out update through Bluetooth
			meas_roll = roll;
			meas_pitch = pitch;
			meas_yaw = yaw;

			transmitted_data();
			time_to_update_data = 0;
			
			received_data();			
		}
		else
		{
			delay_ms(7);			
		}	
		time_to_update_data++;		

	}
	
}

static void twim_init(void)
{
	uint8_t status;
	const gpio_map_t TWIM_GPIO_MAP = {
		(AVR32_TWIMS0_TWCK_0_0_PIN, AVR32_TWIMS0_TWCK_0_0_FUNCTION,AVR32_TWIMS0_TWD_0_0_PIN, AVR32_TWIMS0_TWD_0_0_FUNCTION)
	};

	gpio_enable_module(TWIM_GPIO_MAP, sizeof(TWIM_GPIO_MAP)/sizeof(TWIM_GPIO_MAP[0]));
	status = twim_master_init(TWIM, &twi_opt_GYRO);	
	
	twim_set_speed (TWIM, TWI_STD_MODE_SPEED, FPBA_HZ);
}

volatile void initialize_IMU(void)
{
	twim_init();
	Enable_Acc(TWIM);
	Enable_Gyro(TWIM);
	Enable_Magn(TWIM);
	
	delay_ms(20);
	
	for (int i = 0; i < 32; i++)
	{
		Read_Gyro();
		Read_Accel();
		RAW_OFFSET[0] += RAW[0];
		RAW_OFFSET[1] += RAW[1];		
		RAW_OFFSET[2] += RAW[2];	
		RAW_OFFSET[3] += RAW[3];	
		RAW_OFFSET[4] += RAW[4];	
		RAW_OFFSET[5] += RAW[5];	
		delay_ms(20);
	}
	
	RAW_OFFSET[0] = 0;
	RAW_OFFSET[1] = 0;
	RAW_OFFSET[2] = 0;
	RAW_OFFSET[3] = 0;
	RAW_OFFSET[4] = 0;
	RAW_OFFSET[5] = 0;
	
	timer_current = rtc_get_value(&AVR32_RTC);
	delay_ms(20);
	
	get_Angles();	
}

volatile void get_Angles()
{
	static float prev_g_d_roll  = 0;
	timer_old = timer_current;
	timer_current = rtc_get_value(&AVR32_RTC);

	if (timer_current > timer_old)
	{
		g_dt = (float)((timer_current-timer_old))*0.001*0.125*0.98; // Time elapsed in seconds -- sensors are in seconds
	}	
	else
	{
		g_dt = 0;		
	}
	
	// GET DATA
	Read_Gyro();
	Read_Accel();
	
	
	// GYROSCOPE
	
	// Y = roll
	float g_d_roll = gyro_x * GYRO_GAIN_RPS; // delta roll in radians/sec
	//g_roll = roll + g_d_roll * g_dt;
	g_roll = roll + ((g_d_roll+prev_g_d_roll)/2)*g_dt;
	prev_g_d_roll = g_d_roll;
	
	// X = pitch
	float g_d_pitch = gyro_y * GYRO_GAIN_RPS; // delta pitch in radians/sec
	g_pitch = pitch + g_d_pitch * g_dt;
	
	// Z = yaw
	float g_d_yaw = gyro_z * GYRO_GAIN_RPS; // delta yaw in radians/sec
	g_yaw = yaw + g_d_yaw * g_dt;
	
	
	// ACCELEROMETER
	float acc_x = accel_x * 0.002;
	float acc_y = accel_y * 0.002;
	float acc_z = accel_z * 0.002;
	
	a_roll = atan2(-acc_y, acc_z);
	
	a_pitch = atan(acc_x/sqrt(acc_y*acc_y + acc_z*acc_z));
	
	// Weigh
	roll = TRUST_GAIN * a_roll + (1-TRUST_GAIN) * g_roll;
	pitch = TRUST_GAIN * a_pitch + (1-TRUST_GAIN) * g_pitch;

	
}

float atan2new(float x, float y)
{
	float result = atan(y/x);
	if (x > 0)
	{
		return result;		// x > 0 - result already valid
	}
	else if (x == 0)
	{
		if (y>0)
		{
			return (M_PI_2);	// y > 0 , x = 0
		}
		else if (y<0)
		{
			return (-M_PI_2);	// y < 0 , x = 0
		}
		return 0; // y = 0, x = 0
	}
	else if (y >= 0)
	{
		return (result + M_PI); // y >= 0, x < 0
	}
	return (result - M_PI); // y < 0, x < 0
}

