/*
 * sensors.h
 *
 * Created: 3/21/2013 11:57:22 PM
 *  Author: Kerrigan
 */ 


#ifndef SENSORS_H_
#define SENSORS_H_

// For accelerometer
// Sensitivity = 8g (+/- 4g)
// Resolution = 12 bits
// Per division = 8/2^(12-1) = 3.90625 mg/digit; 1 g = 256
#define ACC_GAIN_GD		0.00390625/2
//#define ACC_GAIN_GD		0.0078125
#define GRAVITY 256

// For gyroscope
// Degrees per second = 2000
// Resolution = 16 bits
// Per division = 2000/2^(16-1) = 61.04 mdegrees/second/digit; 1 dps = 0.06104;
//#define GYRO_GAIN_DPS	0.06104
#define GYRO_GAIN_DPS	0.07
#define GYRO_GAIN_RPS	GYRO_GAIN_DPS*M_PI/180

// TWI stuff
#define TWIM (&AVR32_TWIM0)		//use TWIM0
#define GYRO_ADDR 0b1101011		//slave address of gyroscope
#define ACC_ADDR 0b0011001		//slave address of accelerometer
#define MAGN_ADDR 0b0011110		//slave address of magnetometer

// X is pitch, Y is roll, Z is yaw
extern volatile int g_x, g_y, g_z, a_x, a_y, a_z, m_x, m_y, m_z;
extern volatile int SENSOR_SIGN[9];

// ENABLING
void Enable_Gyro(volatile avr32_twim_t *TWIM_channel);
void Enable_Acc(volatile avr32_twim_t *TWIM_channel);
void Enable_Magn(volatile avr32_twim_t *TWIM_channel);

// FUNCTIONS TO INTERFACE WITH
void Read_Gyro(void);
void Read_Accel(void);
void Read_Mag(void);

// FUNCTIONS THAT GATHER DATA
void Request_Acc(volatile avr32_twim_t *TWIM_channel);
void Request_Magn(volatile avr32_twim_t *TWIM_channel);
void Request_Gyro(volatile avr32_twim_t *TWIM_channel);

// MISC
uint8_t readReg(uint8_t write_register,int address);

// EXTERNS
extern volatile int gyro_x, gyro_y, gyro_z;
extern volatile int accel_x, accel_y, accel_z;
extern volatile int magnetom_x, magnetom_y, magnetom_z;
extern volatile int RAW[6];						// Stores raw acc and gyro values
extern volatile int RAW_OFFSET[6];				// Stores offset for sensors

#endif /* SENSORS_H_ */