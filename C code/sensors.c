/*
 * sensors.c
 *
 * Created: 3/21/2013 11:51:18 PM
 *  Author: Kerrigan
 */ 
#include <asf.h>
#include "sensors.h"

extern volatile int g_x, g_y, g_z, a_x, a_y, a_z, m_x, m_y, m_z;
extern volatile int SENSOR_SIGN[9];


// ENABLING SENSORS
void Enable_Gyro(volatile avr32_twim_t *TWIM_channel){
	
	uint8_t write_register = 0x20;	//CTRL_REG1
	uint8_t write_command = 0x3f;	//Enable GYRO command
	uint8_t write_register2 = 0x23;	//CTRL_REG4
	uint8_t write_command2 = 0x20;	//GYRO sampling rate command -> 2000 dps -> FS:10
	uint8_t write_register3 = 0x24;
	uint8_t write_command3 = 0x00;
	twim_package_t enable_package;
	
	uint8_t write_data[] = {	//Create Array of register address and command data
	write_register,
	write_command
	};

	enable_package.chip = GYRO_ADDR;
	enable_package.addr_length = 0;	//no address length specified
	enable_package.buffer = (void*) write_data;
	enable_package.length = 2;	//1 byte for register, 1 byte for command data

	twim_write_packet(TWIM, &enable_package);

	uint8_t write_data2[] = {	//Create Array of register address and command data
	write_register2,
	write_command2
	};
	
	enable_package.chip = GYRO_ADDR;
	enable_package.addr_length = 0;	//no address length specified
	enable_package.buffer = (void*) write_data2;
	enable_package.length = 2;	//1 byte for register, 1 byte for command data
	
	twim_write_packet(TWIM, &enable_package);
	
		uint8_t write_data3[] = {	//Create Array of register address and command data
			write_register3,
			write_command3
		};
		
		enable_package.chip = GYRO_ADDR;
		enable_package.addr_length = 0;	//no address length specified
		enable_package.buffer = (void*) write_data3;
		enable_package.length = 2;	//1 byte for register, 1 byte for command data
		
		twim_write_packet(TWIM, &enable_package);

}

void Enable_Acc(volatile avr32_twim_t *TWIM_channel)
{
	uint8_t write_register = 0x20;	//CTRL_REG1_A
	uint8_t write_command = 0x5f;	//Enable ACC command
	uint8_t write_register2 = 0x23;	//CTRL_REG4
	uint8_t write_command2 = 0x10;	//ACC sampling rate command -> +/- 4g -> FS:01
	twim_package_t enable_package;
	
	uint8_t write_data[] = {	//Create Array of register address and command data
		write_register,
		write_command
	};

	enable_package.chip = ACC_ADDR;
	enable_package.addr_length = 0;	//no address length specified
	enable_package.buffer = (void*) write_data;
	enable_package.length = 2;	//1 byte for register, 1 byte for command data

	twim_write_packet(TWIM, &enable_package);

	uint8_t write_data2[] = {	//Create Array of register address and command data
		write_register2,
		write_command2
	};

	enable_package.chip = ACC_ADDR;
	enable_package.addr_length = 0;	//no address length specified
	enable_package.buffer = (void*) write_data2;
	enable_package.length = 2;	//1 byte for register, 1 byte for command data

	twim_write_packet(TWIM, &enable_package);
}
void Enable_Magn(volatile avr32_twim_t *TWIM_channel)
{	
	uint8_t write_register = 0x02;	//CTRL_REG_M
	uint8_t write_command = 0x00;	//Enable MAGN command
	twim_package_t enable_package;
	
	uint8_t write_data[] = {	//Create Array of register address and command data
	write_register,
	write_command
	};

	enable_package.chip = MAGN_ADDR;
	enable_package.addr_length = 0;	//no address length specified
	enable_package.buffer = (void*) write_data;
	enable_package.length = 2;	//1 byte for register, 1 byte for command data

	twim_write_packet(TWIM, &enable_package);	
}

// READING FUNCTIONS
void Read_Gyro()
{
	Request_Gyro(TWIM);
	
	RAW[0] = g_x;
	RAW[1] = g_y;
	RAW[2] = g_z;
	gyro_x = SENSOR_SIGN[0] * (g_x - RAW_OFFSET[0]);
	gyro_y = SENSOR_SIGN[1] * (g_y - RAW_OFFSET[1]);
	gyro_z = SENSOR_SIGN[2] * (g_z - RAW_OFFSET[2]);
}

// Reads x,y and z accelerometer registers
void Read_Accel(void)
{
	Request_Acc(TWIM);
	
	RAW[3] = a_x;
	RAW[4] = a_y;
	RAW[5] = a_z;
	accel_x = SENSOR_SIGN[3] * (a_x - RAW_OFFSET[3]);
	accel_y = SENSOR_SIGN[4] * (a_y - RAW_OFFSET[4]);
	accel_z = SENSOR_SIGN[5] * (a_z - RAW_OFFSET[5]);
}

void Read_Mag()
{
	Request_Magn(TWIM);
	magnetom_x = SENSOR_SIGN[6] * m_x;
	magnetom_y = SENSOR_SIGN[7] * m_y;
	magnetom_z = SENSOR_SIGN[8] * m_z;
}

// USING SENSORS
void Request_Acc(volatile avr32_twim_t *TWIM_channel)
{
	uint8_t write_command = 0b10101000;		//'1' enables next address to be read next, 0101000 is the register address of GYRO_X_L
	int nbytes_write = 1;
	uint8_t read_buffer_acc[6];
	int nbytes_read = 6;

	twim_write(TWIM_channel,&write_command,nbytes_write,ACC_ADDR,0);
	twim_read (TWIM_channel,read_buffer_acc,nbytes_read,ACC_ADDR,0);

	a_x = (int16_t)((read_buffer_acc[1] << 8) | read_buffer_acc[0])>>4;
	a_y = (int16_t)((read_buffer_acc[3] << 8) | read_buffer_acc[2])>>4;
	a_z = (int16_t)((read_buffer_acc[5] << 8) | read_buffer_acc[4])>>4;
}
void Request_Magn(volatile avr32_twim_t *TWIM_channel)
{
	uint8_t write_command = 0b00000011;		//'1' enables next address to be read next, 0101000 is the register address of GYRO_X_L
	int nbytes_write = 1;
	uint8_t read_buffer_magn[6];
	int nbytes_read = 6;

	twim_write(TWIM_channel,&write_command,nbytes_write,MAGN_ADDR,0);
	twim_read (TWIM_channel,read_buffer_magn,nbytes_read,MAGN_ADDR,0);

	m_x = (int16_t)((read_buffer_magn[1] << 8) | (read_buffer_magn[0]));
	m_y = (int16_t)((read_buffer_magn[3] << 8) | (read_buffer_magn[2]));
	m_z = (int16_t)((read_buffer_magn[5] << 8) | (read_buffer_magn[4]));
}
void Request_Gyro(volatile avr32_twim_t *TWIM_channel)
{
	uint8_t write_command = 0b10101000;		//'1' enables next address to be read next, 0101000 is the register address of GYRO_X_L
	int nbytes_write = 1;
	uint8_t read_buffer_gyro[6];
	int nbytes_read = 6;

	twim_write(TWIM_channel,&write_command,nbytes_write,GYRO_ADDR,0);
	twim_read (TWIM_channel,read_buffer_gyro,nbytes_read,GYRO_ADDR,0);

	g_x = (int16_t)((read_buffer_gyro[1] << 8) | (read_buffer_gyro[0]));
	g_y = (int16_t)((read_buffer_gyro[3] << 8) | (read_buffer_gyro[2]));
	g_z = (int16_t)((read_buffer_gyro[5] << 8) | (read_buffer_gyro[4]));
}


// USEFUL FOR CHECKING TWI
uint8_t readReg(uint8_t write_register,int address)
{
	uint8_t read_register;
	
	twim_write(TWIM, &write_register, 1, address, 0);
	twim_read(TWIM, &read_register, 1 ,address,0);
	
	return read_register;
}