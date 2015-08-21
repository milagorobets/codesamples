/*! \file RobotController.c \Maintains main loop */
//*****************************************************************************
//
// File Name	: 'RobotController.c'
// Title		: Maintains main loop
// Author		: Mila Gorobets
// Created		: 24/08/2012
// Revised		: 24/08/2012
// Version		: 1.0
// Target MCU	: any
//
//
//*****************************************************************************

#include "common.h"

    int main(void) {


      DDRA = 0xFF; // Set 0x00001111
	  			   // PA0: (o) LCD Register Set
				   // PA1: (o) LCD Enable
				   // PA2: (o) LCD Read/(not-Write)
				   // PA3: (o) Status LED
				   // PA4:
				   // PA5:
				   // PA6:
				   // PA7:
	  DDRB = 0x00; // Set 0x00000000
	  			   // PB0: (i)
				   // PB1: (i)
				   // PB2: (i)
				   // PB3: (i)
				   // PB4: (i)
				   // PB5: (i)
				   // PB6: (i)
				   // PB7: (i)
	  DDRC = 0xFF; // Set 0x11111111
	  			   // PC0: (o) LCD data bit 0
				   // PC1: (o) LCD data bit 1
				   // PC2: (o) LCD data bit 2
				   // PC3: (o) LCD data bit 3
				   // PC4: (o) LCD data bit 4
				   // PC5: (o) LCD data bit 5
				   // PC6: (o) LCD data bit 6
				   // PC7: (o) LCD data bit 7
	  DDRD = 0x00; // Set 0x00000000
	  			   // PB0: (i) UART RXD0
				   // PB1: (i) UART TXD0
				   // PB2: (i)
				   // PB3: (i)
				   // PB4: (i)
				   // PB5: (i)
				   // PB6: (i)
				   // PB7: (i)

	char set_copy[20];
	
	uartInit();					// initialize UART (serial port)
	uartSetBaudRate(19200);		// set UART speed to 9600 baud
	rprintfInit(uartSendByte);  // configure rprintf to use UART for output
	servo_table[0] = 10000;
	servo_table[1] = 10000;
	servo_table[2] = 10000;
	servo_table[3] = 10000;
	servo_table[4] = 10000;
	servo_table[5] = 10000;
	servo_table[6] = 10000;
	servo_table[7] = 10000;
	servo_table[8] = 10000;
	servo_table[9] = 10000;



	initialize_Servos();
	sei();
	
	while(1)
		{
			if(uartReceiveBufferHasSet())
			{
				for (int b = 0; b < 19; b++)
				{
					set_copy[b] = uartGetByte();
				}
				set_copy[19] = '\0';
				rprintfStr(set_copy);
				if (set_copy[18] == 66) // Data transfer terminated correctly
				{ 
					update_counts(set_copy);
					uartSendByte('\n'); // Confirm data received
					uartSendByte('Q'); // Confirm data received
				}
				else if (set_copy[18] == 'R')
				{
					// Emergency shut off
					// Disable all interrupts
					TIFR1 = (1 << OCF1A)|(1 << OCF1B)|(1<< TOV1);
  					TIMSK1 = (0<<OCIE1B)|(0<<OCIE1A)|(0<<TOIE1);
					// Set outputs to zero
					PORTC = 0x00;
					PORTD &= ~(0x80);
					while(1);
				}
				else 
				{
					uartSendByte('A'); // Error occurred, data will be resent
				}
			}
		}
				
	return(1);

}


	

