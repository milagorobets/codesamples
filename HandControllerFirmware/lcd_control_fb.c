/*! \file lcd_control_fb.c \Four-bit LCD Controller */
//*****************************************************************************
//
// File Name	: 'lcd_control_fb.c'
// Title		: Functions to handle LCD interface with 4 data bits
// Author		: Mila Gorobets
// Created		: 24/08/2012
// Revised		: 24/08/2012
// Version		: 1.0
// Target MCU	: any
//
//
//*****************************************************************************

#include "common.h"

/* -----------------------------------------------------
	This function checks the busy flag of the LCD screen
	If flag is set, the function waits
	Once the busy bit is cleared, bits are reconfigured
--------------------------------------------------------*/
void LCD_busy()
{
	unsigned char busy_bit;

	LCD_line = 0x00;

	DDRA = 0x77; // configure db7 as input
	LCD_line = 0x02; // R/W =1, RS = 0, EN =0
	LCD_line = 0x06; // R/W =1, RS = 0, EN =1
	
	busy_bit = PINA; // Read values from port C (data lines)

	while (busy_bit & 0x80){ // Check busy flag, bit 7
		busy_bit = PINA; // Read value again
	}

	// Set port configurations back
		DDRA = 0xF7; // All data lines are outputs
		LCD_line = 0x00; // R/W =0, RS = 0, EN =0

}

/* -----------------------------------------------------
	This function intializes the LCD screen
--------------------------------------------------------*/
void LCD_initialize()
{
	// Function set
	_delay_ms(200); // Delay following power on of LCD
	
	LCD_line = (LCD_line & (~DB_MASK))|(3<<4);
	_delay_us(LCD_DUR_US);
	LCD_line = (LCD_line & (~EN_MASK))|(1<<LCD_EN);
	_delay_us(LCD_DUR_US);
	LCD_line = (LCD_line & (~EN_MASK))|(0<<LCD_EN); // enable low
	
	_delay_ms(5);

	LCD_line = (LCD_line & (~DB_MASK))|(3<<4);
	_delay_us(LCD_DUR_US);
	LCD_line = (LCD_line & (~EN_MASK))|(1<<LCD_EN);
	_delay_us(LCD_DUR_US);
	LCD_line = (LCD_line & (~EN_MASK))|(0<<LCD_EN); // enable low

	_delay_us(200);


	LCD_line = (LCD_line & (~DB_MASK))|(3<<4);
	_delay_us(LCD_DUR_US);
	LCD_line = (LCD_line & (~EN_MASK))|(1<<LCD_EN);
	_delay_us(LCD_DUR_US);
	LCD_line = (LCD_line & (~EN_MASK))|(0<<LCD_EN); // enable low

	_delay_us(200);
		
		// Function set - High byte
	LCD_line = (LCD_line & (~DB_MASK))|(2<<4); // 2 Line, 5x8 dots
	_delay_us(LCD_DUR_US);
	LCD_line = (LCD_line & (~EN_MASK))|(1<<LCD_EN);
	_delay_us(LCD_DUR_US);
	LCD_line = (LCD_line & (~EN_MASK))|(0<<LCD_EN); // enable low
	_delay_ms(5);
	LCD_busy();

	// Function set - High byte
	LCD_line = (LCD_line & (~DB_MASK))|(2<<4); // 2 Line, 5x8 dots
	_delay_us(LCD_DUR_US);
	LCD_line = (LCD_line & (~EN_MASK))|(1<<LCD_EN);
	_delay_us(LCD_DUR_US);
	LCD_line = (LCD_line & (~EN_MASK))|(0<<LCD_EN); // enable low
	_delay_ms(5);
	LCD_busy();

	// Function set - Low byte
	LCD_line = (LCD_line & (~DB_MASK))|(8<<4); // 2 Line, 5x8 dots
	_delay_us(LCD_DUR_US);
	LCD_line = (LCD_line & (~EN_MASK))|(1<<LCD_EN);
	_delay_us(LCD_DUR_US);
	LCD_line = (LCD_line & (~EN_MASK))|(0<<LCD_EN); // enable low
	_delay_ms(5);
	LCD_busy();

	// Display control - High byte
	// Display on, cursor on, blink on
	LCD_line = (LCD_line & (~DB_MASK))|(0<<4);
	_delay_us(LCD_DUR_US);
	LCD_line = (LCD_line & (~EN_MASK))|(1<<LCD_EN);
	_delay_us(LCD_DUR_US);
	LCD_line = (LCD_line & (~EN_MASK))|(0<<LCD_EN); // enable low
	_delay_ms(5);
	LCD_busy();
	
	// Display control - Low byte
	// Display on, cursor on, blink on
	LCD_line = (LCD_line & (~DB_MASK))|(15<<4);
	_delay_us(LCD_DUR_US);
	LCD_line = (LCD_line & (~EN_MASK))|(1<<LCD_EN);
	_delay_us(LCD_DUR_US);
	LCD_line = (LCD_line & (~EN_MASK))|(0<<LCD_EN); // enable low
	_delay_ms(5);
	LCD_busy();

	// Clear display - High byte
	LCD_line = (LCD_line & (~DB_MASK))|(0<<4); 
	_delay_us(LCD_DUR_US);
	LCD_line = (LCD_line & (~EN_MASK))|(1<<LCD_EN);
	_delay_us(LCD_DUR_US);
	LCD_line = (LCD_line & (~EN_MASK))|(0<<LCD_EN); // enable low
	_delay_ms(5);
	LCD_busy();

		// Clear display - Low byte
	LCD_line = (LCD_line & (~DB_MASK))|(1<<4); 
	_delay_us(LCD_DUR_US);
	LCD_line = (LCD_line & (~EN_MASK))|(1<<LCD_EN);
	_delay_us(LCD_DUR_US);
	LCD_line = (LCD_line & (~EN_MASK))|(0<<LCD_EN); // enable low
	_delay_ms(5);
	LCD_busy();

	
	// Entry mode set
	LCD_line = (LCD_line & (~DB_MASK))|(0<<4); // Increment on, shift off
	_delay_us(LCD_DUR_US);
	LCD_line = (LCD_line & (~EN_MASK))|(1<<LCD_EN);
	_delay_us(LCD_DUR_US);
	LCD_line = (LCD_line & (~EN_MASK))|(0<<LCD_EN); // enable low
	_delay_ms(5);
	LCD_busy();

		// Entry mode set
	LCD_line = (LCD_line & (~DB_MASK))|(6<<4); // Increment on, shift off
	_delay_us(LCD_DUR_US);
	LCD_line = (LCD_line & (~EN_MASK))|(1<<LCD_EN);
	_delay_us(LCD_DUR_US);
	LCD_line = (LCD_line & (~EN_MASK))|(0<<LCD_EN); // enable low
	_delay_ms(5);
	LCD_busy();
	
}

/* -----------------------------------------------------
	This function displays a string sent to it
	It does not split lines
	Line 3 is continuation of line 1, line 4 of line 2
--------------------------------------------------------*/
void display_String(char* word)
{
	int i = 0;
	int word_length = 0;
	unsigned char letter;

	word_length = strlen(word);
	LCD_line = 0x00;
	_delay_us(LCD_DUR_US);

	for (i=0; i < word_length; i++){
		letter = word[i];
		LCD_line = (1<< LCD_RS);
		_delay_us(200);
		LCD_line = (LCD_line & (~DB_MASK))|(letter & 0xF0);
		_delay_us(LCD_DUR_US);
		LCD_line = (LCD_line & (~EN_MASK))|(1<<LCD_EN);
		_delay_us(LCD_DUR_US);
		LCD_line = (LCD_line & (~EN_MASK))|(0<<LCD_EN);
		_delay_us(200);

		LCD_line = (LCD_line & (~DB_MASK))|((letter & 0x0F) << 4);
		_delay_us(LCD_DUR_US);
		LCD_line = (LCD_line & (~EN_MASK))|(1<<LCD_EN);
		_delay_us(LCD_DUR_US);
		LCD_line = (LCD_line & (~EN_MASK))|(0<<LCD_EN);
		_delay_us(200);
		LCD_busy();
	}

}

void LCD_changeline(int num)
{	
	int ddram_address;
	// Change cursor address
	if (num == 1){
		ddram_address = 0x80; //0x00
	}
	else if (num == 2){
		ddram_address = 0xC0; //0x40
	}
	else if (num == 3){
		ddram_address = 0x94; //0x14
	}
	else if (num == 4){
		ddram_address = 0xD4; //0x54
	}
	else{
		display_String("Invalid command");	
	}
	LCD_line = 0x00;
	
	LCD_line = (LCD_line & (~DB_MASK))|(ddram_address & 0xF0);
	LCD_line = (LCD_line & (~EN_MASK))|(1<<LCD_EN);
	_delay_us(LCD_DUR_US);
	LCD_line = (LCD_line & (~EN_MASK))|(0<<LCD_EN);
	_delay_us(200);

	LCD_line = (LCD_line & (~DB_MASK))|((ddram_address & 0x0F) << 4);
	LCD_line = (LCD_line & (~EN_MASK))|(1<<LCD_EN);
	_delay_us(LCD_DUR_US);
	LCD_line = (LCD_line & (~EN_MASK))|(0<<LCD_EN);
	LCD_busy();

}

