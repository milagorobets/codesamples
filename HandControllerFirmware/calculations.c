/*! \file calculations.c \Updates servo counts */
//*****************************************************************************
//
// File Name	: 'calculations.c'
// Title		: Updates servo counts
// Author		: Mila Gorobets
// Created		: 24/08/2012
// Revised		: 24/08/2012
// Version		: 1.0
// Target MCU	: any
//
//
//*****************************************************************************

#include "common.h"


/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   Update Servo Counters
   - Uses newly received data to update counts
   - Uses copy of data to prevent changes
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void update_counts(unsigned char* p)
{
	// p is pointer to beginning of entire array
	int c1 = 0;
	for (int i = 0; i < 9; i++)
	{
		char l_b = p[c1] - 32;
		c1++;
		char h_b = p[c1] - 32;
		servo_table[i] = ((h_b << 8) + l_b);
		c1++;
	}	
}
