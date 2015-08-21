///////////////////////////////////////////////////////////////
//   					SERVO CONTROL 						 //
// Executes control of 9 servos 							 //
// Uses interrupts to minimize the use of processor time     //
// 															 //
// For ATmega 1248P											 //
// Frequency: 20 MHz										 //
// Written by: Mila Gorobets, June 2012						 //
//  														 //
///////////////////////////////////////////////////////////////

#include "common.h"


// ~~~~~~ For Servo Controller
// Current servo line (first line of pair)
volatile int current_line;
// Counts for individual servos past the initial 1ms; Servo 10 is off

// Keeps track of overflows when counting out rest of 20ms
volatile int overflow_count;
// Masks for updating channels
volatile uint8_t maskA, maskB, maskC, maskD;

uint8_t masktbl[CHANNELS] = {     // look up the mask for each channel
 (1<<0), (1<<1), (1<<2), (1<<3), (1<<4), (1<<5), (1<<6), (1<<7),    // PC0..PC7
 												         (1<<7),     // PD7
														 (0<<6)		// Low value for spare servo spot
};


/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   INTERRUPT SERVICE ROUTINE - Timer 1 Comparator A
   - Clears servo pin for Comp A
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
ISR(TIMER1_COMPA_vect){
	CLR_PIN(current_line-2, maskA);
}


/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   INTERRUPT SERVICE ROUTINE - Timer 1 Comparator B
   - Clears servo pin for Comp B
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
ISR(TIMER1_COMPB_vect){
	CLR_PIN(current_line-2, maskB);
}


/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   INTERRUPT SERVICE ROUTINE - Timer 1 Overflow
   - Counts 1 ms intervals from set value to overflow
   - Controls paired, staggered lines
   - 
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
ISR(TIMER1_OVF_vect){

	if  (current_line > 8){
		// Counting rest of 20 ms period
		overflow_count++;
		TCNT1 = 15537;
		if (overflow_count == 3){
			current_line = 0; // Reset servo counter
			overflow_count = 0;
			}
		return;
	}

	maskA = masktbl[current_line];
	maskB = masktbl[current_line+1];
	OCR1A = 35536 + servo_table[current_line];
	OCR1B = 35536 + servo_table[current_line + 1];
	TCNT1 = 15537; // 2.5ms until overflow
	if (current_line == 8){OCR1B = 10000;}
	SET_PIN(current_line,maskA);
	SET_PIN(current_line + 1, maskB);

	current_line += 2;
}


/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   INITIALIZATION ROUTINE - Servos
   - Turns on servo ports
   - Enables Timer 1 interrupts
   - Initializes Timer 1
   - Starts initial counter to reach first interrupt routine
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void initialize_Servos()
{	
	current_line = -4;
  	
  	SERVO_PINS_L1 |= SERVO_ALL_C; // turn on proper ports
	SERVO_PINS_L2 |= SERVO_ALL_D; // Servo 9

	TCNT1 = 0;
  	TCCR1A = (0<<WGM11);
  	TCCR1B = (0<<WGM13)|(0<<WGM12)|(1<<CS10);  // mode#0, div8 prescalar
  	OCR1A = 40000;
  	OCR1B = 40000;
	TIFR1 = (1 << OCF1A)|(1 << OCF1B)|(1<< TOV1);
  	TIMSK1 = (1<<OCIE1B)|(1<<OCIE1A)|(1<<TOIE1);
  
   	TCNT1 = 15537; // 2.5 ms to do other initializations
} 
