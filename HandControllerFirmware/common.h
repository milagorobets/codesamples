#define FOSC 20000000
#define F_CPU 20000000UL

#define TRUE 1
#define FALSE 0

#include <avr/wdt.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/atomic.h>
#include <stdint.h> 
#include <string.h>
#include <inttypes.h>
#include <util/delay.h>
#include <avr/sfr_defs.h>

#include "rprintf.h"
#include "lcd_control_fb.h"
#include "buffer.h"
#include "uart.h"
#include "ServoControl.h"
#include "calculations.h"



