#define SERVO_PINS_L1 DDRC
#define SERVO_PINS_L2 DDRD
#define SERVO_L1 PORTC
#define SERVO_L2 PORTD

#define ONE_MS      19999 // No prescaler
#define SERVO_ALL_C 0xFF
#define SERVO_ALL_D 0x80

#define SET_MASK(port, mask)      port |= (mask)
#define CLR_MASK(port, mask)      port &= ~(mask)
#define SET_PIN(n, mask)          {if (n<8) SET_MASK(PORTC,mask);else if (n<11) SET_MASK(PORTD,mask);}
#define CLR_PIN(n, mask)          {if (n<8) CLR_MASK(PORTC,mask);else if (n<11) CLR_MASK(PORTD,mask);}

#define CHANNELS 10

void initialize_Servos();

volatile int servo_table[CHANNELS];


