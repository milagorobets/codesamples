#include <asf.h>
#include "pwm_motors.h"

// Motor speed variables
extern volatile float motor1_speed;
extern volatile float motor2_speed;
extern volatile float motor3_speed;
extern volatile float motor4_speed;

extern volatile avr32_tc_t	 *tc0;	// PWM timer
extern volatile avr32_tc_t	 *tc1;	// PWM timer

void initialize_PWM(){
	
	// CHANNEL WAVEFORM CONFIGURATIONS
	tc_waveform_opt_t waveform_opt_motor1 = {
		.channel	= PWM_MOTOR1_CHANNEL,			// Specify channel
		
		// TIOB
		.bswtrg		= TC_EVT_EFFECT_NOOP,			// Software trigger effect
		.beevt		= TC_EVT_EFFECT_NOOP,			// External event effect
		.bcpc		= TC_EVT_EFFECT_NOOP,			// RC compare effect
		.bcpb		= TC_EVT_EFFECT_NOOP,			// RB compare effect
		
		// TIOA
		.aswtrg		= TC_EVT_EFFECT_NOOP,			// Software trigger effect
		.aeevt		= TC_EVT_EFFECT_NOOP,			// External event effect
		.acpc		= TC_EVT_EFFECT_CLEAR,			// RC compare effect: CLEAR output
		.acpa		= TC_EVT_EFFECT_SET,			// RA compare effect: SET output

		.wavsel		= TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,	// Waveform selection - auto trigger, reset on RC
		.enetrg		= false,						// External event trigger enable
		.eevt		= 1,							// External event selection - has to be non zero for B outputs
		.eevtedg	= TC_SEL_NO_EDGE,				// External event edge selection
		.cpcdis		= false,						// Counter disable when RC compare
		.cpcstop	= false,						// Counter clock stopped with RC compare
		
		.burst		= TC_BURST_NOT_GATED,			// Burst signal selection
		.clki		= TC_CLOCK_RISING_EDGE,			// Clock inversion
		.tcclks		= TC_CLOCK_SOURCE_TC3			// Internal source clock 3, connected to FPBA/8
	};
	tc_waveform_opt_t waveform_opt_motor2 = {
		.channel	= PWM_MOTOR2_CHANNEL,			// Specify channel
		
		// TIOB
		.bswtrg		= TC_EVT_EFFECT_NOOP,			// Software trigger effect
		.beevt		= TC_EVT_EFFECT_NOOP,			// External event effect
		.bcpc		= TC_EVT_EFFECT_CLEAR,			// RC compare effect: CLEAR output
		.bcpb		= TC_EVT_EFFECT_SET,			// RB compare effect: SET output
		
		// TIOA
		.aswtrg		= TC_EVT_EFFECT_NOOP,			// Software trigger effect
		.aeevt		= TC_EVT_EFFECT_NOOP,			// External event effect
		.acpc		= TC_EVT_EFFECT_NOOP,			// RC compare effect
		.acpa		= TC_EVT_EFFECT_NOOP,			// RA compare effect

		.wavsel		= TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,	// Waveform selection - auto trigger, reset on RC
		.enetrg		= false,						// External event trigger enable
		.eevt		= 1,							// External event selection - has to be non zero for B outputs
		.eevtedg	= TC_SEL_NO_EDGE,				// External event edge selection
		.cpcdis		= false,						// Counter disable when RC compare
		.cpcstop	= false,						// Counter clock stopped with RC compare
		
		.burst		= TC_BURST_NOT_GATED,			// Burst signal selection
		.clki		= TC_CLOCK_RISING_EDGE,			// Clock inversion
		.tcclks		= TC_CLOCK_SOURCE_TC3			// Internal source clock 3, connected to FPBA/8
	};
	tc_waveform_opt_t waveform_opt_motor3 = {
		.channel	= PWM_MOTOR3_CHANNEL,			// Specify channel
		
		// TIOB
		.bswtrg		= TC_EVT_EFFECT_NOOP,			// Software trigger effect
		.beevt		= TC_EVT_EFFECT_NOOP,			// External event effect
		.bcpc		= TC_EVT_EFFECT_CLEAR,			// RC compare effect: CLEAR output
		.bcpb		= TC_EVT_EFFECT_SET,			// RB compare effect: SET output
		
		// TIOA
		.aswtrg		= TC_EVT_EFFECT_NOOP,			// Software trigger effect
		.aeevt		= TC_EVT_EFFECT_NOOP,			// External event effect
		.acpc		= TC_EVT_EFFECT_NOOP,			// RC compare effect
		.acpa		= TC_EVT_EFFECT_NOOP,			// RA compare effect

		.wavsel		= TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,	// Waveform selection - auto trigger, reset on RC
		.enetrg		= false,						// External event trigger enable
		.eevt		= 1,							// External event selection - has to be non zero for B outputs
		.eevtedg	= TC_SEL_NO_EDGE,				// External event edge selection
		.cpcdis		= false,						// Counter disable when RC compare
		.cpcstop	= false,						// Counter clock stopped with RC compare
		
		.burst		= TC_BURST_NOT_GATED,			// Burst signal selection
		.clki		= TC_CLOCK_RISING_EDGE,			// Clock inversion
		.tcclks		= TC_CLOCK_SOURCE_TC3			// Internal source clock 3, connected to FPBA/8
	};
	tc_waveform_opt_t waveform_opt_motor4 = {
		.channel	= PWM_MOTOR4_CHANNEL,			// Specify channel
		
		// TIOB
		.bswtrg		= TC_EVT_EFFECT_NOOP,			// Software trigger effect
		.beevt		= TC_EVT_EFFECT_NOOP,			// External event effect
		.bcpc		= TC_EVT_EFFECT_CLEAR,			// RC compare effect: CLEAR output
		.bcpb		= TC_EVT_EFFECT_SET,			// RB compare effect: SET output
		
		// TIOA
		.aswtrg		= TC_EVT_EFFECT_NOOP,			// Software trigger effect
		.aeevt		= TC_EVT_EFFECT_NOOP,			// External event effect
		.acpc		= TC_EVT_EFFECT_NOOP,			// RC compare effect
		.acpa		= TC_EVT_EFFECT_NOOP,			// RA compare effect

		.wavsel		= TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,	// Waveform selection - auto trigger, reset on RC
		.enetrg		= false,						// External event trigger enable
		.eevt		= 1,							// External event selection - has to be non zero for B outputs
		.eevtedg	= TC_SEL_NO_EDGE,				// External event edge selection
		.cpcdis		= false,						// Counter disable when RC compare
		.cpcstop	= false,						// Counter clock stopped with RC compare
		
		.burst		= TC_BURST_NOT_GATED,			// Burst signal selection
		.clki		= TC_CLOCK_RISING_EDGE,			// Clock inversion
		.tcclks		= TC_CLOCK_SOURCE_TC3			// Internal source clock 3, connected to FPBA/8
	};
	
	
	// ENABLE PINS AND FUNCTIONS
	gpio_enable_module_pin(PWM_MOTOR1_PIN, PWM_MOTOR1_FUNCTION);
	gpio_enable_module_pin(PWM_MOTOR2_PIN, PWM_MOTOR2_FUNCTION);
	gpio_enable_module_pin(PWM_MOTOR3_PIN, PWM_MOTOR3_FUNCTION);
	gpio_enable_module_pin(PWM_MOTOR4_PIN, PWM_MOTOR4_FUNCTION);

	// INITIALIZE WAVEFORMS
	tc_init_waveform(tc1, &waveform_opt_motor1);
	tc_init_waveform(tc0, &waveform_opt_motor2);
	tc_init_waveform(tc1, &waveform_opt_motor3);
	tc_init_waveform(tc1, &waveform_opt_motor4);

	// INPUT INITIAL VALUES
	// Motor 1
	tc_write_rc(tc1, PWM_MOTOR1_CHANNEL, TC_PERIOD);		// Set RC value
	tc_write_ra(tc1, PWM_MOTOR1_CHANNEL, (TC_PERIOD - TC_MULTIPLE));	// Set 5% duty cycle
	// tc_write_ra(tc1, PWM_MOTOR1_CHANNEL, (TC_MULTIPLE));	// Set 5% duty cycle
	// Motor 2
	tc_write_rc(tc0, PWM_MOTOR2_CHANNEL, TC_PERIOD);		// Set RC value
	tc_write_rb(tc0, PWM_MOTOR2_CHANNEL, (TC_PERIOD - TC_MULTIPLE));	// Set 5% duty cycle
	//tc_write_rb(tc0, PWM_MOTOR2_CHANNEL, (TC_MULTIPLE));	// Set 5% duty cycle
	// Motor 3
	tc_write_rc(tc1, PWM_MOTOR3_CHANNEL, TC_PERIOD);		// Set RC value
	tc_write_rb(tc1, PWM_MOTOR3_CHANNEL, (TC_PERIOD - TC_MULTIPLE));	// Set 5% duty cycle
	//tc_write_rb(tc1, PWM_MOTOR3_CHANNEL, (TC_MULTIPLE));	// Set 5% duty cycle
	// Motor 4
	tc_write_rc(tc1, PWM_MOTOR4_CHANNEL, TC_PERIOD);		// Set RC value
	tc_write_rb(tc1, PWM_MOTOR4_CHANNEL,(TC_PERIOD - TC_MULTIPLE));	// Set 5% duty cycle
	//tc_write_rb(tc1, PWM_MOTOR4_CHANNEL,(TC_MULTIPLE));	// Set 5% duty cycle
}


void update_Motors()
{
	tc_write_rc(tc1, PWM_MOTOR1_CHANNEL, TC_PERIOD);		// Set RC value
	tc_write_rc(tc0, PWM_MOTOR2_CHANNEL, TC_PERIOD);		// Set RC value
	tc_write_rc(tc1, PWM_MOTOR3_CHANNEL, TC_PERIOD);		// Set RC value
	tc_write_rc(tc1, PWM_MOTOR4_CHANNEL, TC_PERIOD);		// Set RC value
	
	tc_write_ra(tc1, PWM_MOTOR1_CHANNEL, (int)(TC_PERIOD - TC_MULTIPLE*(motor1_speed*0.01 + 1)));
	tc_write_rb(tc0, PWM_MOTOR2_CHANNEL, (int)(TC_PERIOD - TC_MULTIPLE*(motor2_speed*0.01 + 1)));
	tc_write_rb(tc1, PWM_MOTOR3_CHANNEL, (int)(TC_PERIOD - TC_MULTIPLE*(motor3_speed*0.01 + 1)));
	tc_write_rb(tc1, PWM_MOTOR4_CHANNEL, (int)(TC_PERIOD - TC_MULTIPLE*(motor4_speed*0.01 + 1)));
	/*
	tc_write_ra(tc1, PWM_MOTOR1_CHANNEL, (int)(TC_MULTIPLE*(motor1_speed*0.01 + 1)));
	tc_write_rb(tc0, PWM_MOTOR2_CHANNEL, (int)(TC_MULTIPLE*(motor2_speed*0.01 + 1)));
	tc_write_rb(tc1, PWM_MOTOR3_CHANNEL, (int)(TC_MULTIPLE*(motor3_speed*0.01 + 1)));
	tc_write_rb(tc1, PWM_MOTOR4_CHANNEL, (int)(TC_MULTIPLE*(motor4_speed*0.01 + 1)));
	*/
}