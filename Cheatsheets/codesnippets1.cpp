// ------------------------ Turning on and off LED  ------------------------
/*
	General rule:
	Set high = use |= ____1_____ 
	Set low =  use &= ____0_____
	For checking conditions: We usually use & operator 

	Bit-shifting:
	eg. x = 0xFA
	x_high = (x & 0xF0) >> 4 // & to remove the lower byte 
	x_low = (x & 0x0F) // & to remove the uppper byte
*/
void switchLED () {
    if (!turn) {
        PORTB &= 0b11101111;
    }else {
        // Switch LED at pin 12 on. Pin 12 is PB4
        PORTB |= 0b00010000; // Assuming this is active high pin, where logic 1 will turn it ON
    }
}

void toggleLED () {
	PORTB ^= 0b00010000;
}

void conditions() {
	// To check if an input pin is currently high or low
	if (PORTB & 0b00100000)
		// Pin is currently high 

	if (!(PORTB & 0b00100000))
		// Pin is currently low 
}

int main () {
	DDRB |= 0b00001000; // Set pin to output 
	DDRD &= 0b00100000; // Set pin to input, make sure we do this to set the pin to input mode

    // 0 = input, 1 = output
    
    PIND |= 0b00100000; // Will set the output pin to active high, meaning it will register when pulled down 
	// Note: For output or input even if its active high or low, we do not care. We will still set this according to
	// whether it is an input or output. 
	switchLED();
}

// ------------------------ Setting up ISR for INT 0 ------------------------
/*
	Legend:
	EIMSK: External interrupt mask register (Contains: INT0, INT1 - write 1 to enable them)
	EIFR: External interrupt flag register (Contains: INTF1 and INTF0 - write 1 to clear them)
		Similar to timer interrupts too for detecting compare match 
*/
ISR(INT0_vect)
{
    // Modify this ISR to do anything you want
}

// Nested interrupts 
// Nested interrupts cannot happen unless we renable global interrupts in the vector table itself 
ISR (INT1_vect) {
    /*
        Activate external events to trigger INT0 
    */
    // Interrupt will not occur unless we call 
    sei(); // enables global interrupts. 

}

int main(void)
{
	// Enable INT0
    EIMSK |= 0b00000001;

    // Set up EICRA
    // For falling edge on INT0, Bits 1 and 0 should be 10
    EICRA |= 0b00000011;
    
    // Set pin 12 to output
    DDRB |= 0b00001000;
        
    /* Replace with your application code */
    while (1)
    {
    }
}


// ------------------------ Setting up Timer ---------------------------------
/*
    >>>>>>>>>>>>>Timer 0: 8 bit<<<<<<<<<<<<
    >>>>>>>>>>>>>Timer 2: 8 bit<<<<<<<<<<<<

	    Legend: 
	    OCR: Output compare register (Usually your TOP value)
	    TCCR: Timer/Counter control register 
	    TCNT: Timer/Counter value
	    TIMSK: Timer interrupt mask register (Contains == OCIE, output compare match interrupt enable)
			Note: OCIE depends on TIFR to be set, then only interrupt vector will activate 

	    TIFR: Timer interrupt flag register (Contains == OCF: Output compare match flag - write 1 to clear them)
	    	Note: It does not automatically clear itself if interrupt is not enabled. 

	    - Warning: Do not write to TNCT while it is running, it will remove compare match
        Also do not modify TNCT as you run the risk of missing a compare match

	    - Num clock cycles required to process an operation: time_per_op/(1/clock_speed)
		
		- freq pulse (CTC): clock_speed/(2 * N * (OCR + 1))
		N is prescalar 

        - Rate at which TCNT is incremented = N/ clock_speed        where N is the prescalar
            because of clock division, it takes longer to increment TCNT by 1. it will take N * time_taken per cycle to make one increment 

	    - choosing a good prescaler value: V that does not exceed 255 and does not have a decimal (hopefully)

        - On CTC mode, even if we have 2 OCRs the register will be hit and cleared the instance it hits the first OCR even if they have different 
        prescalar value.     

    >>>>>>>>>>>>>Timer 1: 16 bit<<<<<<<<<<<<

    	Setting up of OCR we will have to use OCR_AL and OCR_AH, set to high and low byte registers
	
	>>>>>>>>>>>>>   Normal mode <<<<<<<<<<<<
	Clock will count all the way till overflow and then resets itself. - it will also trigger TOV
		Can be set to automatically TOV when timer interrupt is enabled. 
*/

// The turn variable decides whose turn it is to go now.
volatile int turn=0;

// The _timerTicks variable maintains a count of how many x ticks have passed by.
static unsigned long _timerTicks = 0;

// ISR for timer 2
ISR(TIMER2_COMPA_vect) {
    // This interrupt will happen whenever TCNT reaches 256
}

// Timer set up function.
void setupTimer()
{
    //Set Initial Timer value
    TCNT2=0; // initialise at 0

    //Place TOP timer value to Output compare register
    OCR2A=255; // Set this to the V value (cant go above 255)
	/*
		res = 1/(F_clock/prescaler)
			Note: prescalar acts as a clock divider 
		V = period/res

        if V > 255, choose a larger prescaler 
    */
    //Set CTC mode
    //    and make toggle PD6/OC0A pin on compare match
    // OC0A will only work if the DDR bit corresponding to that pin is set 
    TCCR2A=0b10000001;
    // Enable interrupts.
    TIMSK2|=0b010;
}

void startTimer()
{
    //Set prescaler 256 and start timer
    TCCR2B=0b00000110;
    // Enable global interrupts
    sei();
}


int main(void)
{
    setupTimer();
    startTimer();    
    while (1)
    {
    }
}

// ------------------------ Setting up PWM -----------------------------------
/*
    * Digital bits are encoded over a fixed period of time
    * Analog values are encoded by the proportion of "1" time to "0" time within this fixed time
    
		During fast PWM mode: TOP will be set to 255 (0xFF). Freq will be 
		
		freq of PWM (FAST): clock_speed/(N * 256)
		freq of PWM (PHASE CORRECT): clock_speed/(N * 510)
		N is prescalar 

	    eg. for 500Hz PWM, we get N = 62.74 --> Closest value of N is 64, set this for prescalar

	    The duty cycle is determined by the value stored in the OCR0A register
	    D = (0CR0A/255) * 100 
*/

void InitPWM()
{
    TCNT0 = 0; // To generate phase correct 
    OCR0A = 0;
    TCCR0A = 0b11000001; // This is to configure PWM phase correct mode rather than CTC mode
    // Recall: CTC mode basically calls the interrupt only when it is == OCR0A (TOP) and then resets it. 

    TIMSK0 |= 0b10;
}

void startPWM()
{
    TCCR0B = 0b00000011;
    sei();
}

ISR(TIMER0_COMPA_vect)
{
    OCR0A = 25; // The duty cycle is determined by the value stored in the OCR0A register
}
int main(void)
{
    DDRD |= (1 << DDD6); 
    InitPWM();
    startPWM();
    while (1)
    {
    }
}
