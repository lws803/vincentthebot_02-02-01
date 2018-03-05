// ------------------------ Turning on and off LED  ------------------------

void switchLED () {
    if (!turn) {
        PORTB &= 0b11101111;
    }else {
        // Switch LED at pin 12 on. Pin 12 is PB4
        PORTB |= 0b00010000;
    }
}

// ------------------------ Setting up ISR for INT 0 ------------------------

ISR(INT0_vect)
{
    // Modify this ISR to do anything you want
    // Upon hardware activation, this function will be called 
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
     	// Replace with whatever you wish here   
    }
}


// ------------------------ Setting up Timer ---------------------------------

// The turn variable decides whose turn it is to go now.
volatile int turn=0;

// The _timerTicks variable maintains a count of how many x ticks have passed by.
static unsigned long _timerTicks = 0;

// ISR for timer 2
ISR(TIMER2_COMPA_vect) {
    // This interrupt will happen whenever TCNT reaches 256
    _timerTicks++; // Global variable to count on each timer interrupts
    if (_timerTicks == 251) {
        turn = 1 - turn;
        _timerTicks = 0;
    }
}

// Timer set up function.
void setupTimer()
{
    // Set timer 2 to produce 100 microsecond (100us) ticks
    // But do no start the timer here.
    //Set Initial Timer value
    TCNT2=0; // initialise at 0
    
    //Place TOP timer value to Output compare register
    OCR2A=15625; // Set this to the V value

	/*
		res = 1/(F_clock/prescaler)

		V = period/res
    */

    //Set CTC mode
    //    and make toggle PD6/OC0A pin on compare match
    TCCR2A=0b10000001;
    // Enable interrupts.
    TIMSK2|=0b010;
}

void startTimer()
{
    // Start timer 2 here.
    //Set prescaler 256 and start timer
    TCCR2B=0b00000110;
    // Enable global interrupts
    sei();
}


int main(void)
{
    // Set up timer 2
    setupTimer();
    
    // Start timer 2
    startTimer();
    
    // Ensure that interrupts are enabled
    sei();
    
    /* Replace with your application code */
    while (1)
    {
    	// Fill in whatever you wish whilst using turn to control your functions
    }
}

