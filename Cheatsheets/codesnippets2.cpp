// Serial communications 

// TODO: Add serial polling and interrupt implementation 



// Serialising 

u_char buf[100]; // buffer to store incoming/ outgoing data

unsigned int PACKET_OK = 0, PACKET_BAD_CHECKSUM = 1;



unsigned int serialize(void *p, size_t size)
{
    char checksum = 0;
    buf[0]=size; // 0th index will be used to store the size 

    // memcpy everything from p to buf starting from the 1st index 
    memcpy(buf+1, p, size);

    // Forming the checksum
    for(int i=1; i<=size; i++)
    {
        checksum ^= buf[i];
    }
    cout << "Checksum: " << (int)checksum << endl;
    buf[size+1]=checksum; // Last packet will be the checksum
    return size+2;
}

void sendSerialData(u_char *buffer, int len)
{
    for(int i=0; i<len; i++)
        cout << (int)buffer[i] << " ";
    cout << endl;
}


unsigned int deserialize(void *p, u_char *buf)
{
    size_t size = buf[0]; // get size from 0th index 
    char checksum = 0;
    
    for(int i=1; i<=size; i++)
        checksum ^= buf[i];
    
    cout << "Checksum: " << (int)checksum << endl;
    
    if(checksum == buf[size+1]) // Compare the checksum and see if it meets the requirements 
    {
    	// if yes, memcpy everything except checksum into local buffer. 
        memcpy(p, buf+1, size); 
        return PACKET_OK;
    }
    else
    {
        printf("CHECKSUM ERROR\n");
        return PACKET_BAD_CHECKSUM;
    }
}



// ADC using interrupts 
// atmega328p has 8 10-bit adc which uses successive approximation to get the data w.r.t ref voltage

// TODO: Fill up more info regarding ADC stuff from the datasheet 

unsigned adcvalue;

void ledToggle()
{
	PORTB ^= 0b00100000;
}

// Possible implementation using PWM 

void InitPWM()
{
	TCNT0 = 0;
	OCR0A = 0;
	TCCR0A = 0b10000001;
	TIMSK0 |= 0b10;
}

void startPWM()
{
	TCCR0B = 0b00000011;
	sei();
}

ISR(TIMER0_COMPA_vect)
{
	OCR0A = adcvalue; // connect LED to pin 6 
}


int main(void)
{
    unsigned loval, hival;
	
	PRR &= 0b11111110; // Turn off power reserve to the ADC
	ADCSRA = 0b10000111; // Switch on ADC with the division factor
	ADMUX = 0b01000000; // Choose reference voltage
	DDRB |= 0b00100000; // setup LED
	
    while (1) 
    {
		ADCSRA |= 0b01000000;
		
		while(ADCSRA & 0b01000000); // wait for conversion to complete, only required for polling
		
		ADCSRA |= 0b00010000; // Clearing the ADIF 

		// Reading the result

		loval = ADCL; // Always read from ADCL first
		hival = ADCH;
		adcvalue = hival * 256 + loval;
		
		ledToggle();
		_delay_loop_2(adcvalue);
    }
}

// Interrupt imlpementation 

int adcvalue; // changed this to int as char will overflow above 255
unsigned loval, hival;

void ledOn()
{
	PORTB |= 0b00100000;	
}

void ledOff()
{
	PORTB &= 0b11011111;
}

// ADC Conversion Complete Interrupt Service Routine (ISR)
ISR(ADC_vect)
{
	loval = ADCL; // Always read from ADCL first
	hival = ADCH;
	adcvalue = hival * 256 + loval;
	ADCSRA |= 0b00010000; // Clearing the ADIF 

	ADCSRA |= 0b01000000; // possible method to start the conversion again 
}



int main(void)
{
	PRR &= 0b11111110;
	ADCSRA = 0b10001111;
	ADMUX = 0b01000000;
	DDRB |= 0b00100000;
	
	sei(); // enable global interrupts, dont have to use mask after this
	ADCSRA |= 0b01000000; // start conversion 
	
    while (1) 
    {
    	// Max adcvalue = 670
		ledOn();
		_delay_loop_2(adcvalue);
		ledOff();
		_delay_loop_2(670);
    }
}

