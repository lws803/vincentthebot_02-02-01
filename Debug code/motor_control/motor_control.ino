#include <avr/io.h>
#include <avr/interrupt.h>

/*#ifndef F_CPU
#define F_CPU   16000000UL
#endif
*/
#include <util/delay.h>

// Masks for pins 12 and 11
#define PIN12MASK   0b00010000
#define PIN11MASK   0b00100000

// pin 6 and 5, 11 and 3

// UDRIE mask. Use this to enable/disable
// the UDRE interrupt
#define UDRIEMASK   0b00100000


char dataRecv, dataSend;

// volatile char currState = '\0';
volatile int startSpeed = 255;


// Completes receiving 
ISR( USART_RX_vect )
{
  // Write received data to dataRecv
  dataRecv = UDR0;
  sendData(dataRecv); // Sending the character back
  // sei(); //- uncomment this if it doesnt work 
  if (dataRecv == 'w') {
    // Nested interrupts - probably 
    startSpeed = 200;
    right_motor_forward(); // Will keep interrupting and calling move forward
    left_motor_forward();
  }else if (dataRecv == 's') {
    startSpeed = 200;
    right_motor_reverse();
    left_motor_reverse();        
  }
  else if (dataRecv == 'a') {
    // Turn left
    startSpeed = 200;
    right_motor_forward(); // Will keep interrupting and calling move forward
    left_motor_reverse();
  }
  else if (dataRecv == 'd') {
    startSpeed = 200;
    right_motor_reverse(); // Will keep interrupting and calling move forward
    left_motor_forward();
  }
  else if (dataRecv == 'q') {
    startSpeed = 0;
  }
  else if (dataRecv == 'r') {
    startSpeed = 200;
  }
  UCSR0B |= 0b00100000;
}

// Complete sending
ISR( USART_UDRE_vect )
{
  // Write dataSend to UDR0
  // Disable UDRE interrupt
  //UDR0 = dataSend;
  UCSR0B &= 0b11011111; // Reset UDRIE to 0 
  //flashRed();
  
}

void InitPWM () {
  TCNT0 = 0;
  OCR0A = 0;
  OCR0B = 0;
  TIMSK0 |= 0b110;

  TCNT2 = 0;
  OCR2A = 0;
  OCR2B = 0;
  TIMSK2 |= 0b110;
}

void startPWM() {
  TCCR0B = 0b0000011;
  TCCR2B = 0b0000100;
  //sei();
}

ISR(TIMER0_COMPA_vect) {
  OCR0A = startSpeed;
}

ISR(TIMER0_COMPB_vect) {
  OCR0B = startSpeed;
}

ISR(TIMER2_COMPA_vect) {
  OCR2A = startSpeed;
}

ISR(TIMER2_COMPB_vect) {
  OCR2B = startSpeed;
}

void right_motor_forward(void) {
  TCCR0A = 0b10000001;
  PORTD &= 0b11011111;
}

void right_motor_reverse(void) {
  TCCR0A = 0b00100001;
  PORTD &= 0b10111111;
}

void left_motor_forward(void) {
  TCCR2A = 0b10000001;
  PORTD &= 0b11011111;
}

void left_motor_reverse(void) {
  TCCR2A = 0b00100001;
  PORTD &= 0b11011111;
}


void stop_motor () {
  startSpeed = 0;
}

void sendData(const char data)
{
  // Copy data to be sent to dataSend
  //dataSend = buttonVal+'0';

  
  // Enable UDRE interrupt below

  while ((UCSR0A & 0b00100000) == 0); 
  UDR0 = data;
}

char recvData()
{
  unsigned char data;
  while ((UCSR0A & 0b10000000) == 0); 
  data = UDR0;

  
  return data;
}

void setupSerial()
{
  // Set up the 115200 8N1 when using
  // Serial Monitor to test
  UCSR0A = 0;
  UCSR0C = 0b00000110;
  
  
  // Change to 115200 7E1 when
  // communicating between Arduinos.
  
}

void startSerial()
{

  UCSR0B = 0b10011000;
  // Start the serial port.
  // Enable RXC interrupt, but NOT UDRIE
  // Remember to enable the receiver
  // and transmitter
}

// ISRs for external interrupts
ISR(INT0_vect)
{
  
}

ISR(INT1_vect)
{
  //buttonVal=2;
  //sendData(buttonVal);
}



void setBaud(unsigned long baudRate)
{
  unsigned int b;
  b = (unsigned int) round(F_CPU / (16.0 *
  baudRate))- 1;
  UBRR0H = (unsigned char) (b >> 8);
  UBRR0L = (unsigned char) b;
}


void setup() {

  cli();
  // put your setup code here, to run once:
  DDRD |= ((1 << DDD6) | (1 << DDD5) | (1 << DDD3));
  DDRB |= (1 << DDB3);

  InitPWM();
  startPWM();

  DDRB |= (PIN11MASK | PIN12MASK);
  setupSerial();
  setBaud(9600);
  startSerial();
  sei();
}


void loop() {
    //stop_motor(); // So that it will stop as long as user does not enter keypress. 
}

