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

// UDRIE mask. Use this to enable/disable
// the UDRE interrupt
#define UDRIEMASK   0b00100000

static volatile char flashWhich=1;
static volatile char buttonVal=1;

char dataRecv, dataSend;

// Completes receiving 
ISR( USART_RX_vect )
{
  // Write received data to dataRecv
  dataRecv = UDR0;
  sendData(dataRecv); // Sending the character back
  if (dataRecv == 'F') {
    
    flashWhich = 1 - flashWhich;
    UCSR0B |= 0b00100000;
  }else if (dataRecv == 'B') {
    
    flashWhich = 1- flashWhich;
    UCSR0B |= 0b00100000;
  }
  
}

// Complete sending
ISR( USART_UDRE_vect )
{
  // Write dataSend to UDR0
  // Disable UDRE interrupt
  //dataSend = 'W';
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
  OCR0A = 128;
}

ISR(TIMER0_COMPB_vect) {
  OCR0B = 128;
}

ISR(TIMER2_COMPA_vect) {
  OCR2A = 128;
}

ISR(TIMER2_COMPB_vect) {
  OCR2B = 128;
}

void right_motor_forward(void) {
  TCCR0A = 0b10000001;
  PORTD &= 0b1101111;
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

// Enable external interrupts 0 and 1
void setupEINT()
{
  // Configure INT0 and INT1 for rising edge triggered.
  // Remember to enable INT0 and INT1 interrupts.
}


// ISRs for external interrupts
ISR(INT0_vect)
{
  //buttonVal=1;  
  //sendData(buttonVal);
  flashRed();
}

ISR(INT1_vect)
{
  //buttonVal=2;
  //sendData(buttonVal);
}


// Red is on pin 12
void flashRed()
{
    PORTB |= PIN12MASK;
    delay(100);
    PORTB &= ~PIN12MASK;
    delay(500);
}

// Green is on pin 11
void flashGreen()
{
    PORTB |= PIN11MASK;
    delay(100);
    PORTB &= ~PIN11MASK;
    delay(500);
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
  setupEINT();
  setupSerial();
  setBaud(9600);
  startSerial();
  sei();
}


void loop() {
    if (!flashWhich) {
        right_motor_reverse();
        left_motor_reverse();
    }else {
        right_motor_forward();
        left_motor_forward();
    }
}
