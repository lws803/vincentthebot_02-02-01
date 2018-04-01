int encoderValue_A = 0; // LEFT ENCODER TICKS
int encoderValue_B = 0; // RIGHT ENCODER TICKS

ISR(INT0_vect) {
	encoderValue_A++;
	Serial.print("LEFT TICKS: ");
	Serial.println(encoderValue_A);
}

ISR(INT1_vect) {
	encoderValue_B++;
	Serial.print("RIGHT TICKS: ");
	Serial.println(encoderValue_B);
}

void setupEINT() {
	// Activate INT0 and INT1
	EIMSK |= 0b00000011;
	// Set to FALLING EDGE read
	EICRA |= 0b00001010;
	
	// Enable pull up resistors at respective pins
	DDRD &= 0b11110011;
	PIND |= 0b00001100;
}

void setup() {

	Serial.begin(9600);
	
	setupEINT();
}

void loop() {
	// nothing
}
