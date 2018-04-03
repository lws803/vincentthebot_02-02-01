#include<math.h>
#include<Wire.h>
#define Addr 0x0E

float heading;
void setup() {
	Wire.begin();
	Serial.begin(9600);

	Wire.beginTransmission(Addr);
	Wire.write(0x10);
	Wire.write(0x01);
	Wire.endTransmission();
}

void loop()  {
	int x, y, z;
	//Tell the MAG3110 where to begin reading data
	Wire.beginTransmission(MAG_address);
	Wire.write((byte)0x01); //select register 1
	Wire.endTransmission();
	delay(2);
	//Read data from each axis, 2 registers per axis
	Wire.requestFrom(MAG_address, 6);
	if (Wire.available() == 6){
		x = Wire.read() << 8; //X msb
		x |= Wire.read();     //X lsb
		y = Wire.read() << 8; //Y msb
		y |= Wire.read();     //Y lsb
		z = Wire.read() << 8; //Z msb
		z |= Wire.read();     //Z lsb
	} else { x=0; y=0; z=0; }

	heading = atan2((float)y, (float)x);
	delay(100);
	Serial.println(heading);
	delay(1000);
}
