#define LEFT 12  
#define RIGHT 13
int detectionL = HIGH;    // no obstacle on left
int detectionR = HIGH;    // no obstacle on right

void setup() {
  Serial.begin(9600);   
  pinMode(LEFT, INPUT); 
  pinMode(RIGHT, INPUT);
}

void loop() {  
  detectionL = digitalRead(LEFT);
  detectionR = digitalRead(RIGHT);
  if(detectionL == LOW && detectionR == HIGH){
    Serial.print("Left obstacle detected!\n");
  }
  else if (detectionR == LOW && detectionL == HIGH){ 
    Serial.print("Right obstacle detected!\n");
  }
  else if (detectionR == HIGH && detectionL == HIGH) {
    Serial.print("No obstacle detected!\n");
  }
  else {
    Serial.print("Obstacles detected on both sides!\n");
  }
  delay(500);    // in ms
}
