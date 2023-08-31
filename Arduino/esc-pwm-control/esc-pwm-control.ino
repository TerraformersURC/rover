#include <Servo.h>

byte motor1_pin = 9;
Servo motor1;

void setup() {
  
 Serial.begin(115200);
 motor1.attach(motor1_pin);

 motor1.writeMicroseconds(1500); // send "stop" signal to ESC.

 delay(7000); // delay to allow the ESC to recognize the stopped signal
}

void loop() {
    
  while (Serial.available() == 0);
  
  int val = Serial.parseInt(); 
  Serial.println(val);
  
  if(val < 1100 || val > 1900)
  {
    Serial.println("not valid");
  }
  else
  {
    motor1.writeMicroseconds(val); // Send signal to ESC.
  }
}
