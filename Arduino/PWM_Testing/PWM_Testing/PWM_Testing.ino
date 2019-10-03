#include "Servo.h"

int inPin = 5;
int outPin = 3;
Servo esc;

//range from 1200 to 1930
//Total Pwm time is 21480

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(outPin, OUTPUT);
  esc.attach(6);
  esc.write(50);
}

void loop() {
  // put your main code here, to run repeatedly:
  val = pulseIn(inPin, HIGH);
  Serial.print("Val: ");
  Serial.println(val);
  delay(100);

}
