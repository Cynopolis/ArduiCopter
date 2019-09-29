#include <PID.h>

PID pidTest(1, 1, 1, 50, 0, 5);
int x = 0;
float output;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  output = pidTest.calcPID();
  Serial.print("Output: " + output);
  if(output > 5){
    x += 5;
  }
  else{
    x += output;
  }
  Serial.println("X: " + x);
}
