#include <AutoPID.h>
#include "IMU.h"
#include "Servo.h"

#define DT  0.02          // Loop time
#define G_GAIN 0.070    // [deg/s/LSB]

/*This stores the data recieved from the accelerometer,
 * Gyroscope, and compass in a 3x3 array.
 * Accelerometer is in IMUData[0] (g-force)
 * Gyroscope is in IMUData[1] (degrees)
 * Compass data is in IMUData[2] (raw compass data)
 * Unless stated otherwise ALL data is stored in the order of x, y, z
 */
boolean failSafe = false;
float IMUData[3][3];
//the filtered angles that combines the accel and gyro input (degrees)
float filteredData[3] = {0, 0, 0};
//Using servo objects to output data to ESCs
Servo ESC[4];
//pin numbers of the ESC pins in the order of FL, FR, BL, BR
int ESCPin[4] = {2,6,7,8};
/*The pins to be connected to the reciever
 * pitch, roll, yaw, height, switch1, switch2
 */
int RXPin[6] = {A0, A1, A2, A3, A4, A5};
float RXData[6] = {0,0,0,0,0,0};
//compass heading
float heading = 0;

//the tilt angles computed by the accelerometer data (degrees)
float accelAngles[3];
//the target angles the drone should be at (degrees)
float targetAngles[3] = {90, 90, 0}; //90, 90, 0 is level flight

//PID controllers, their k values, and their outputs
float kVals[3][3] = {{1,1,1}, {1,1,1}, {1,1,1}};
float PIDOutput[3] = {0, 0, 0};
AutoPID pitch(&filteredData[0], &targetAngles[0], &PIDOutput[0], 0, 100, kVals[0][0], kVals[0][1], kVals[0][2]);
AutoPID roll(&filteredData[1], &targetAngles[1], &PIDOutput[1], 0, 100, kVals[1][0], kVals[1][1], kVals[1][2]);
AutoPID yaw(&filteredData[2], &targetAngles[2], &PIDOutput[2], 0, 100, kVals[2][0], kVals[2][1], kVals[2][2]);
float height = 0;

void setup() {
  Serial.begin(9600);
  for(int i = 0; i < 4; i++){
    ESC[i].attach(ESCPin[i]);
  }
  detectIMU();
  enableIMU();
  //read the data from the IMU
  readIMU();
}

void loop() {
  unsigned long timer = millis();
  //get raw IMU data
  readIMU();
  //calculate tilt angle
  calcLevelAngle();
  //filter IMU data
  filterData();
  //get reciever input channels 1-4
  getRXInput(4);
  //update the PID loop with the new information
  updatePID();
  //output new values to the ESC
  updateESCs();
  Serial.println(millis()-timer);
}

//read the data from the IMU
void readIMU(){ 
  byte buff[6];
  unsigned int startTime = millis();

  //Read accelerometer data
  readACC(buff);
  for(int i = 0; i <= 4; i += 2){
    //Read the accelerometer data from the I2C bus
    int accRaw = (int)(buff[i] | (buff[i+1] << 8));
    /*Convert the data to G-force and save it to the 
     * first entry in the IMU array.
     */
    IMUData[0][i/2] = (float)accRaw*0.122/1000;
  }

  //read gyroscope data
  readGYR(buff);
  for(int i = 0; i <=4; i+=2){
    int gyrRaw = (int)(buff[i] | (buff[i+1] << 8));
    //convert data to degrees per second then add it to the array
    IMUData[1][i/2] += (float) gyrRaw * G_GAIN * DT;
  }

  //Read the compass data
  readMAG(buff);
  for(int i = 0; i <=4; i += 2){
    IMUData[2][i/2] = (int)(buff[i] | (buff[i+1] << 8));
  }
  //Compute heading  
   heading = 180 * atan2(IMUData[2][1],IMUData[2][0])/M_PI;
  
  //Convert heading to 0 - 360
  while(IMUData[2][0] < 0)
    IMUData[2][0] += 360;
  
  //Each loop should be at least 20ms.
  while(millis() - startTime < (DT*1000)){
    delay(1);
  }
}

//calculate the tilt angles in degrees from the accelerometer data
void calcLevelAngle(){
  float magnitude = sqrt(sq(IMUData[0][0]) + sq(IMUData[0][1]) + sq(IMUData[0][2]));
  for(int i = 0; i < 3; i++){
    accelAngles[i] = round(180*acos(IMUData[0][i]/magnitude)/3.1415);
  }
}

//updates the PID loops with new information
void updatePID(){
  pitch.run();
  roll.run();
  yaw.run();
}

//Assigns new values to the ESCs based on the PID output
void updateESCs(){
  float e = 2.71828;
  float FL = height - PIDOutput[0] + PIDOutput[1] + PIDOutput[2];
  float FR = height - PIDOutput[0] - PIDOutput[1] - PIDOutput[2];
  float BL = height + PIDOutput[0] + PIDOutput[1] - PIDOutput[2];
  float BR = height + PIDOutput[0] - PIDOutput[1] + PIDOutput[2];

  /*apply the logistic growth formula of (1/(1+e^(-x)))
   * to normalize the output to something between 0 and 180
   */
  ESC[0].write(180/(1 + pow(e, -0.05*(FL-100))) + 90);
  //FR's output range is different so it gets a different formula
  ESC[1].write(180/(1 + pow(e, -0.05*(FL+100))) + 90);
  ESC[2].write(180/(1 + pow(e, -0.05*(FL-100))) + 90);
  ESC[3].write(180/(1 + pow(e, -0.05*(FL-100))) + 90);
}

//combines the IMU data to get more usable results
void filterData(){
  //combine accelX with gyroX
  filteredData[0] = 0.98*IMUData[1][0] + 0.02*accelAngles[0];
  //combine accelY with gyroY
  filteredData[1] = 0.98*IMUData[1][1] + 0.02*accelAngles[1];
  //combine heading with gyroZ
  filteredData[2] = 0.98*IMUData[1][2] + 0.02*heading;
}

//read the data from the RC reciever.
void getRXInput(int pinRange){
  int maxAngle = 30; //degrees
  for(int i = 0; i < pinRange; i++){
    //Takes the input and converts to a range between -1 and 1.
    RXData[i] = (pulseIn(RXPin[i], HIGH, 21495)-1565)/365;
    if(i < 3){
      targetAngles[i] = maxAngle*RXData[i];
    }
  }
  if(RXData[0] == 0 && RXData[1] == 0 && RXData[2] == 0 && RXData[3] == 0){
    failSafe = true;
  }
  height = (RXData[3]+1)*50;
}
