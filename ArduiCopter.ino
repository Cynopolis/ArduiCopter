#include <AutoPID.h>
#include "IMU.h"
#include "Servo.h"

#define G_GAIN 70/1000    // [deg/s/LSB]

/*This stores the data recieved from the accelerometer,
 * Gyroscope, and compass in a 3x3 array.
 * Accelerometer is in IMUData[0] (g-force)
 * Gyroscope is in IMUData[1] (degrees)
 * Compass data is in IMUData[2] (raw compass data)
 * Unless stated otherwise ALL data is stored in the order of x, y, z
 */
float IMUData[3][3];//calibration data from the accelerometer and gyroscope
float IMUCalibration[2][3];
//the filtered angles that combines the accel and gyro input (degrees)
float filteredData[3] = {0, 0, 0};
//Using servo objects to output data to ESCs
Servo ESC[4];
//pin numbers of the ESC pins in the order of FL, FR, BL, BR
int ESCPin[4] = {2,6,7,8};
/*The pins to be connected to the reciever
 * pitch, roll, yaw, throttle, switch1, switch2
 */
int RXPin[6] = {A0, A1, A2, A3, A4, A5};
float RXData[6] = {0,0,0,0,0,0};
//true if the quadcopter has lost connection to the radio transmitter
boolean failSafe = false;
//compass heading
float heading = 0;

//the tilt angles computed by the accelerometer data (degrees)
float accelAngles[3];
//the target angles and the throttle the drone should be at (degrees) and throttle
float targetPosition[4] = {0, 0, 0, 0}; //this is level flight with no throttle

//PID controllers, their k values, and their outputs
float kVals[3][3] = {{1,1.5,1}, {1,1,1}, {1,1,1}};
float PIDOutput[3] = {0, 0, 0};
AutoPID pitch(&filteredData[0], &targetPosition[0], &PIDOutput[0], -100, 100, kVals[0][0], kVals[0][1], kVals[0][2]);
AutoPID roll(&filteredData[1], &targetPosition[1], &PIDOutput[1], -100, 100, kVals[1][0], kVals[1][1], kVals[1][2]);
AutoPID yaw(&filteredData[2], &targetPosition[2], &PIDOutput[2], -100, 100, kVals[2][0], kVals[2][1], kVals[2][2]);
//a timer
unsigned long timer = millis();

void setup() {
  Serial.begin(9600);
  for(int i = 0; i < 4; i++){
    ESC[i].attach(ESCPin[i]);
  }
  //IMU startup
  detectIMU();
  enableIMU();
  timer = millis();
  callibrateIMU();
  //read the data from the IMU
  readIMU();
}

void loop() {
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
  Serial.print("Filtered Pitch: ");
  Serial.print(filteredData[0]);
  Serial.print(", Accel Pitch: ");
  Serial.print(accelAngles[0]);
  Serial.print(", Gyro Pitch: ");
  Serial.print(IMUData[1][0]);
  Serial.print(", PID Output: ");
  Serial.println(PIDOutput[0]);
}

//read the data from the IMU
void readIMU(){ 
  byte buff[6];

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
    //Serial.println(gyrRaw);
    //convert data to degrees per second then add it to the array
    IMUData[1][i/2] += ((((float) gyrRaw-IMUCalibration[1][i/2]) * G_GAIN)) * ((float)(millis() -timer))/1000;
  }
  timer = millis();

  //Read the compass data
  readMAG(buff);
  for(int i = 0; i <=4; i += 2){
    IMUData[2][i/2] = (int)(buff[i] | (buff[i+1] << 8));
  }
  //Compute heading  
   heading = 180 * atan2(IMUData[2][1],IMUData[2][0])/M_PI;
  
  //keep heading between 0 - 360
  while(heading < 0)
    heading += 360;
  while(heading > 360)
    heading -= 360;
}

//calculate the tilt angles in degrees from the accelerometer data
void calcLevelAngle(){
  float magnitude = sqrt(sq(IMUData[0][0]) + sq(IMUData[0][1]) + sq(IMUData[0][2]));
  for(int i = 0; i < 3; i++){
    accelAngles[i] = round(180*acos(IMUData[0][i]/magnitude)/3.1415) -IMUCalibration[0][i];
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
  float props[4];
  //These equations describe the unique behavior of each motor
  props[0] = targetPosition[3] - PIDOutput[0] + PIDOutput[1] + PIDOutput[2];
  props[1] = targetPosition[3] - PIDOutput[0] - PIDOutput[1] - PIDOutput[2];
  props[2] = targetPosition[3] + PIDOutput[0] + PIDOutput[1] - PIDOutput[2];
  props[3] = targetPosition[3] + PIDOutput[0] - PIDOutput[1] + PIDOutput[2];

  /*apply the logistic growth formula (1/(1+e^(-x)))
   * to normalize the output to something between 0 and 180
   */
  for(int i = 0; i < 4; i++){
    //the weird negative constant is -1/67.
    //logistic formula has been modified so it's input range is from -400 to 400,  
    //and its output range is 0 to 180
    ESC[i].write((int)(180/(1 + pow(e, -0.01492537*(props[i])))));
  }
}

//combines the IMU data to get more usable results
void filterData(){
  //combine accelX with gyroX
  filteredData[0] = 0.9*IMUData[1][0] + 0.1*accelAngles[0];
  //combine accelY with gyroY
  filteredData[1] = 0.9*IMUData[1][1] + 0.1*accelAngles[1];
  //combine heading with gyroZ
  filteredData[2] = 0.9*IMUData[1][2] + 0.1*heading;
}

//read the data from the RC reciever.
void getRXInput(int pinRange){
  int maxAngle = 30; //degrees
  for(int i = 0; i < pinRange; i++){
    //Takes the input and converts to a range between -1 and 1.
    RXData[i] = (float)pulseIn(RXPin[i], HIGH, 21495);
    if(RXData[i] != 0){RXData[i] = (RXData[i]-1565)/365;}
  }
  if(RXData[0] == 0 && RXData[1] == 0 && RXData[2] == 0 && RXData[3] == 0){
    failSafe = true;
  }
  targetPosition[0] = RXData[0]*maxAngle;
  targetPosition[1] = RXData[1]*maxAngle;
  targetPosition[2] = RXData[2]*maxAngle;
  targetPosition[3] = (RXData[3]+1)*50;
}

//get rid of Gyro drift and normalize accelerometer angles.
void callibrateIMU(){
  Serial.println("Callibrating...");
  IMUCalibration[0][0] = 90;
  IMUCalibration[0][1] = 90;
  IMUCalibration[0][2] = 0;

  byte buff[6];
  float sums[3] = {0, 0, 0};
  int gyrData[3];
  for(int j = 0; j < 1000; j++){
    readGYR(buff);
    for(int i = 0; i <=4; i+=2){
      gyrData[i/2] = (int)(buff[i] | (buff[i+1] << 8));
      sums[i/2] += (float)gyrData[i/2];
    }
  }
  IMUCalibration[1][0] = sums[0]/1000;
  IMUCalibration[1][1] = sums[1]/1000;
  IMUCalibration[1][2] = sums[2]/1000;
  Serial.println(IMUCalibration[1][0]);
  Serial.println(IMUCalibration[1][1]);
  Serial.println(IMUCalibration[1][2]);
  Serial.println("Calibration Complete!");
}
