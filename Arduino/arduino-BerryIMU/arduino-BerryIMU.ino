#include <AutoPID.h>

#include "IMU.h"

#define DT  0.02          // Loop time
#define G_GAIN 0.070    // [deg/s/LSB]

/*This stores the data recieved from the accelerometer,
 * Gyroscope, and compass in a 3x3 array.
 * Accelerometer is in IMUData[0] (g-force)
 * Gyroscope is in IMUData[1] (degrees)
 * Compass data is in IMUData[2] (raw compass data)
 */
float IMUData[3][3];
//the calculated heading from the compass
float heading = 0;

//0.5 to 2.5 or 3 ms on-time PWM signal
AutoPID

void setup() {
  Serial.begin(9600);
  //begins I2C communications with the IMU
  detectIMU();
  enableIMU();
  //read the data from the IMU
  readIMU();
}

void loop() {
  readIMU();
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
  while(millis() - startTime < (DT*1000))
        {
            delay(1);
        }
}

//calculate the polar angles for the accelerometer
float[] calcLevelAngle(){
  float angles[3];
  float magnitude = sqrt(sq(IMUData[0][0]) + sq(IMUData[0][1]) + sq(IMUData[0][2]));
  for(int i = 0; i < 3; i++){
    angles[i] = acos(IMUData[0][i]/magnitude);
  }
  return angles;
}
