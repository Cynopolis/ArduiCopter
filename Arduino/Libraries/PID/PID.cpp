/*PID.h - Library for calculating a PID value given inputs and a setpoint
 * Created by Quinn Henthorne September 29, 2019
 */

#include "Arduino.h"
#include "PID.h"

//if max output is -1 then there will be no clamping
PID::PID(float kP, float kI, float kD, float target, float pos, float maxOutput){
  _kP = kP;
  _kI = kI;
  _kD = kD;
  _target = target;
  _pos = pos;
  _currentTime = millis();
  _lastTime = millis();
  _error = target - pos;
  _lastError = _error;
  _sumError = 0;
  _maxOutput = maxOutput;
}

float PID::calcPID(){
  _currentTime = millis();
  unsigned long elapsedTime = _currentTime - _lastTime;
  
  _error = _target - _pos; //proportional
  if(_isClamped == false){
    _sumError += _error * elapsedTime; //integral
  }
  float rateError = (_error - _lastError) / elapsedTime; //derivative
  
  _lastError = _error;
  _lastTime = _currentTime;

  float PID = kP * _error + kI * _sumError + kD * rateError;
  if(_maxOutput == -1){return PID;}
  else if(PID > _maxOutput){ _isClamped = true; return _maxOutput; }
  else{_isClamped = false; return PID;} 
}

void PID::setTarget(float target){_target = target;}
float PID::getTarget(){return _target;}

void PID::setPos(float pos){_pos = pos;}
float PID::getPos(){return pos;}

void PID::setKP(float kP){_kP = kP;}
float PID::getKP(){return kP;}

void PID::setKI(float kI){_kI = kI;}
float PID::getKI(){return kI;}

void PID::setKD(float kD){_kD = kD;}
float PID::getKD(){return kD;}

void PID::setMaxOutput(float maxOuput){_maxOutput = maxOutput;}
float PID::getMaxOutput(){return maxOutput;}
