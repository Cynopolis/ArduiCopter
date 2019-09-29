/*PID.h - Library for calculating a PID value given inputs and a setpoint
 * Created by Quinn Henthorne September 29, 2019
 */

#ifndef PID_h
#define PID_h

#include "Arduino.h"
class PID{
  public:
    PID(float kP, float kI, float kD, float target, float pos, float maxOutput);
    float calcPID();
    void setTarget(float target);
    float getTarget();
    void setPos(float pos);
    float getPos();
    void setKP(float kP);
    float getKP();
    void setKI(float kI);
    float getKI();
    void setKD(float kD);
    float getKD();
    void setMaxOutput(float maxOuput);
    float getMaxOutput();
  private:
    float _kP;
    float _kI;
    float _kD;
    float _target;
    float _pos;
    unsigned long _currentTime;
    unsigned long _lastTime;
    float _error;
    float lastError;
    float _sumError;
    float _maxOutput;
    boolean _isClamped;
    
}

#endif
