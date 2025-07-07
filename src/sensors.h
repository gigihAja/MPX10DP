#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

class Sensors {
private:
    // int sampling;
    int doutPinA, sckPinA;
    int doutPinB, sckPinB;
    // long offsetA;
    // long offsetB;
    long readPressure(int doutPin, int sckPin);
public:
    Sensors(int dA, int sA, int dB, int sB);

    
    // long avg(int doutPin, int sckPin);
    // void setOffsets(int doutPinA, int sckPinA, int doutPinB, int sckPinB);
    // long getOffsetA() const;
    // long getOffsetB() const;
    // long getCalibratedA();
    // long getCalibratedB();
    long getRaw1();
    long getRaw2();
};

#endif  // SENSORS_H
