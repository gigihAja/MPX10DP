#include "Sensors.h"
    
Sensors::Sensors(int dA, int sA, int dB, int sB)
    : doutPinA(dA), sckPinA(sA), doutPinB(dB), sckPinB(sB){}

long Sensors::readPressure(int doutPin, int sckPin) {
    long value = 0;
    while (digitalRead(doutPin));
  
    for (int i = 0; i < 24; i++) {
        digitalWrite(sckPin, HIGH);
        delayMicroseconds(1);
        value = (value << 1) | digitalRead(doutPin);
        digitalWrite(sckPin, LOW);
        delayMicroseconds(1);
    }

    for (int i = 0; i < 3; i++) {
        digitalWrite(sckPin, HIGH);
        delayMicroseconds(1);
        digitalWrite(sckPin, LOW);
        delayMicroseconds(1);
    }

    return (value < 0 || value > 16777216) ? 0 : value;
}

// long Sensors::avg(int doutPin, int sckPin) {
//     long sum = 0;
//     for (int i = 0; i < sampling; i++) {
//         sum += readPressure(doutPin, sckPin);
//         delay(5);
//     }
//     return sum / sampling;
// }

// void Sensors::setOffsets(int doutPinA, int sckPinA, int doutPinB, int sckPinB) {
//     offsetA = avg(doutPinA, sckPinA);
//     offsetB = avg(doutPinB, sckPinB);
// }

// long Sensors::getOffsetA() const { return offsetA; }
// long Sensors::getOffsetB() const { return offsetB; }

// long Sensors::getCalibratedA() {
//     long raw = avg(doutPinA, sckPinA);
//     return raw - offsetA;
// }

// long Sensors::getCalibratedB() {
//     long raw = avg(doutPinB, sckPinB);
//     return raw - offsetB;
// }

long Sensors::getRaw1(){
    long getRaw = readPressure(doutPinA,sckPinA);
    return getRaw;
}

long Sensors::getRaw2(){
    long getRaw = readPressure(doutPinB,sckPinB);
    return getRaw;
}