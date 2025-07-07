#include "calculate.h"
#include <cmath> 
#include<Arduino.h>
Calculate::Calculate(float in, float out, float ad)
    : inletDiameter(in), throatDiameter(out), rho(ad), pressure1(0), pressure2(0) {}

float Calculate::convertToPressure(long raw, long offset, float scale) {  
    return (raw - offset) * scale;
}

float Calculate::findDelta(float p1, float p2) {
    float delta = p1 - p2;
    if (delta <= 0) {
        return 0;
    }
    return delta;
}

void Calculate::showPressures(){
     Serial.print(pressure1);Serial.println("pa");
     Serial.print(pressure2);Serial.println("pa");
}

float Calculate::getAirflow(long raw1, long raw2, long offset1, long offset2, float scale1, float scale2) {
    
    pressure1 = convertToPressure(raw1, offset1, scale1);  
    pressure2 = convertToPressure(raw2, offset2, scale2);  

    float delta = findDelta(pressure1, pressure2);

    float A2 = (M_PI * pow(throatDiameter, 2)) / 4.0f;  
    float beta = throatDiameter / inletDiameter;
    float term = 1.0f - pow(beta, 4);

    if (term <= 0.0f) { 
        return 0.0f;
    }

    float airflow = A2 * sqrt((2.0f * delta) / (rho * term)); 
    return airflow*15000;
    
}

