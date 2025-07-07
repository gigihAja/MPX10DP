#ifndef CALCULATE_H
#define CALCULATE_H 

class Calculate {
    private:
        float inletDiameter; // m
        float throatDiameter; // m
        float rho; // kg/m³ (air density)
        float airflow; // m³/s
        float pressure1; // Pa
        float pressure2; // Pa
        void convert(float p1, float p2);
        float findDelta(float p1, float p2);
        const float pi = 3.1415926f;

    public:
        Calculate(float in, float out, float ad);

        float convertToPressure(long raw, long offset, float scale);  
        
        float getAirflow(long raw1, long raw2, long offset1, long offset2, float scale1, float scale2); 

        void showPressures();

};

#endif
