#include <Arduino.h>
#include "Sensors.h"
#include "calculate.h"
// #include <FreeRTOS.h>
// #include <Adafruit_NeoPixel.h>
#include <iostream>

using namespace std;
#define LED_PIN 48
#define NUMPIXELS 1

int led_colur;
// inlet pressure sensor
#define DOUT_1 19
#define SCK_1 18
// throat pressure sensor
#define DOUT_2 4 // 19
#define SCK_2 2  // 18

#define calibrate_button 22
#define calibrate_sampling 100

#define inlet 0.25 // cm
#define throat 0.1 // cm
#define airdensity 1.225

volatile bool calibrate = false;

// Adafruit_NeoPixel strip(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
Sensors sensor(DOUT_2, SCK_2, DOUT_1, SCK_1);
Calculate calculate(inlet, throat, airdensity);

QueueHandle_t queue1;
QueueHandle_t queue2;
QueueHandle_t queue3;

long offset1 = 0;
long offset2 = 0;
float scale1 = 0.1;
float scale2 = 0.1;
float airVolumeGlobal = 0;

void readSensor1(void *pvParameters)
{
    while (1)
    {
        long pressure1 = sensor.getRaw1();
        Serial.println("Inlet Pressure : " + pressure1);
        xQueueSend(queue1, &pressure1, portMAX_DELAY);
        vTaskDelay(150 / portTICK_PERIOD_MS);
    }
}

void readSensor2(void *pvParameters)
{
    while (1)
    {
        long pressure2 = sensor.getRaw2();
        Serial.println("Throat Pressure : " + pressure2);
        xQueueSend(queue2, &pressure2, portMAX_DELAY);
        vTaskDelay(150 / portTICK_PERIOD_MS);
    }
}

void proccessAirflow(void *pvParameters)
{
    while (1)
    {
        if (!calibrate)
        {
            long pressure1, pressure2;

            if (xQueueReceive(queue1, &pressure1, portMAX_DELAY) == pdPASS)
            {
                if (xQueueReceive(queue2, &pressure2, portMAX_DELAY) == pdPASS)
                {
                    float airflow = calculate.getAirflow(
                        pressure2, pressure1, offset1, offset2, scale1, scale2);

                    if (airflow < 0)
                    {
                        Serial.println("[Warning] airflow invalid, skipped.");
                        continue;
                    }

                    Serial.print("Airflow: ");
                    Serial.print(airflow, 2);
                    Serial.println(" ml/s");
                    led_colur = map(airflow, 0, 300, 0, 255);
                    xQueueSend(queue3, &airflow, portMAX_DELAY);
                }
            }
        }
    }
}

void airVolume(void *pvParameters)
{
    float airVolume = 0;
    float airVolumeLast = 0;
    float prevAirflow = -1.0f;
    TickType_t lastTick = xTaskGetTickCount();

    while (1)
    {
        if (calibrate)
        {
            airVolume = 0;
            prevAirflow = -1.0f; // Reset juga prevAirflow agar tidak salah akumulasi
            continue;
        }
        if (!calibrate)
        {
            float airflow;
            if (xQueueReceive(queue3, &airflow, portMAX_DELAY) == pdPASS)
            {
                TickType_t nowTick = xTaskGetTickCount();
                float samplingTime = (nowTick - lastTick) * portTICK_PERIOD_MS / 1000.0f; // detik
                lastTick = nowTick;

                if (calibrate)
                    airVolume = 0;

                if (prevAirflow >= 0)
                {
                    float area = 0.5f * (prevAirflow + airflow) * samplingTime;
                    airVolumeLast = airVolume;
                    airVolume += area;

                    if (!isfinite(airVolume))
                    {
                        Serial.println("[Error] Volume overflow detected. Rolling back...");
                        airVolume = airVolumeLast;
                    }
                }

                prevAirflow = airflow;

                Serial.print("Total volume: ");
                Serial.print(airVolume);
                Serial.println(" ml");
            }
        }
    }
}

void calibrateTask(void *pvParameters)
{
    long avg1;
    long avg2;
    long delta;
    bool lastButtonState = HIGH;
    while (1)
    {
        bool currentState = digitalRead(calibrate_button);

        if (lastButtonState == HIGH && currentState == LOW)
        {
            calibrate = true;
            // strip.setPixelColor(0, strip.Color(255, 255, 0));
            // strip.show();
            long sum1 = 0, sum2 = 0;
            const int samples = 100;
            Serial.println("kalibrasi mulai");
            vTaskDelay(1500 / portTICK_PERIOD_MS);
            for (int i = 0; i < samples; i++)
            {
                sum1 += sensor.getRaw1();
                sum2 += sensor.getRaw2();
                vTaskDelay(5 / portTICK_PERIOD_MS);
            }
            avg1 = sum1 / samples;
            avg2 = sum2 / samples;

            if (avg1 > avg2)
            {
                delta = (avg1 - avg2);
                offset2 = (delta / 2);
                offset1 = -delta / 2;
            }
            else
            {
                delta = (avg2 - avg1);
                offset1 = (delta / 2);
                offset2 = -delta / 2;
            }

            scale1 = 40000.0f / 16777216.0f;
            scale2 = 40000.0f / 16777216.0f;
            Serial.print("offset1: ");
            Serial.println(offset1);
            Serial.print("offset2: ");
            Serial.println(offset2);
            Serial.println("Kalibrasi selesai");
            airVolumeGlobal = 0;
            //   strip.setPixelColor(0, strip.Color(0, 0, 0));
            // strip.show();
        }
        calibrate = false;
        lastButtonState = currentState;
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void setup()
{
    Serial.begin(115200);
    pinMode(DOUT_1, INPUT);
    pinMode(SCK_1, OUTPUT);
    pinMode(DOUT_2, INPUT);
    pinMode(SCK_2, OUTPUT);
    pinMode(calibrate_button, INPUT_PULLUP);
    Serial.println("Setup started!");
    delay(1000);

    queue1 = xQueueCreate(1, sizeof(long));
    queue2 = xQueueCreate(1, sizeof(long));
    queue3 = xQueueCreate(1, sizeof(float));

    xTaskCreate(readSensor1, "sensor 1", 1000, NULL, 1, NULL);
    xTaskCreate(readSensor2, "sensor 2", 1000, NULL, 1, NULL);
    xTaskCreate(proccessAirflow, "process airflow", 4096, NULL, 1, NULL);
    xTaskCreate(airVolume, "air volume", 2048, NULL, 1, NULL);
    xTaskCreate(calibrateTask, "calibrate", 4096, NULL, 1, NULL);

    // strip.begin();
    // strip.show();
}

void loop()
{
    //   strip.setPixelColor(0, strip.Color(led_colur, 0, led_colur));
    //   strip.show();
    //   delay(5);
}
