#include <Arduino.h>
#include "MPU6050.h"

MPU6050 mpu;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("Starting MPU6050...");

    // ESP32 DevKit V1
    // SDA = GPIO21
    // SCL = GPIO22
    mpu.begin(21, 22);

    Serial.println("MPU6050 initialized");

    // Calibrate the MPU6050
    // Keep the sensor completely still and level
    mpu.calibrate();

    Serial.println("Calibration finished");
    Serial.println("Starting Kalman filter...");
}

void loop()
{

    static unsigned long lastIMUTime = 0;
    static unsigned long lastPrintTime = 0;
    static unsigned long lastRateTime = 0;

    static float latestX = 0;
    static float latestY = 0;

    unsigned long currentTime = micros();
    static unsigned long imuCount = 0;


    // =========================
    // IMU + Kalman: 500 Hz
    // =========================

    if (currentTime - lastIMUTime >= 2000)
    {
        lastIMUTime = currentTime;

        MPUAngles angles = mpu.calculateAngles();

        latestX = angles.x;
        latestY = angles.y;

        imuCount++; 
    }


    // =========================
    // Serial debug: 10 Hz
    // =========================

    if (currentTime - lastPrintTime >= 100000)
    {
        lastPrintTime = currentTime;

        Serial.print("X: ");
        Serial.print(latestX, 2);

        Serial.print("  Y: ");
        Serial.println(latestY, 2);
    }

    // =========================
// Measure actual IMU rate
// =========================

    if (currentTime - lastRateTime >= 1000000)
    {
        lastRateTime = currentTime;

        Serial.print("Actual IMU rate: ");
        Serial.print(imuCount);
        Serial.println(" Hz");

        imuCount = 0;
    }
}