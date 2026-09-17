#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <Wire.h>
#include "KalmanFilter.h"

struct MPUData
{
    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;
};

struct MPUAngles
{
    float x;
    float y;
};

class MPU6050
{

private:
    KalmanFilter kalman;

public:

    // MPU6050 registers
    const uint8_t PWR_MGMT_1   = 0x6B;
    const uint8_t SMPLRT_DIV   = 0x19;
    const uint8_t CONFIG       = 0x1A;
    const uint8_t GYRO_CONFIG  = 0x1B;

    const uint8_t ACCEL_XOUT_H = 0x3B;
    const uint8_t ACCEL_YOUT_H = 0x3D;
    const uint8_t ACCEL_ZOUT_H = 0x3F;

    const uint8_t GYRO_XOUT_H  = 0x43;
    const uint8_t GYRO_YOUT_H  = 0x45;
    const uint8_t GYRO_ZOUT_H  = 0x47;

    const uint8_t MPU6050_ADDR = 0x68;

    // Calibration values
    float x_acc_bias;
    float y_acc_bias;
    float z_acc_bias;

    float bX;
    float bY;

    float base_Rx;
    float base_Ry; 

    // Predicted gyro angles
    float gX_pred;
    float gY_pred;

    // Timing
    unsigned long previousTime;

    // Constructor
    MPU6050();

    // Start MPU6050
    void begin(int SDA, int SCL);

    int16_t bytesToSigned16(uint8_t hi, uint8_t lo);

    MPUData readAll();

    void calibrate(int n = 500, int delay_ms = 2);

    MPUAngles calculateAngles();

    float calculateDt();
};

#endif