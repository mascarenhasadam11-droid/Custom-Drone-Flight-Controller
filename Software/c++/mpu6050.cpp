#include <Arduino.h>
#include "MPU6050.h"
#include <math.h>
#include "KalmanFilter.h"

MPU6050::MPU6050()
{
    x_acc_bias = 0;
    y_acc_bias = 0;
    z_acc_bias = 0;

    bX = 0;
    bY = 0;

    base_Rx = 0;
    base_Ry = 0;

    gX_pred = 0;
    gY_pred = 0;
    previousTime = 0;
}

void MPU6050::begin(int SDA, int SCL)
{
    Wire.begin(SDA, SCL);

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(PWR_MGMT_1);
    Wire.write(0x01);
    Wire.endTransmission();

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(CONFIG);
    Wire.write(0x03);
    Wire.endTransmission();


    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(SMPLRT_DIV);
    Wire.write(0x01);
    Wire.endTransmission();
}

int16_t MPU6050::bytesToSigned16(uint8_t hi, uint8_t lo)
{
    return (int16_t)((hi << 8) | lo);
}


MPUData MPU6050::readAll()
{
    MPUData data;

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU6050_ADDR, (uint8_t)14);

    uint8_t buffer[14];

    for (int i = 0; i < 14; i++)
    {
        buffer[i] = Wire.read();
    }

    int16_t ax_raw = bytesToSigned16(buffer[0], buffer[1]);
    int16_t ay_raw = bytesToSigned16(buffer[2], buffer[3]);
    int16_t az_raw = bytesToSigned16(buffer[4], buffer[5]);

    int16_t gx_raw = bytesToSigned16(buffer[8], buffer[9]);
    int16_t gy_raw = bytesToSigned16(buffer[10], buffer[11]);
    int16_t gz_raw = bytesToSigned16(buffer[12], buffer[13]);

    data.ax = ax_raw / 16384.0 - x_acc_bias;
    data.ay = ay_raw / 16384.0 - y_acc_bias;
    data.az = az_raw / 16384.0 - z_acc_bias;

    data.gx = gx_raw / 131.0;
    data.gy = gy_raw / 131.0;
    data.gz = gz_raw / 131.0;

    return data;
}

void MPU6050::calibrate(int n, int delay_ms)
{
    Serial.println("Calibrating MPU6050...Keep still and level...");

    x_acc_bias = 0 ;
    y_acc_bias = 0;
    z_acc_bias = 0;

    bX = 0;
    bY = 0;

    float sum_ax = 0;
    float sum_ay = 0;
    float sum_az = 0;

    float sum_gx = 0;
    float sum_gy = 0;

    for (int i = 0; i < n; i++)
    {
        MPUData data = readAll();
        sum_ax += data.ax;
        sum_ay += data.ay;
        sum_az += data.az;

        sum_gx += data.gx;
        sum_gy += data.gy;

        delay(delay_ms);
    }

    x_acc_bias = sum_ax / n;
    y_acc_bias = sum_ay / n;
    z_acc_bias = sum_az / n - 1.0; // Assuming the device is stationary and facing up

    bX = sum_gx / n;
    bY = sum_gy / n;

    Serial.println("Calibration complete.");

    Serial.print("X Acc Bias: ");
    Serial.println(x_acc_bias, 4);
    Serial.print("Y Acc Bias: ");
    Serial.println(y_acc_bias, 4);
    Serial.print("Z Acc Bias: ");
    Serial.println(z_acc_bias, 4);
    Serial.print("X Gyro Bias: ");
    Serial.println(bX, 4);
    Serial.print("Y Gyro Bias: ");
    Serial.println(bY, 4);

}

float MPU6050::calculateDt()
{
    unsigned long currentTime = micros();

    if (previousTime == 0)
    {
        previousTime = currentTime;
        return 0.01;
    }

    float dt = (currentTime - previousTime) / 1000000.0;

    previousTime = currentTime;

    if (dt <= 0 || dt > 0.1)
    {
        dt = 0.01;
    }

    return dt;
}

MPUAngles MPU6050::calculateAngles()
{
    MPUData data = readAll();

    float ax = data.ax;
    float ay = data.ay;
    float az = data.az;

    float accel_mag = sqrt(ax * ax + ay * ay + az * az);

    float mag_error = fabs(accel_mag - 1.0);

    float R_scale = 1.0 + (mag_error * 12.0);

    if (R_scale > 15.0)
    {
        R_scale = 15.0;
    }

    kalman.Rx = base_Rx * R_scale;
    kalman.Ry = base_Ry * R_scale;

    float gX = data.gx;
    float gY = data.gy;

    float aX_angle =
        atan2(ay, sqrt(ax * ax + az * az))
        * 180.0 / PI;

    float aY_angle =
        atan2(-ax, sqrt(ay * ay + az * az))
        * 180.0 / PI;

    float dt = calculateDt();

    gX_pred += dt * (gX - bX);
    gY_pred += dt * (gY - bY);

    KalmanResult resultKalman = kalman.update(aX_angle, aY_angle, gX_pred, gY_pred, dt);

    bX += resultKalman.y_X * resultKalman.K1_X;
    bY += resultKalman.y_Y * resultKalman.K1_Y;

    gX_pred += resultKalman.y_X * resultKalman.K0_X;
    gY_pred += resultKalman.y_Y * resultKalman.K0_Y;

    MPUAngles result;

    result.x = gX_pred;
    result.y = gY_pred;

    return result;
}