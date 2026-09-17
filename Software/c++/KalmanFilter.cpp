#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() {
    P00_X = 1;
    P01_X = 0;
    P10_X = 0;
    P11_X = 1;

    P00_Y = 1;
    P01_Y = 0;
    P10_Y = 0;
    P11_Y = 1;

    Rx = 1; // Measurement noise covariance for X
    Ry = 1; // Measurement noise covariance for Y

    Q_angle = 0.8; // Process noise covariance for angle
    Q_bias = 0.003; // Process noise covariance for bias
}

KalmanResult KalmanFilter::update(float Acc_X, float Acc_Y, float Gyro_X, float Gyro_Y, float dt) {
    // Prediction step for X
    P00_X += dt * (dt * P11_X - P01_X - P10_X + Q_angle);
    P01_X -= dt * P11_X;
    P10_X -= dt * P11_X;
    P11_X += dt * Q_bias;

    P00_Y += dt * (dt * P11_Y - P01_Y - P10_Y + Q_angle);
    P01_Y -= dt * P11_Y;
    P10_Y -= dt * P11_Y;
    P11_Y += dt * Q_bias;

    // Error

    float y_X = Acc_X - Gyro_X;
    float y_Y = Acc_Y - Gyro_Y;

    // Innovation Covariance

    float S_X = P00_X + Rx;
    float S_Y = P00_Y + Ry;

    // Kalman Gains

    float K0_X = P00_X / S_X;
    float K1_X = P10_X / S_X;
    float K0_Y = P00_Y / S_Y;
    float K1_Y = P10_Y / S_Y;

    // Save the current state of the covariance matrices before updating them

    float temp_P00_X = P00_X;
    float temp_P01_X = P01_X;
    float temp_P10_X = P10_X;
    float temp_P11_X = P11_X;

    float temp_P00_Y = P00_Y;
    float temp_P01_Y = P01_Y;
    float temp_P10_Y = P10_Y;
    float temp_P11_Y = P11_Y;

    // Update the covariance matrices

    P00_X -= K0_X*temp_P00_X;
    P01_X -= K0_X*temp_P01_X;
    P10_X -= K1_X*temp_P00_X;
    P11_X -= K1_X*temp_P01_X;

    P00_Y -= K0_Y*temp_P00_Y;
    P01_Y -= K0_Y*temp_P01_Y;
    P10_Y -= K1_Y*temp_P00_Y;
    P11_Y -= K1_Y*temp_P01_Y;

    KalmanResult result;
    result.K0_X = K0_X;
    result.K1_X = K1_X;

    result.K0_Y = K0_Y;
    result.K1_Y = K1_Y;

    result.y_X = y_X;
    result.y_Y = y_Y;
    
    return result;

}

