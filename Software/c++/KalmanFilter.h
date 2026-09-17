#ifndef KALMAN_H
#define KALMAN_H

struct KalmanResult {
    float K0_X;
    float K1_X;

    float K0_Y;
    float K1_Y;

    float y_X;
    float y_Y;
};

class KalmanFilter 
{
public:

    KalmanFilter();

    KalmanResult update(float Acc_X, float Acc_Y, float Gyro_X, float Gyro_Y, float dt);
    
    float Rx;
    float Ry;

private:

    float P00_X;
    float P01_X;
    float P10_X;
    float P11_X;

    float P00_Y;
    float P01_Y;
    float P10_Y;
    float P11_Y;

    float Q_angle;
    float Q_bias;

};

#endif