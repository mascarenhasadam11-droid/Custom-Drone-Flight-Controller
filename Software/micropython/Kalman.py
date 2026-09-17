from machine import I2C, Pin
import time
from math import atan, degrees, atan2, sqrt

print("MPU6050 initialized successfully")

class KalmanFilter:
    def __init__(self):
        
        self.P00_X = 1
        self.P01_X = 0
        self.P10_X = 0
        self.P11_X = 1

        self.P00_Y = 1
        self.P01_Y = 0
        self.P10_Y = 0
        self.P11_Y = 1

        # P = FPF^T + Q

        self.Rx = 1
        self.Ry = 1

        self.Q_angle = 0.8
        self.Q_bias = 0.003

    def update(self, Acc_X, Acc_Y, Gyro_X, Gyro_Y, dt):
        
        self.P00_X += dt * (dt*self.P11_X - self.P01_X - self.P10_X + self.Q_angle)
        self.P01_X -= dt * self.P11_X
        self.P10_X -= dt * self.P11_X
        self.P11_X += self.Q_bias * dt

        self.P00_Y += dt * (dt*self.P11_Y - self.P01_Y - self.P10_Y + self.Q_angle)
        self.P01_Y -= dt * self.P11_Y
        self.P10_Y -= dt * self.P11_Y
        self.P11_Y += self.Q_bias * dt

        y_X = Acc_X - Gyro_X
        y_Y = Acc_Y - Gyro_Y


        S_X = self.P00_X + self.Rx
        S_Y = self.P00_Y + self.Ry

        self.K_X = [self.P00_X/S_X, self.P10_X/S_X]       #Kalman Gain Calculation [Angle Gain, Bias Gain]
        self.K_Y = [self.P00_Y/S_Y, self.P10_Y/S_Y]

        self.temp_P00_X = self.P00_X
        self.temp_P01_X = self.P01_X
        self.temp_P10_X = self.P10_X
        self.temp_P11_X = self.P11_X

        self.temp_P00_Y = self.P00_Y
        self.temp_P01_Y = self.P01_Y
        self.temp_P10_Y = self.P10_Y
        self.temp_P11_Y = self.P11_Y

        self.P00_X -= self.K_X[0]*self.temp_P00_X
        self.P01_X -= self.K_X[0]*self.temp_P01_X
        self.P10_X -= self.K_X[1]*self.temp_P00_X
        self.P11_X -= self.K_X[1]*self.temp_P01_X

        self.P00_Y -= self.K_Y[0]*self.temp_P00_Y
        self.P01_Y -= self.K_Y[0]*self.temp_P01_Y
        self.P10_Y -= self.K_Y[1]*self.temp_P00_Y
        self.P11_Y -= self.K_Y[1]*self.temp_P01_Y

        return self.K_X, self.K_Y, y_X, y_Y



class MPU6050:
    def __init__(self, busid, SDA, SCL):
        self.PWR_MGMT_1 = 0x6B
        self.SMPLRT_DIV = 0x19
        self.CONFIG = 0x1A
        self.GYRO_CONFIG = 0x1B
        self.ACCEL_XOUT_H = 0x3B
        self.ACCEL_YOUT_H = 0x3D
        self.ACCEL_ZOUT_H = 0x3F
        self.GYRO_XOUT_H = 0x43
        self.GYRO_YOUT_H = 0x45
        self.GYRO_ZOUT_H = 0x47
        self.mpu6050_addr = 0x68

        self.gX_pred = 0  #Initializing the predicted gyro angles to zero
        self.gY_pred = 0  

        self.base_Rx = 0.1
        self.base_Ry = 0.1

        self.i2c = I2C(busid, sda=Pin(SDA), scl=Pin(SCL))
        self.i2c.writeto_mem(self.mpu6050_addr, self.PWR_MGMT_1, b'\x01')
        self.i2c.writeto_mem(self.mpu6050_addr, self.CONFIG, b'\x03')

        self.start = None
        
        self.kalman = KalmanFilter()

    def _bytes_to_signed_16bit_(self, hi, lo):
        return (((hi << 8) | lo) ^ 0x8000) - 0x8000

    def _read_raw_data_(self, addr):
        bytes = self.i2c.readfrom_mem(self.mpu6050_addr, addr, 2)
        return self._bytes_to_signed_16bit_(bytes[0], bytes[1])

    def _read_all_(self):
        data = self.i2c.readfrom_mem(self.mpu6050_addr, 0x3B, 14)
        ax = self._bytes_to_signed_16bit_(data[0], data[1])
        ay = self._bytes_to_signed_16bit_(data[2], data[3])
        az = self._bytes_to_signed_16bit_(data[4], data[5])
        
        gx = self._bytes_to_signed_16bit_(data[8], data[9])
        gy = self._bytes_to_signed_16bit_(data[10], data[11])
        gz = self._bytes_to_signed_16bit_(data[12], data[13])
        return ax, ay, az, gx, gy, gz

    def read_acc(self):
        return (self._read_raw_data_(0x3B)/16384 - self.x_acc_bias, self._read_raw_data_(0x3D)/16384 - self.y_acc_bias, self._read_raw_data_(0x3F)/16384 - self.z_acc_bias)

    def read_gyro(self):
        return (self._read_raw_data_(0x43)/131, self._read_raw_data_(0x45)/131, self._read_raw_data_(0x47)/131)

    def calibrate(self, n=500, delay_ms=2):
        print("Calibrating... keep still and level")
        sum_ax = sum_ay = sum_az = 0
        sum_gx = sum_gy = 0

        for _ in range(n):
            ax, ay, az, gx, gy, gz = self._read_all_()
            sum_ax += ax/16384
            sum_ay += ay/16384
            sum_az += az/16384
            sum_gx += gx/131
            sum_gy += gy/131
            time.sleep_ms(delay_ms)

        self.x_acc_bias = sum_ax/n
        self.y_acc_bias = sum_ay/n
        self.z_acc_bias = (sum_az/n) - 1.0

        self.bX = sum_gx/n
        self.bY = sum_gy/n

        print("Calibration done: acc_bias=({:.4f},{:.4f},{:.4f}) gyro_bias=({:.4f},{:.4f})".format(
            self.x_acc_bias, self.y_acc_bias, self.z_acc_bias, self.bX, self.bY))

    def calculate_dt(self):
        start = self.start or time.ticks_us()
        now = time.ticks_us()
        dt = time.ticks_diff(now, start)/10**6
        self.start = now

        if dt <= 0 or dt > 0.1:
            dt = 0.01

        return dt

    def calculate_angles(self):

        ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = self._read_all_()

        ax = ax_raw/16384 - self.x_acc_bias
        ay = ay_raw/16384 - self.y_acc_bias
        az = az_raw/16384 - self.z_acc_bias

        accel_mag = sqrt(ax*ax + ay*ay + az*az)
        mag_error = abs(accel_mag - 1.0)
        R_scale = min(1.0 + (mag_error * 12), 15.0)   # tune the 50 multiplier empirically
        self.kalman.Rx = self.base_Rx * R_scale
        self.kalman.Ry = self.base_Ry * R_scale

        gX = gx_raw/131
        gY = gy_raw/131

        aX_angles = degrees(atan2(ay, sqrt(ax*ax + az*az)))               #ACCELEROMETER ANGLE CALCULATION
        aY_angles = degrees(atan2(-ax, sqrt(ay*ay + az*az)))

        dt = self.calculate_dt()
        self.gX_pred += dt*(gX-self.bX)                                  #GYRO ANGLE CALCULATION
        self.gY_pred += dt*(gY-self.bY)
        
        K_X, K_Y, X_Error, Y_Error = self.kalman.update(aX_angles, aY_angles, self.gX_pred, self.gY_pred, dt)

        return (self.return_angles(K_X, K_Y, X_Error, Y_Error))

    def return_angles(self, X_Gain, Y_Gain, X_Error, Y_Error):

        self.bX += (X_Error*X_Gain[1])                          #KALMAN FILTER ANGLE CALCULATION
        self.bY += (Y_Error*Y_Gain[1])

        self.gX_pred += X_Error*X_Gain[0]
        self.gY_pred += Y_Error*Y_Gain[0]

        return (self.gX_pred, self.gY_pred)