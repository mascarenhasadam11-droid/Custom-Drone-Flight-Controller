# main.py
from Kalman import MPU6050
import time
import PID
from PID import PID, mixer
from dshot import motors
from Kalman import MPU6050
from receiver import unpack

sensor = MPU6050(0, 21, 22)   # busid, SDA pin, SCL pin
sensor.calibrate()            # <-- add this: measures offset, keep sensor still+level here

KP = [1, 2, 3]
KR = [1, 2, 3]
Integral_P = 0
Integral_R = 0
error_old_P = 0
error_old_R = 0

last_time = time.ticks_us()

while True:
    
    now = time.ticks_us()
    dt = time.ticks_diff(now, last_time) / 1_000_000
    last_time = now
    
    X_angle, Y_angle = sensor.calculate_angles()
    #time.sleep(0.005)
    
    uncalibrated_yaw, throttle, sel1, uncalibrated_roll, uncalibrated_pitch, sel2 = unpack()
    
    MAX_ANGLE = 30
    
    desired_pitch = uncalibrated_pitch * 0.001 * MAX_ANGLE
    desired_roll = uncalibrated_roll * 0.001 * MAX_ANGLE
    

    pitch, Integral_P, error_old_P = PID(desired_pitch, X_angle, Integral_P, error_old_P, KP[0], KP[1], KP[2])
    roll, Integral_R, error_old_R = PID(desired_roll, Y_angle, Integral_R, error_old_R, KR[0], KR[1], KR[2])
    
    M1, M2, M3, M4 = mixer(throttle, pitch, roll)
    
    motors[1].set_throttle(M1)
    motors[2].set_throttle(M2)
    motors[3].set_throttle(M3)
    motors[4].set_throttle(M4)
