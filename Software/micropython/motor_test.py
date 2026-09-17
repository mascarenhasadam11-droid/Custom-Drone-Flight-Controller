from dshot import DShotMotor
import time


# =========================
# Motor mapping
# =========================

MOTOR_PINS = {
    1: 25,
    2: 27,
    3: 26,
    4: 14
}


# =========================
# Create motors
# =========================

motor1 = DShotMotor(0, MOTOR_PINS[1])
motor2 = DShotMotor(1, MOTOR_PINS[2])
motor3 = DShotMotor(2, MOTOR_PINS[3])
motor4 = DShotMotor(3, MOTOR_PINS[4])


motors = [
    motor1,
    motor2,
    motor3,
    motor4
]


# =========================
# ESC initialization
# =========================

print("Initializing ESCs...")

start = time.ticks_ms()

while time.ticks_diff(time.ticks_ms(), start) < 3000:

    for motor in motors:
        motor.set_throttle(0)

    time.sleep_us(1000)


print("ESCs ready")


# =========================
# Test all motors
# =========================

print("Running all motors at 100")

start = time.ticks_ms()

while time.ticks_diff(time.ticks_ms(), start) < 3000:

    for motor in motors:
        motor.set_throttle(100)

    time.sleep_us(1000)


# =========================
# Stop
# =========================

print("Stopping")

while True:

    for motor in motors:
        motor.stop()

    time.sleep_us(1000)