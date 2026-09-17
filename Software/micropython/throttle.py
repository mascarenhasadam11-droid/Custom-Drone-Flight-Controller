from machine import Pin
import esp32
import time

# =========================
# Motor / GPIO configuration
# =========================

MOTOR_PINS = {
    1: 25,
    2:27,
    3: 26, 
    4: 14
}

RMT_RESOLUTION = 20_000_000

T0_HIGH = 25
T0_LOW = 42

T1_HIGH = 50
T1_LOW = 17


# =========================
# Create RMT channels
# =========================

motors = {}

for channel, (motor, pin_number) in enumerate(MOTOR_PINS.items()):

    motors[motor] = esp32.RMT(
        channel,
        pin=Pin(pin_number),
        resolution_hz=RMT_RESOLUTION
    )

    print(
        "Motor",
        motor,
        "GPIO",
        pin_number,
        "ready"
    )


# =========================
# DShot packet
# =========================

def make_dshot_packet(throttle, telemetry=False):

    throttle &= 0x7FF

    value = (throttle << 1) | int(telemetry)

    checksum = (
        value ^
        (value >> 4) ^
        (value >> 8)
    ) & 0x0F

    return (value << 4) | checksum


# =========================
# DShot waveform
# =========================

def dshot_waveform(packet):

    pulses = []

    for i in range(15, -1, -1):

        bit = (packet >> i) & 1

        if bit:
            pulses.extend((T1_HIGH, T1_LOW))
        else:
            pulses.extend((T0_HIGH, T0_LOW))

    return tuple(pulses)


# =========================
# Send to one motor
# =========================

def send_motor(motor, throttle):

    packet = make_dshot_packet(throttle)

    motors[motor].write_pulses(
        dshot_waveform(packet),
        1
    )


# =========================
# Send to all motors
# =========================

def send_all(throttle):

    for motor in motors:
        send_motor(motor, throttle)


# =========================
# Initialize ESCs
# =========================

print("Initializing all ESCs...")

start = time.ticks_ms()

while time.ticks_diff(time.ticks_ms(), start) < 3000:

    send_all(0)

    time.sleep_us(1000)


print("All ESCs initialized.")
print("Throttle = 0")

print("Testing M1 at throttle 100")

start = time.ticks_ms()

while time.ticks_diff(time.ticks_ms(), start) < 3000:
    send_motor(4, 100)
    time.sleep_us(1000)

print("Stopping M1")
send_motor(4, 0)