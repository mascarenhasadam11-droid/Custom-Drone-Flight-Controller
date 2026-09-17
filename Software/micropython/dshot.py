from machine import Pin
import esp32


class DShotMotor:

    # DShot300 timing at 20 MHz
    T0_HIGH = 25
    T0_LOW = 42

    T1_HIGH = 50
    T1_LOW = 17

    def __init__(self, channel, pin):

        self.pin = pin

        self.rmt = esp32.RMT(
            channel,
            pin=Pin(pin),
            resolution_hz=20_000_000
        )

    # -------------------------
    # Create DShot packet
    # -------------------------

    def make_packet(self, throttle):

        throttle = max(0, min(2047, throttle))

        value = throttle << 1

        checksum = (
            value ^
            (value >> 4) ^
            (value >> 8)
        ) & 0x0F

        return (value << 4) | checksum

    # -------------------------
    # Convert packet to pulses
    # -------------------------

    def make_waveform(self, packet):

        pulses = []

        for i in range(15, -1, -1):

            bit = (packet >> i) & 1

            if bit:
                pulses.extend((
                    self.T1_HIGH,
                    self.T1_LOW
                ))
            else:
                pulses.extend((
                    self.T0_HIGH,
                    self.T0_LOW
                ))

        return tuple(pulses)

    # -------------------------
    # Send throttle
    # -------------------------

    def set_throttle(self, throttle):

        packet = self.make_packet(throttle)

        waveform = self.make_waveform(packet)

        self.rmt.write_pulses(
            waveform,
            1
        )

    # -------------------------
    # Stop motor
    # -------------------------

    def stop(self):

        self.set_throttle(0)