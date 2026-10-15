"""
Minimal PCA9685 I2C PWM driver, used for the kick servo. Per
pi_controller_spec.md, the servo's PWM comes from this dedicated chip
rather than a Pi GPIO pin — sidesteps the Pi 5 RP1 GPIO-timing question
entirely for the servo, independent of whatever's decided for the ESCs.

Requires smbus2 (sudo apt install python3-smbus2) and I2C enabled
(raspi-config -> Interface Options -> I2C). Default address 0x40 does not
conflict with a BNO055 IMU at 0x28, if/when that's wired up.
"""

import time

_MODE1 = 0x00
_PRESCALE = 0xFE
_LED0_ON_L = 0x06
_PWM_FREQ_HZ = 50  # standard servo frequency (20ms period)
_OSC_CLOCK_HZ = 25_000_000
_SLEEP_BIT = 0x10
_RESTART_BIT = 0x80
_FULL_OFF_BIT = 0x10  # set in the OFF_H register to stop sending pulses entirely


class PCA9685:
    def __init__(self, i2c_address, bus_number=1):
        import smbus2  # imported here so this module still loads without it installed

        self.bus = smbus2.SMBus(bus_number)
        self.address = i2c_address
        self._reset()
        self._set_pwm_freq(_PWM_FREQ_HZ)

    def _reset(self):
        self.bus.write_byte_data(self.address, _MODE1, 0x00)
        time.sleep(0.01)

    def _set_pwm_freq(self, freq_hz):
        prescale = round(_OSC_CLOCK_HZ / (4096 * freq_hz)) - 1
        old_mode = self.bus.read_byte_data(self.address, _MODE1)
        self.bus.write_byte_data(self.address, _MODE1, (old_mode & 0x7F) | _SLEEP_BIT)
        self.bus.write_byte_data(self.address, _PRESCALE, prescale)
        self.bus.write_byte_data(self.address, _MODE1, old_mode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, _MODE1, old_mode | _RESTART_BIT)

    def set_pulse_us(self, channel, pulse_us):
        """Sets a channel's PWM high-time to pulse_us microseconds within
        the 20ms (50Hz) period — the standard RC-servo pulse-width convention."""
        period_us = 1_000_000 / _PWM_FREQ_HZ
        off_tick = max(0, min(4095, round(pulse_us / period_us * 4096)))
        reg = _LED0_ON_L + 4 * channel
        self.bus.write_byte_data(self.address, reg, 0)
        self.bus.write_byte_data(self.address, reg + 1, 0)
        self.bus.write_byte_data(self.address, reg + 2, off_tick & 0xFF)
        self.bus.write_byte_data(self.address, reg + 3, (off_tick >> 8) & 0xF)

    def release(self, channel):
        """Stops sending pulses on this channel entirely (software fail-safe,
        same spirit as gpiozero's Servo.detach() for the thrusters)."""
        reg = _LED0_ON_L + 4 * channel
        self.bus.write_byte_data(self.address, reg + 3, _FULL_OFF_BIT)
