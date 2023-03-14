""""
sierralobo_mc3419.py

circuitpython library for MC3419

Caden H.
"""
import time
from micropython import const
from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_register.i2c_bits import ROBits, RWBits
from adafruit_register.i2c_bit import ROBit, RWBit
from adafruit_register.i2c_struct import UnaryStruct
from typing import Tuple

_MC3419_ADDR_DEFAULT: int = const(0x0)
_MC3419_WAKE = const(0x01)
_MC3419_STBY = const(0x00)

_MC3419_DEV_STAT_REG = const(0x05)
_MC3419_MODE_REG = const(0x07)
_MC3419_SR_REG = const(0x08)
_MC3419_XOUT_L_REG = const(0x0D)
_MC3419_STATUS_REG = const(0x13)
_MC3419_RANGE_REG = const(0x20)
_MC3419_RATE_2_REG = const(0x30)

class MXC6655:
    """Driver for the MMC5603 3-axis magnetometer."""

    _state_read = ROBits(2, _MC3419_DEV_STAT_REG, 0)
    _state_write = RWBits(2, _MC3419_MODE_REG, 0)
    _i2c_wdt = RWBits(2, _MC3419_MODE_REG, 4) # further testing with this may be required
    _sr = UnaryStruct(_MC3419_SR_REG, "<B") # internal sample rate
    _new_data = ROBit(_MC3419_STATUS_REG, 7)
    _range = RWBits(3, _MC3419_RANGE_REG, 4)
    _lpf_en = RWBit(_MC3419_RANGE_REG, 3)
    _lpf_bw = RWBits(3, _MC3419_RANGE_REG, 0)
    _dec_mode_rate = RWBits(4, _MC3419_RATE_2_REG, 0) #sets output data rate, 0x0 for = internal sample rate

    def __init__(self, i2c_bus, addr):
        pass

    @property
    def wake(self) -> bool:
        """
        property for if the chip is in wake mode.
            wake needs to be set true for the device to sample.
            cannot write to any other registers besides mode when wake
        """
        return self._state_read == _MC3419_WAKE

    @wake.setter
    def wake(self, value: bool) -> None:
        if value != self.wake:
            if value:
                self._state_write = _MC3419_WAKE
            else:
                self._state_write = _MC3419_STBY
            