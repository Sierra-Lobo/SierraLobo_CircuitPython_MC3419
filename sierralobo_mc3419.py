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
from adafruit_register.i2c_struct import ROUnaryStruct
from typing import Tuple

_MC3419_ADDR_DEFAULT: int = const(0x0)
_MC3419_CHIP_ID = (0xA4)
_MC3419_STATE_WAKE = const(0b01)
_MC3419_STATE_STBY = const(0b00)

_MC3419_RANGE_2G = const(0b000)
_MC3419_RANGE_4G = const(0b001)
_MC3419_RANGE_8G = const(0b010)
_MC3419_RANGE_12G = const(0b011)
_MC3419_RANGE_16G = const(0b100)

_MC3419_SENS_2G = const(16384)
_MC3419_SENS_4G = const(16384)
_MC3419_SENS_8G = const(16384)
_MC3419_SENS_12G = const(16384)
_MC3419_SENS_16G = const(16384)

_MC3419_BW_DIV4P255 = const(0b001)
_MC3419_BW_DIV6 = const(0b010)
_MC3419_BW_DIV12 = const(0b011)
_MC3419_BW_DIV16 = const(0b101)

_MC3419_IDR_50Hz = const(0b000)
_MC3419_IDR_100Hz = const(0b001)
_MC3419_IDR_125Hz = const(0b010)
_MC3419_IDR_200Hz = const(0b011)
_MC3419_IDR_250Hz = const(0b100)
_MC3419_IDR_500Hz = const(0b101)
_MC3419_IDR_1000Hz = const(0b110)
_MC3419_IDR_2000Hz = const(0b111)

_MC3419_DEC_OFF = const(0b0000)
_MC3419_DEC_2 = const(0b0001)
_MC3419_DEC_4 = const(0b0010)
_MC3419_DEC_5 = const(0b0011)
_MC3419_DEC_8 = const(0b0100)
_MC3419_DEC_10 = const(0b0101)
_MC3419_DEC_16 = const(0b0110)
_MC3419_DEC_20 = const(0b0111)
_MC3419_DEC_40 = const(0b1000)
_MC3419_DEC_67 = const(0b1001)
_MC3419_DEC_80 = const(0b1010)
_MC3419_DEC_100 = const(0b1011)
_MC3419_DEC_200 = const(0b1100)
_MC3419_DEC_250 = const(0b1101)
_MC3419_DEC_500 = const(0b1110)
_MC3419_DEC_1000 = const(0b1111)

_MC3419_CHIP_ID_REG = const(0x18)
_MC3419_DEV_STAT_REG = const(0x05)
_MC3419_MODE_REG = const(0x07)
_MC3419_SR_REG = const(0x08)
_MC3419_XOUT_L_REG = const(0x0D)
_MC3419_STATUS_REG = const(0x13)
_MC3419_RANGE_REG = const(0x20)
_MC3419_RATE_2_REG = const(0x30)

class MXC6655:
    """Driver for the MMC5603 3-axis magnetometer."""

    _chip_id = ROUnaryStruct(_MC3419_CHIP_ID)
    _state_read = ROBits(2, _MC3419_DEV_STAT_REG, 0)
    _state_write = RWBits(2, _MC3419_MODE_REG, 0)
    _i2c_wdt = RWBits(2, _MC3419_MODE_REG, 4) # further testing with this may be required
    _idr = RWBits(3, _MC3419_SR_REG, 0) # internal sample rate
    _new_data = ROBit(_MC3419_STATUS_REG, 7)
    _range = RWBits(3, _MC3419_RANGE_REG, 4)
    _lpf_en = RWBit(_MC3419_RANGE_REG, 3)
    _lpf_bw = RWBits(3, _MC3419_RANGE_REG, 0)
    _dec = RWBits(4, _MC3419_RATE_2_REG, 0) #sets output data rate, 0x0 for = internal sample rate

    def __init__(self, i2c_bus, address: int = _MC3419_ADDR_DEFAULT) -> None:
        self.i2c_device = I2CDevice(i2c_bus, address)
        if self._chip_id != _MC3419_CHIP_ID:
            raise("Failed to find MMC5603.")
        self.reset()
        self._buffer = bytearray(6)

    def reset(self) -> None:
        """Reset the sensor to the default state set by the library"""
        if self.wake: # if WAKE, put self into STBY
            self.wake = False
        self.idr = _MC3419_IDR_500Hz
        self.dec = _MC3419_DEC_1000
        self.range = _MC3419_RANGE_2G
        self.lpf_bw = _MC3419_BW_DIV16
        self.lpf = True
        self._i2c_wdt = 0b11

    @property
    def acceleration(self) -> Tuple[float, float, float]:
        """The processed acceration sensor values.
        A 3-tuple of X, Y, Z axis values in g that are signed floats.
        """
        if not self.wake:
            raise RuntimeError("MC3419 only updates acceleation when in WAKE mode")
        self._buffer[0] = _MC3419_XOUT_L_REG
        with self.i2c_device as i2c:
            i2c.write_then_readinto(self._buffer, self._buffer, out_end=1)
        x = self._buffer[1] << 8 | self._buffer[0] 
        y = self._buffer[3] << 8 | self._buffer[2] 
        z = self._buffer[5] << 8 | self._buffer[4]
        # fix center offsets
        x -= 1 << 16
        y -= 1 << 16
        z -= 1 << 16
        # scale to g by LSB in datasheet
        temp = self.sensitivity
        x *= temp
        y *= temp
        z *= temp
        return (x, y, z)

    @property
    def wake(self) -> bool:
        """
        property for if the chip is in wake mode.
            wake needs to be set true for the device to sample.
            cannot write to any other registers besides mode when wake
        """
        return self._state_read == _MC3419_STATE_WAKE

    @wake.setter
    def wake(self, value : bool) -> None:
        if value != self.wake:
            if value:
                self._state_write = _MC3419_STATE_WAKE
            else:
                self._state_write = _MC3419_STATE_STBY
            time.sleep(0.001)

    @property
    def range(self) -> int:
        """MC3419 sensor range, 0b000 to 0b111.
        see above _MC3419_RANGE_xG constants"""
        return self._range
    
    @range.setter
    def range(self, value : int) -> None:
        if self.wake:
            raise RuntimeError("MC3419 range must be in STBY mode to set range")
        if not (0 <= value <= 4):
            raise RuntimeError("MC3419 range must be between 0b000 and 0b100")
        self._range = value

    @property
    def sensitivity(self) -> int:
        """MC3419 datasheet sensitivity units: LSB/g
        any better way to code this while keeping the const() declarations?
        if the const() is not used, these values will be held in RAM..."""
        temp = self.range
        if self.range == _MC3419_RANGE_2G:
            return _MC3419_SENS_2G
        if self.range == _MC3419_RANGE_4G:
            return _MC3419_SENS_4G
        if self.range == _MC3419_RANGE_8G:
            return _MC3419_SENS_8G
        if self.range == _MC3419_RANGE_12G:
            return _MC3419_SENS_12G
        if self.range == _MC3419_RANGE_16G:
            return _MC3419_SENS_16G
        
    @property
    def idr(self) -> int:
        return self._idr
    
    @idr.setter
    def idr(self, value : int) -> None:
        """MC3419 internal data rate. 
        also referred to SR or sample rate in datasheet.
        if self.idr_div = 0 this is also the rate at which 
        the sensor self-refreshes its data registers"""
        if self.wake:
            raise RuntimeError("MC3419 must be in STBY mode to set IDR")
        if not (0 <= value <= 7):
            raise RuntimeError("MC3419 idr must be between 0b000 and 0b111")
        self._idr = value

    @property
    def dec(self) -> int:
        return self._dec
    
    @dec.setter
    def dec(self, value : int) -> None:
        """MC3419 internal data rate decimation. 
        If 0, decimation is disabed and accel output registers 
        update at IDR frequency.
        otherwise, the IDR is divided by the value corresponding 
        to the consts defined above."""
        if self.wake:
            raise RuntimeError("MC3419 must be in STBY mode to set decimation")
        if not (0 <= value <= 15):
            raise RuntimeError("MC3419 decimation must be between 0b0000 and 0b1111")
        self._dec = value

    @property
    def lpf_en(self) -> bool:
        """MC3419 low pass filter enable bit."""
        return self._lpf_en
    
    @lpf_en.setter
    def lpf_en(self, value : bool) -> None:
        if self.wake:
            raise RuntimeError("MC3419 must be in STBY mode to set LPF_EN")
        self._lpf_en = value

    @property
    def lpf_bw(self) -> int:
        return self._lpf_bw
    
    @lpf_bw.setter
    def lpf_bw(self, value : int) -> None:
        if self.wake:
            raise RuntimeError("MC3419 must be in STBY mode to set LPF_BW")
        if not (1 <= value <= 3) or (value == 5):
            raise RuntimeError("MC3419 LPF_BW must be between 0b001 to 0b011, or be equal to 0b101")
        self._lpf_bw = value
