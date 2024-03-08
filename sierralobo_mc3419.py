"""
'sierralobo_mc3419.py'

    CircuitPython library for Memsic MC3419 Accelerometer

* Author(s): chillis
* Affiliation(s): Sierra Lobo, Inc.

* Repo Link: https://github.com/Sierra-Lobo/SierraLobo_CircuitPython_MC3419
* MC3419 Datasheet Link:
    https://www.memsic.com/Public/Uploads/uploadfile/files/20220615/MC3419Datasheet(APS-048-0071v1.2).pdf


Implementation Notes:

    Chip Capabilities Implemented:
        * I2C Interface
        * POR
        * Range setting 2-16g
        * Data rate setting 0.5-1000Hz
        * LPF (only fc = idr/4)

    Unimplemented:
        * SPI Interface
        * Device interrupts
        * FIFO
        * Gesture Detection
        * LPF other than fc = IDR/4
"""

import time
from struct import unpack_from
from micropython import const
from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_register.i2c_bits import ROBits, RWBits
from adafruit_register.i2c_bit import ROBit, RWBit
from adafruit_register.i2c_struct import ROUnaryStruct

try:
    from typing import Tuple
    from busio import I2C
except ImportError:
    pass

# Device Consts / Defaults
_MC3419_ADDR_DEFAULT = const(0x0)
_MC3419_CHIP_ID = const(0xA4)
_MC3419_STATE_WAKE = const(0x01)
_MC3419_STATE_STBY = const(0x0)
_MC3419_BITS = const(2**15)  # signed (so not 16)

# Device Registers
_MC3419_DEV_STAT_REG = const(0x05)
_MC3419_CHIP_ID_REG = const(0x18)
_MC3419_RESET_REG = const(0x1C)
_MC3419_MODE_REG = const(0x07)
_MC3419_SR_REG = const(0x08)
_MC3419_XOUT_L_REG = const(0x0D)
_MC3419_STATUS_REG = const(0x13)
_MC3419_RANGE_REG = const(0x20)
_MC3419_RATE_2_REG = const(0x30)

# Device Paramater options, corresponding index is written value.
_MC3419_RANGE_OPTS = (  # in order of device register settings
    const(2),
    const(4),
    const(8),
    const(16),  # yes, this is out of order.
    const(12),
)

_MC3419_IDR_OPTS = (
    # 3 bits, in order of device register settings
    const(25),
    const(50),
    const(62.5),
    const(100),
    const(125),
    const(250),
    const(500),
    const(1000),
)

_MC3419_DECIM_OPTS = (
    # 4 bits, in order of device register settings
    const(1),
    const(2),
    const(4),
    const(5),
    const(8),
    const(10),
    const(16),
    const(40),
    const(67),
    const(80),
    const(100),
    const(200),
    const(250),
    const(500),
    const(1000),
)

# pylint: disable=too-many-instance-attributes
class MC3419:
    """Driver for the MMC3419 3-axis accelerometer."""

    _chip_id = ROUnaryStruct(_MC3419_CHIP_ID_REG, "<B")
    _reset = RWBit(_MC3419_RESET_REG, 6)
    _range = RWBits(3, _MC3419_RANGE_REG, 4)
    _mode_rd = ROBits(2, _MC3419_DEV_STAT_REG, 0)
    _mode_wr = RWBits(2, _MC3419_MODE_REG, 0)
    _rate = RWBits(3, _MC3419_SR_REG, 0)
    _lpf_en = RWBit(_MC3419_RANGE_REG, 3)
    _lpf_bw = RWBits(3, _MC3419_RANGE_REG, 0)
    _dec = RWBits(4, _MC3419_RATE_2_REG, 0)
    _new_data = ROBit(_MC3419_STATUS_REG, 7)

    def __init__(self, i2c_bus: I2C, address: int) -> None:
        """class for MC3419 accelerometer"""
        self.i2c_device = I2CDevice(i2c_bus, address)
        if self._chip_id != _MC3419_CHIP_ID:
            raise RuntimeError(
                "Invalid Chip ID: {}, expected {}".format(
                    hex(self._chip_id), hex(_MC3419_CHIP_ID)
                )
            )
        self._buffer = bytearray(6)
        self.reset()

    def reset(self) -> None:
        """Reset the sensor to the default state set by the library"""
        self.wake = False
        self._reset = True
        time.sleep(0.010)
        self.range = 2
        self.data_rate = 2
        self.lpf = True
        self.wake = True

    @property
    def acceleration(self) -> Tuple[float, float, float]:
        """The processed acceration sensor values.
        A 3-tuple of X, Y, Z axis values in g that are signed floats.
        """
        sens = _MC3419_BITS // self.range
        accel = []
        for axis in self.raw:
            accel.append(float(axis) / sens)  # scale to units 'g'
        return tuple(accel)

    @property
    def raw(self) -> Tuple[int, int, int]:
        """The raw accelerometer sensor values.
        A 3-tuple of X,Y,Z axis values. Ranging from -2^15 to (2^16)-1
        """
        change = bool(not self.wake)
        if change:
            self.wake = True  # mode must be 'stby' to change configuration

        # wait for new data
        while not self._new_data:
            pass

        x = None
        y = None
        z = None

        # read values
        self._buffer[0] = _MC3419_XOUT_L_REG
        with self.i2c_device as i2c:
            i2c.write_then_readinto(self._buffer, self._buffer, out_end=1)

        x = unpack_from("<h", self._buffer, 0)[0]
        y = unpack_from("<h", self._buffer, 2)[0]
        z = unpack_from("<h", self._buffer, 4)[0]

        print(x, y, z)

        if change:
            self.wake = False

        return (x, y, z)

    @property
    def wake(self) -> bool:
        """Property of whether the device is in 'wake' mode or not.

        :return bool: True if device mode is 'wake'
        """
        return self._mode_rd == _MC3419_STATE_WAKE

    @wake.setter
    def wake(self, value: bool) -> None:
        """Set device mode
            'wake' mode is to read sensors
            'stby' mode is for configuration and low power

        :param bool value: True to set mode to 'wake', False for mode to 'stby'
        """
        if value:
            self._mode_wr = _MC3419_STATE_WAKE
        else:
            self._mode_wr = _MC3419_STATE_STBY
        time.sleep(0.001)  # datasheet wake-up time = 3ms

    @property
    def range(self) -> int:
        """Return sensor's range.

        :return int: +/- range; units 'g' (9.8 m/s^2)
        """
        return _MC3419_RANGE_OPTS[self._range]

    @range.setter
    def range(self, value: int) -> None:
        """Set the sensor's range.

        :param int value: a value in _ranges, 2,4,8,12, or 16.
        :raises ValueError: the passed range is not a valid setting.
        """
        # pylint: disable=no-else-raise
        try:
            i = _MC3419_RANGE_OPTS.index(value)
        except ValueError as e:
            raise ValueError("MC3419 range setting invalid") from e
        else:
            change = bool(self.wake)  # mode must be 'stby' to change configuration
            if change:
                self.wake = False
            self._range = i  # write range
            if change:
                self.wake = True

    @property
    def data_rate(self) -> float:
        """get the update rate of the sensor

        :return float: update rate in hz
        """
        return self._idr / self._dec

    @data_rate.setter
    def data_rate(self, rate_hz: float, tol_hz: float = 0.25) -> None:
        """set the update rate of the sensor

        mildly overcomplicated

        :param float rate_hz: desired update rate, between 0.5 and 1000
        :raises ValueError: _description_
        """
        if not 0.5 <= rate_hz <= 1000:
            raise ValueError(f"data_rate setting out of valid range: {rate_hz}")

        idr = 0
        dec = 0
        diff = rate_hz
        for rate in _MC3419_IDR_OPTS:  # find idr and dec
            if rate >= rate_hz:
                temp_dec = _find_dec(rate_hz, rate)
                temp_diff = abs(rate_hz - rate / temp_dec)
                if temp_diff < diff:
                    idr = rate
                    dec = temp_dec
                    diff = temp_diff
                if diff <= tol_hz:
                    break  # break early if eq

        change = bool(self.wake)
        if change:
            self.wake = False  # mode must be 'stby' to change configuration

        self._idr = _MC3419_IDR_OPTS.index(idr)  # write idr
        self._dec = _MC3419_DECIM_OPTS.index(dec)  # write dec

        if change:
            self.wake = True

    @property
    def lpf(self) -> bool:
        """if the device low pass filter is enabled"""
        return self._lpf_en

    @lpf.setter
    def lpf(self, en: bool) -> None:
        """enable the low pass filter, setting fc=idr/4"""
        change = bool(self.wake)
        if change:
            self.wake = False  # mode must be 'stby' to change configuration

        self._lpf_en = en
        if en:
            self._lpf_bw = 0x05  # fc = idr/4

        if change:
            self.wake = True


def _find_dec(rate_target: float, idr: float):
    """find best decimation value at a given idr"""
    best_dec = idr / rate_target
    for i in range(len(_MC3419_DECIM_OPTS) - 1):  # find decimator value at idr
        if _MC3419_DECIM_OPTS[i] <= best_dec <= _MC3419_DECIM_OPTS[i + 1]:
            dif0 = abs(rate_target - idr / _MC3419_DECIM_OPTS[i])
            dif1 = abs(rate_target - idr / _MC3419_DECIM_OPTS[i])
            if dif0 > dif1:
                dec = _MC3419_DECIM_OPTS[i]
            else:
                dec = _MC3419_DECIM_OPTS[i + 1]
    return dec
