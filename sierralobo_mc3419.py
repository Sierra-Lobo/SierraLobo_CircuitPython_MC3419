""""
sierralobo_mc3419.py

circuitpython library for MC3419

Caden H.
"""

from micropython import const
from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_register.i2c_bits import ROBits, RWBits
from adafruit_register.i2c_bit import ROBit, RWBit
from adafruit_register.i2c_struct import UnaryStruct

class MXC6655:

    _sw_rst = RWBit(INT_1, 4)
    _drdy = RWBit(INT_1, 0)
    _ord = ROBit(STATUS, 4)
    _xout_u = UnaryStruct(XOUT_U, "<B")
    _xout_l = UnaryStruct(XOUT_L, "<B")
    _yout_u = UnaryStruct(YOUT_U, "<B")
    _yout_l = UnaryStruct(YOUT_L, "<B")
    _zout_u = UnaryStruct(ZOUT_U, "<B")
    _zout_l = UnaryStruct(ZOUT_L, "<B")
    _tout = UnaryStruct(TOUT, "<B")
    _pd = RWBit(CONTROL, 0)
    _fsr = RWBits(2, CONTROL, 6)
    _drdye = RWBit(INT_MASK1, 0)
    _who_am_i = ROBit(WHO_AM_I, 0)

    class _BAD_WHO_AM_I(Exception):
        pass

    def __init__(self, i2c_bus, addr):
        self.i2c_device = I2CDevice(i2c_bus, addr, probe=False)
        if not self._who_am_i: raise self._BAD_WHO_AM_I("[ERROR][MXC6655][BAD WHO_AM_I VALUE]: " + str(self._who_am_i))
        self.ON()
    
    def SLEEP(self):
        self._pd = True

    def ON(self): #no power adjustment paramaters for this chip, just on or off.
        self._pd = False
        self._fsr = 0 #1024 LSB/g
        self._drdye = True

    def reset(self):
        self._sw_rst = True

    @property
    def read(self): #device self refreshes @ 100Hz
        #adjust values to accel in g
        out = self.read_raw
        for meas in range(len(out)): #12b 2's complememt form, -2048 to 2048, 
            if (out[meas]>>11 == 1): # if it is 2's convert it to neg
                #flip all 12 bits and add 1
                out[meas] = ((out[meas] ^ 0x0FFF) + 1) * (-1)
            out[meas] = out[meas]/1024
        return(out)

    
    # read function returns touple of floats if it got data, None type otherwise., maybe make this FFFFFF or exact 0? idk since it is gonna be a float
    @property
    def read_raw(self): #device self refreshes @ 100Hz
        x = (self._xout_u << 4) + (self._xout_l >> 4)
        y = (self._yout_u << 4) + (self._yout_l >> 4)
        z = (self._zout_u << 4) + (self._zout_l >> 4)
        out = [x, y, z]
        #reset drdy
        self._drdy = False
        return(out)

    @property
    def temp(self):
        temp = self._tout #8b value =0 @ 25degC, 0.568degC/LSB
        #do adjustment
        if(temp>>7 == 1): #if 2's comp
            temp = ((temp ^ 0xFF) + 1) * (-1)
        temp = (temp * 0.568) + 25
        return(temp)