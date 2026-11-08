# tcs3472.py
# A MicroPython library for the TCS3472 light sensing chip
# https://github.com/tti0/tcs3472-micropython

# Copyright (c) 2021 tti0
# Licensed under the MIT License

import struct

class tcs3472:
    def __init__(self, bus, address=0x29):
        """Set up the colour sensor over I2C."""
        self._bus = bus
        self._i2c_address = address
        
        if type(bus) == "<class 'SoftI2C'>":
            self._bus.start()
        
        self._bus.writeto(self._i2c_address, b'\x80\x03')
        self._bus.writeto(self._i2c_address, b'\x81\x2b')

    def scaled(self):
        """Return RGB readings scaled against the clear channel."""
        crgb = self.raw()
        if crgb[0] > 0:
            return tuple(float(x) / crgb[0] for x in crgb[1:])

        return (0,0,0)

    def rgb(self):
        """Return RGB channel values scaled to 0–255."""
        return tuple(int(x * 255) for x in self.scaled())

    def light(self):
        """Read the clear-light channel."""
        return self.raw()[0]
    
    def brightness(self, level=65.535):
        """Return the clear-light reading as a brightness value."""
        return int((self.light() / level))

    def valid(self):
        """Check whether the sensor has a valid reading."""
        self._bus.writeto(self._i2c_address, b'\x93')
        return self._bus.readfrom(self._i2c_address, 1)[0] & 1

    def raw(self):
        """Read the clear, red, green, and blue channels."""
        self._bus.writeto(self._i2c_address, b'\xb4')
        return struct.unpack("<HHHH", self._bus.readfrom(self._i2c_address, 8))