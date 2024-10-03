# DHT11/DHT22 driver for MicroPython on ESP8266
# MIT license; Copyright (c) 2016 Damien P. George

import sys
import machine

if hasattr(machine, "dht_readinto"):
    from machine import dht_readinto
elif sys.platform.startswith("esp"):
    from esp import dht_readinto
elif sys.platform == "pyboard":
    from pyb import dht_readinto
else:
    dht_readinto = __import__(sys.platform).dht_readinto

del machine


class DHTBase:
    def __init__(self, pin):
        self.pin = pin
        self.buf = bytearray(5)

    def measure(self):
        buf = self.buf
        dht_readinto(self.pin, buf)
        if (buf[0] + buf[1] + buf[2] + buf[3]) & 0xFF != buf[4]:
            raise Exception("checksum error")


class DHT11(DHTBase):
    def humidity(self):
        return self.buf[0]

    def temperature(self):
        return self.buf[2]


class DHT22(DHTBase):
    def humidity(self):
        return (self.buf[0] << 8 | self.buf[1]) * 0.1

    def temperature(self):
        t = ((self.buf[2] & 0x7F) << 8 | self.buf[3]) * 0.1
        if self.buf[2] & 0x80:
            t = -t
        return t


class DHT:
    DHT11 = 0x11
    DHT22 = 0x22

    def __init__(self, pin, dht):
        self.dht = dht
        self.pin = pin
        self.buf = bytearray(5)

    def get_measure_results(self):
        buf = self.buf
        dht_readinto(self.pin, buf)
        h_bytes = buf[0:2]
        h_raw = int.from_bytes(h_bytes, "little")
        t_bytes = buf[2:4]
        t_raw = int.from_bytes(t_bytes, "little")
        if self.dht == self.DHT11:
            h_val = h_raw
            t_val = t_raw & 0x7FFF
        elif self.dht == self.DHT22:
            h_val = h_raw / 10
            t_val = (t_raw & 0x7FFF) / 10
            if t_raw & 8000:
                t_raw = -(t_raw & 0x7FFF)
                t_val = -t_val
        else:
            h_val = None
            t_val = None
        checksum = (buf[0] + buf[1] + buf[2] + buf[3]) & 0xFF
        return t_raw, round(t_val, 2), h_raw, round(h_val, 2), bool(checksum == buf[4])
