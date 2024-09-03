# Micropython driver for ambient light sensor TSL2591
#
# Example usage on picoboard:
# @code{.py}
#    from machine import Pin, I2C
#    from tsl2591 import TSL2591
#    import time
#    i2c = I2C(0, scl=Pin(5), sda=Pin(4))
#    sensor = TSL2591(i2c)
#    sensor.start_measure(200, 1)
#    while True:
#        time.sleep(0.01)
#        full_raw, ir_raw = sensor.get_measure_results()
#        if full_raw is not None:
#            break
#    if full_raw != -1:
#      print(f'{full_raw}, {ir_raw}')
#    else:
#      print(f'Mesurement not started.')
# @endcode
#

from machine import I2C


class TSL2591:
    # Register
    ENABLE = 0x00
    CONFIG = 0x01
    STATUS = 0x13
    C0DATA = 0x14
    C1DATA = 0x16

    def __init__(self, i2c, addr=0x29):
        self.i2c = i2c
        self.i2c_addr = addr
        # PowerOn
        self._write(self.ENABLE, 0x01)

    # Read data from register
    # @param addr register address
    # @param len  number of bytes (default: 1)
    # @return register value as int
    def _read(self, addr, len=1):
        cmd = (0xA0 | addr) & 0xFF
        bytes = self.i2c.readfrom_mem(self.i2c_addr, cmd, len)
        return int.from_bytes(bytes, "little")

    # Write data into register
    # @param addr register address
    # @param data register value
    def _write(self, addr, data):
        cmd = (0xA0 | addr) & 0xFF
        self.i2c.writeto_mem(self.i2c_addr, cmd, bytes([int(data)]))

    # Start measurement
    # @param time 100..600, step 100 (time in ms)
    # @param gain 0..3 [Low, Medlium, High, Maximum]
    # @return None
    def start_measure(self, time, gain):
        if time > 600:
            atime = 0x05
        else:
            atime = (time // 100) - 1
        again = (gain & 0x3) << 4
        self._write(self.CONFIG, (atime | again))
        # enable ALS (AEN)
        enable = self._read(0x00)
        self._write(self.ENABLE, (enable | 0x02))

    # Get the measurement value
    # @details
    #   If the measurement has not been started all return parameter has the value -1.
    #   As long as no values available all return parameter are None.
    #   After reading the measurement value the ALS function will be disabled and
    #   the measurement has to be start again
    # @return ch0 (full), ch1 (ir)
    def get_measure_results(self):
        if self._read(self.ENABLE) & 0x02:
            # started
            if self._read(self.STATUS) & 0x01:
                full_raw = self._read(self.C0DATA, 2)
                ir_raw = self._read(self.C1DATA, 2)
                # disable ALS (AEN)
                enable = self._read(self.ENABLE)
                self._write(self.ENABLE, (enable & 0xFD))
                return full_raw, ir_raw
            else:
                return None, None
        else:
            return -1, -1
