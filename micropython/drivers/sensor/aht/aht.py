# Micropython driver for temperature/humidity sensors SHT3x/SHT4x
#
# Example usage on picoboard:
# @code{.py}
#    from machine import Pin, I2C
#    from sht import SHT
#    import time
#    i2c = I2C(0, scl=Pin(5), sda=Pin(4))
#    sensor = SHT(i2c)
#    sensor.start_measure(2)
#    while True:
#        time.sleep(0.01)
#        t_raw, t_val, h_raw, h_val, isvalid = sensor.get_measure_results()
#        if isvalid is not None:
#            break
#    print(f"{t_raw}, {t_val} °C, {h_raw}, {h_val} %RH, {isvalid}")
# @endcode
#

from machine import I2C
import time


class AHT:

    # Init AHT
    # @param i2c  I2C interface
    # @param addr I2C addr (default = 0x38)
    def __init__(self, i2c, addr=0x38):
        self.i2c = i2c
        self.i2c_addr = addr
        # Datasheet:
        # Before reading the temperature and humidity value, get a byte of status
        # word by sending 0x71. If the status word and 0x18 are not equal to 0x18, 
        # initialize the 0x1B, 0x1C, 0x1E registers
        self.i2c.writeto(self.i2c_addr, b"\x71")
        status = self.i2c.readfrom(self.i2c_addr, 1)
        # TODO?
        # no detailed description found
        # Reference: demo code from aosong
        # if (status[0] & 0x18) != 0x18:
        #     JH_Reset_REG(0x1b);
        #     JH_Reset_REG(0x1c);
        #     JH_Reset_REG(0x1e);

    # Verify checksum
    # @param data     data bytes
    # @param checksum received crc
    # @return crc status
    #     0 = crc does not match
    #     1 = crc ok
    def _check_crc(self, data, checksum):
        crc = 0xFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc = crc << 1
        crc = crc & 0xFF
        return checksum == crc

    # Start measurement
    # @return None
    def start_measure(self):
        self.i2c.writeto(self.i2c_addr, b"\x70\xac\x33\x00")

    # Get the measurement values
    # @details
    #   As long as no values available all return parameter are None.
    #   If values not equal None are returned the measurement has been completed
    #   and needs to be restarted again for a new measurement.
    # @return temperature[raw], temperature[°C], humidity[raw], humidity[%RH], valid
    def get_measure_results(self):
        self.i2c.writeto(self.i2c_addr, b"\x71")
        status = self.i2c.readfrom(self.i2c_addr, 1)
        if status[0] & 0x80:
            # measurement still running
            return None, None, None, None, None
        else:
            self.i2c.writeto(self.i2c_addr, b"\x71")
            response = self.i2c.readfrom(self.i2c_addr, 7)
            h_bytes = response[1:4]
            h_raw = int.from_bytes(h_bytes, "big") >> 4
            h_val = (100 * h_raw) / 0x100000
            t_bytes = response[3:6]
            t_raw = int.from_bytes(t_bytes, "big") & 0xFFFFF
            t_val = (200 * t_raw) / 0x100000 - 50
            isvalid = self._check_crc(response[0:6], response[6])
            return t_raw, round(t_val, 2), h_raw, round(h_val, 2), bool(isvalid)

