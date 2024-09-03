# Micropython driver for temperature/humidity/pressure sensors BME280

# Example usage on picoboard:
# @code{.py}
#    from machine import Pin, I2C
#    from bme280 import BME280
#    import time
#    i2c = I2C(0, scl=Pin(5), sda=Pin(4))
#    sensor = BME280(i2c)
#    sensor.start_measure()
#    while True:
#        time.sleep(0.01)
#        t_raw, t_val, h_raw, h_val, p_raw, p_val = sensor.get_measure_results()
#        if t_raw is not None:
#            break
#    print(f'{t_raw}, {t_val} °C, {h_raw}, {h_val} %RH, {p_raw}, {p_val} hPa')
# @endcode
#

from machine import Pin, I2C
import time


class BME280:
    # Init SHT
    # @param i2c  I2C interface
    # @param addr I2C addr (default = 0x76)
    def __init__(self, i2c, addr=0x76):
        self.i2c = i2c
        self.i2c_addr = addr
        # Settings:
        # Sensor mode = forced mode
        # Oversampling settings = pressure * 1, temperature * 1, humidity * 1
        # IIR filter settings = filter off
        self._write(0xF4, 0x24)
        self._write(0xF2, 0x01)
        self._write(0xF5, 0x00)
        self._get_compensation_params()

    # Read data from register
    # @param addr register address
    # @param len  number of bytes (default: 1)
    # @return register value as int
    def _read(self, addr, len=1, byteorder="big"):
        bytes = self.i2c.readfrom_mem(self.i2c_addr, addr, len)
        return int.from_bytes(bytes, byteorder)

    # Write data into register
    # @param addr register address
    # @param data register value
    def _write(self, addr, data):
        self.i2c.writeto_mem(self.i2c_addr, addr, bytes([int(data)]))

    def _get_compensation_params(self):
        l = "little"
        self.dig_T1 = self._read(0x88, 2, l)
        self.dig_T2 = self._read(0x8A, 2, l)
        self.dig_T3 = self._read(0x8C, 2, l)
        self.dig_P1 = self._read(0x8E, 2, l)
        self.dig_P2 = self._read(0x90, 2, l)
        self.dig_P3 = self._read(0x92, 2, l)
        self.dig_P4 = self._read(0x94, 2, l)
        self.dig_P5 = self._read(0x96, 2, l)
        self.dig_P6 = self._read(0x98, 2, l)
        self.dig_P7 = self._read(0x9A, 2, l)
        self.dig_P8 = self._read(0x9C, 2, l)
        self.dig_P9 = self._read(0x9E, 2, l)
        self.dig_H1 = self._read(0xA1)
        self.dig_H2 = self._read(0xE1, 2, l)
        self.dig_H3 = self._read(0xE3)
        E5 = self._read(0xE5)
        self.dig_H4 = self._read(0xE4) << 4 | E5 & 0x0F
        self.dig_H5 = (E5 >> 4) << 8 | self._read(0xE6)
        self.dig_H6 = self._read(0xE7)

    # Start measurement
    # @return None
    def start_measure(self):
        regval = self._read(0xF4)
        self._write(0xF4, regval | 0x01)

    # Get the measurement values
    # @details
    #   As long as no values available all return parameter are None.
    #   If at least one value not equal None are returned the measurement has been completed
    #   and needs to be restarted again for a new measurement.
    # @return temperature[raw], temperature[°C], humidity[raw], humidity[%RH], pressure[raw], pressure[hPa]
    def get_measure_results(self):
        status = self._read(0xF3)
        if status == 0x00:
            t_raw = self._read(0xFA, 3) >> 4
            var1 = (((t_raw >> 3) - (self.dig_T1 << 1)) * self.dig_T2) >> 11
            # var2 = (((((t_raw >> 4) - self.dig_T1) * ((t_raw >> 4) - self.dig_T1)) >> 12) * self.dig_T3) >> 14
            var2 = (((((t_raw >> 4) - self.dig_T1) ** 2) >> 12) * self.dig_T3) >> 14
            t_fine = var1 + var2
            t_int = (t_fine * 5 + 128) >> 8
            t_float = round(t_int / 100, 2)
            #
            h_raw = self._read(0xFD, 2)
            var1 = t_fine - 76800
            var1 = (
                (((h_raw << 14) - (self.dig_H4 << 20) - (self.dig_H5 * var1)) + 16384) >> 15
            ) * (
                (
                    (
                        (
                            (((var1 * self.dig_H6) >> 10) * (((var1 * self.dig_H3) >> 11) + 32768))
                            >> 10
                        )
                        + 2097152
                    )
                    * self.dig_H2
                    + 8192
                )
                >> 14
            )
            # var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * self.dig_H1) >> 4))
            var1 = var1 - (((((var1 >> 15) ** 2) >> 7) * self.dig_H1) >> 4)
            if var1 < 0 or var1 > 419430400:
                h_float = None
            else:
                h_int = var1 >> 12
                h_float = round(h_int / 1024, 2)
            #
            p_raw = self._read(0xF7, 3) >> 4
            p_float = None
            var1 = t_fine - 128000
            var2 = var1 * var1 * self.dig_P6
            var2 = var2 + ((var1 * self.dig_P5) << 17)
            var2 = var2 + (self.dig_P4 << 35)
            var1 = ((var1 * var1 * self.dig_P3) >> 8) + ((var1 * self.dig_P2) << 12)
            var1 = ((1 << 47) + var1) * self.dig_P1 >> 33
            if var1 != 0:
                p_int = 1048576 - p_raw
                p_int = (((p_int << 31) - var2) * 3125) // var1
                var1 = (self.dig_P9 * (p_int >> 13) * (p_int >> 13)) >> 25
                var2 = (self.dig_P8 * p_int) >> 19
                p_int = ((p_int + var1 + var2) >> 8) + (self.dig_P7 << 4)
                p_float = round(p_int / 25600, 2)
            return t_raw, t_float, h_raw, h_float, p_raw, p_float
        else:
            # OSError: [Errno 5] EIO as long as measurement has not completed
            return None, None, None, None, None, None
