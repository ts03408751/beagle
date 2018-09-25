from ..header import *
import smbus
import time
import numpy as np

__all__ = ['BMP']


class BMP(object):

    def __init__(self, bus=BMP_BUS, addr=BMP280_ADDR, oversample_=BMP_OVERSAMPLE_16, filter_=BMP_FILTER_OFF):
        self._bus = smbus.SMBus(bus)
        self._addr = addr

        self._bmp280_cal = {
            'dig_T1': np.uint16(),
            'dig_T2': np.int16(),
            'dig_T3': np.int16(),
            'dig_P1': np.uint16(),
            'dig_P2': np.int16(),
            'dig_P3': np.int16(),
            'dig_P4': np.int16(),
            'dig_P5': np.int16(),
            'dig_P6': np.int16(),
            'dig_P7': np.int16(),
            'dig_P8': np.int16(),
            'dig_P9': np.int16(),
            'sea_level_pa': np.float64(),
        }

        self.__bmp_init(oversample_, filter_)

    def __bmp_init(self, oversample_, filter_):
        # reset the barometer
        self._bus.write_byte_data(self._addr, BMP280_RESET_REG, BMP280_RESET_WORD)

        # check the chip ID register
        c = self._bus.read_byte_data(self._addr, BMP280_CHIP_ID_REG)
        if c != BMP280_CHIP_ID:
            return -1

        # set up the bmp measurement control register settings
        # no temperature oversampling,  normal continuous read mode
        c = BMP_MODE_NORMAL
        c |= BMP_TEMP_OVERSAMPLE_1
        c |= oversample_

        # write the measurement control register
        self._bus.write_byte_data(self._addr, BMP280_CTRL_MEAS, c)

        # set up the filter config register
        c = BMP280_TSB_0
        c |= filter_
        self._bus.write_byte_data(self._addr, BMP280_CONFIG, c)

        # keep checking the status register until the NVM calibration is ready
        # after a short wait
        i = 0
        loop_cond = True
        while loop_cond:
            time.sleep(0.2)
            c = self._bus.read_byte_data(self._addr, BMP280_STATUS_REG)
            if i > 10:
                return -1
            i += 1
            if (c & BMP280_IM_UPDATE_STATUS) == 0:
                loop_cond = False

        # retrieve the factory NVM calibration data all in one go
        buf = self._bus.read_i2c_block_data(self._addr, BMP280_DIG_T1, 24)

        # save calibration in useful format
        self._bmp280_cal['dig_T1'] = np.uint16((buf[1] << 8) | buf[0])
        self._bmp280_cal['dig_T2'] = np.uint16((buf[3] << 8) | buf[2])
        self._bmp280_cal['dig_T3'] = np.uint16((buf[5] << 8) | buf[4])
        self._bmp280_cal['dig_P1'] = np.uint16((buf[7] << 8) | buf[6])
        self._bmp280_cal['dig_P2'] = np.uint16((buf[9] << 8) | buf[8])
        self._bmp280_cal['dig_P3'] = np.uint16((buf[11] << 8) | buf[10])
        self._bmp280_cal['dig_P4'] = np.uint16((buf[13] << 8) | buf[12])
        self._bmp280_cal['dig_P5'] = np.uint16((buf[15] << 8) | buf[14])
        self._bmp280_cal['dig_P6'] = np.uint16((buf[17] << 8) | buf[16])
        self._bmp280_cal['dig_P7'] = np.uint16((buf[19] << 8) | buf[18])
        self._bmp280_cal['dig_P8'] = np.uint16((buf[21] << 8) | buf[20])
        self._bmp280_cal['dig_P9'] = np.uint16((buf[23] << 8) | buf[22])

        # use default pressure for now unless user sets it otherwise
        self._bmp280_cal['sea_level_pa'] = DEFAULT_SEA_LEVEL_PA

        time.sleep(0.5)

    def bmp_read(self):
        raw = self._bus.read_i2c_block_data(self._addr, BMP280_PRESSURE_MSB, 6)

        # run the numbers, thanks to Bosch for putting this code in their datasheet
        adc_P = (raw[0] << 12) | (raw[1] << 4) | (raw[2] >> 4)
        adc_T = (raw[3] << 12) | (raw[4] << 4) | (raw[5] >> 4)

        var1 = (((adc_T >> 3) - (self._bmp280_cal['dig_T1'] << 1)) *
                self._bmp280_cal['dig_T2']) >> 11
        var2 = (((((adc_T >> 4) - self._bmp280_cal['dig_T1']) *
                  ((adc_T >> 4) - self._bmp280_cal['dig_T1'])) >> 12) *
                self._bmp280_cal['dig_T3']) >> 14

        t_fine = var1 + var2
        T = (t_fine * 5 + 128) >> 8
        temp_c = T / 100.0

        var3 = np.int64(t_fine) - 128000
        var4 = var3 * var3 * self._bmp280_cal['dig_P6']
        var4 = var4 + ((var3 * self._bmp280_cal['dig_P5']) << 17)
        var4 = var4 + (self._bmp280_cal['dig_P4'] << 35)
        var3 = (var3 * var3 * self._bmp280_cal['dig_P3'] >> 8) +\
               ((var3 * self._bmp280_cal['dig_P2']) << 12)
        var3 = ((1 << 47) + var3) * self._bmp280_cal['dig_P1'] >> 33

        # avoid exception caused by division by zero
        if var3 == 0:
            return -1

        p = 1048576 - adc_P
        p = (((p << 31) - var4) * 3125) / var3
        var3 = (np.int64(self._bmp280_cal['dig_P9']) * (p >> 13) * (p >> 13)) >> 25
        var4 = (np.int64(self._bmp280_cal['dig_P8']) * p) >> 19

        p = ((p + var3 + var4) >> 8) + (self._bmp280_cal['dig_P7'] << 4)
        pressure_pa = p / 256.0

        alt_m = 44330.0 * (1.0 - pow(pressure_pa / self._bmp280_cal['sea_level_pa'], 0.1903))

        return {'temp': temp_c, 'pressure': pressure_pa / 100.0, 'altitude': alt_m}
