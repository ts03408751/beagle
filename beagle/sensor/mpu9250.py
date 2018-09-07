from ..header import *
import smbus
import time
import numpy as np

__all__ = ['MPU']


class MPU(object):

    def __init__(self, imu_bus=IMU_BUS, mpu_addr=MPU_DEFAULT_I2C_ADDR):
        self._bus = smbus.SMBus(imu_bus)
        self._addr = mpu_addr

        self.__reset_mpu()
        self.__check_who_am_i()

        self._bus.write_byte_data(self._addr, SMPLRT_DIV, 0x00)
        self.__set_accel_fsr(ACCEL_FSR_8G)
        self.__set_gyro_fsr(GYRO_FSR_2000DPS)
        self.__set_accel_dlpf(ACCEL_DLPF_184)
        self.__set_gyro_dlpf(GYRO_DLPF_184)

        time.sleep(0.1)

    def __reset_mpu(self):
        # write the reset bit
        self._bus.write_byte_data(self._addr, PWR_MGMT_1, H_RESET)

        # wait and try again
        time.sleep(0.1)
        self._bus.write_byte_data(self._addr, PWR_MGMT_1, H_RESET)
        time.sleep(0.1)

        return 0

    def __check_who_am_i(self):
        c = self._bus.read_byte_data(self._addr, 0)
        if c != 0x71:
            return -1

        return 0

    def __set_accel_fsr(self, fsr):
        if fsr == ACCEL_FSR_2G:
            c = ACCEL_FSR_CFG_2G
            self._accel_to_ms2 = 9.80665 * 2.0 / 32768.0
        elif fsr == ACCEL_FSR_4G:
            c = ACCEL_FSR_CFG_4G
            self._accel_to_ms2 = 9.80665 * 4.0 / 32768.0
        elif fsr == ACCEL_FSR_8G:
            c = ACCEL_FSR_CFG_8G
            self._accel_to_ms2 = 9.80665 * 8.0 / 32768.0
        elif fsr == ACCEL_FSR_16G:
            c = ACCEL_FSR_CFG_16G
            self._accel_to_ms2 = 9.80665 * 16.0 / 32768.0
        else:
            return -1

        self._bus.write_byte_data(self._addr, ACCEL_CONFIG, c)
        return 0

    def __set_gyro_fsr(self, fsr):
        if fsr == GYRO_FSR_250DPS:
            c = GYRO_FSR_CFG_250 | FCHOICE_B_DLPF_EN
            self._gyro_to_degs = 250.0 / 32768.0
        elif fsr == GYRO_FSR_500DPS:
            c = GYRO_FSR_CFG_500 | FCHOICE_B_DLPF_EN
            self._gyro_to_degs = 500.0 / 32768.0
        elif fsr == GYRO_FSR_1000DPS:
            c = GYRO_FSR_CFG_1000 | FCHOICE_B_DLPF_EN
            self._gyro_to_degs = 1000.0 / 32768.0
        elif fsr == GYRO_FSR_2000DPS:
            c = GYRO_FSR_CFG_2000 | FCHOICE_B_DLPF_EN
            self._gyro_to_degs = 2000.0 / 32768.0
        else:
            return -1

        self._bus.write_byte_data(self._addr, GYRO_CONFIG, c)
        return 0

    def __set_accel_dlpf(self, dlpf):
        c = ACCEL_FCHOICE_1KHZ | BIT_FIFO_SIZE_1024
        if dlpf == ACCEL_DLPF_OFF:
            c = ACCEL_FCHOICE_4KHZ | BIT_FIFO_SIZE_1024
        elif dlpf == ACCEL_DLPF_460:
            c |= 0
        elif dlpf == ACCEL_DLPF_184:
            c |= 1
        elif dlpf == ACCEL_DLPF_92:
            c |= 2
        elif dlpf == ACCEL_DLPF_41:
            c |= 3
        elif dlpf == ACCEL_DLPF_20:
            c |= 4
        elif dlpf == ACCEL_DLPF_10:
            c |= 5
        elif dlpf == ACCEL_DLPF_5:
            c |= 6
        else:
            return -1

        self._bus.write_byte_data(self._addr, ACCEL_CONFIG_2, c)
        return 0

    def __set_gyro_dlpf(self, dlpf):
        c = FIFO_MODE_REPLACE_OLD
        if dlpf == GYRO_DLPF_OFF:
            c |= 7
        elif dlpf == GYRO_DLPF_250:
            c |= 0
        elif dlpf == GYRO_DLPF_184:
            c |= 1
        elif dlpf == GYRO_DLPF_92:
            c |= 2
        elif dlpf == GYRO_DLPF_41:
            c |= 3
        elif dlpf == GYRO_DLPF_20:
            c |= 4
        elif dlpf == GYRO_DLPF_10:
            c |= 5
        elif dlpf == GYRO_DLPF_5:
            c |= 6
        else:
            return -1

        self._bus.write_byte_data(self._addr, CONFIG, c)
        return 0

    def mpu_read_accel(self):
        raw = self._bus.read_i2c_block_data(self._addr, ACCEL_XOUT_H, 6)

        # Turn the MSB and LSB into a signed 16-bit value
        ax_raw = np.int16(np.uint16(raw[0] << 8) | raw[1])
        ay_raw = np.int16(np.uint16(raw[2] << 8) | raw[3])
        az_raw = np.int16(np.uint16(raw[4] << 8) | raw[5])

        # Fill in real unit values and apply calibration
        ax = ax_raw * self._accel_to_ms2 / 1.0
        ay = ay_raw * self._accel_to_ms2 / 1.0
        az = az_raw * self._accel_to_ms2 / 1.0

        # Format number for necessary precision
        ax = round(ax, 4)
        ay = round(ay, 4)
        az = round(az, 4)

        return {'ax': ax, 'ay': ay, 'az': az}

    def mpu_read_gyro(self):
        raw = self._bus.read_i2c_block_data(self._addr, GYRO_XOUT_H, 6)

        # Turn the MSB and LSB into a signed 16-bit value
        gx_raw = np.int16(np.uint16(raw[0] << 8) | raw[1])
        gy_raw = np.int16(np.uint16(raw[2] << 8) | raw[3])
        gz_raw = np.int16(np.uint16(raw[4] << 8) | raw[5])

        # Fill in real unit values and apply calibration
        gx = gx_raw * self._gyro_to_degs / 1.0
        gy = gy_raw * self._gyro_to_degs / 1.0
        gz = gz_raw * self._gyro_to_degs / 1.0

        # Format number for necessary precision
        gx = round(gx, 4)
        gy = round(gy, 4)
        gz = round(gz, 4)

        return {'gx': gx, 'gy': gy, 'gz': gz}
