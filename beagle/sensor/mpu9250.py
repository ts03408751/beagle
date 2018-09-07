from ..header import *
import smbus
import time
import numpy as np

__all__ = ['MPU']


class MPU(object):

    def __init__(self, imu_bus=IMU_BUS, mpu_addr=MPU_DEFAULT_I2C_ADDR, sample_rate=200):
        self._bus = smbus.SMBus(imu_bus)
        self._addr = mpu_addr
        self._dmp_en = 1
        self._bypass_en = 0
        self._mag_factory_adjust = [0.0] * 3

        self.__reset_mpu()
        self.__check_who_am_i()

        self._bus.write_byte_data(self._addr, SMPLRT_DIV, 0x00)
        self.__set_accel_fsr(ACCEL_FSR_8G)
        self.__set_gyro_fsr(GYRO_FSR_2000DPS)
        self.__set_accel_dlpf(ACCEL_DLPF_184)
        self.__set_gyro_dlpf(GYRO_DLPF_184)
        # self.__mpu_set_sample_rate(sample_rate)
        self.__init_magnetometer()

        time.sleep(0.1)

    def __reset_mpu(self):
        # write the reset bit
        self._bus.write_byte_data(self._addr, PWR_MGMT_1, H_RESET)

        # wait and try again
        time.sleep(0.1)
        self._bus.write_byte_data(self._addr, PWR_MGMT_1, H_RESET)
        time.sleep(0.1)

        return 0

    def __init_magnetometer(self):
        if self.__mpu_set_bypass(1) < 0:
            return -1

        # Power down magnetometer
        self._bus.write_byte_data(AK8963_ADDR, AK8963_CNTL, MAG_POWER_DN)
        time.sleep(0.01)

        #  Enter Fuse ROM access mode
        self._bus.write_byte_data(AK8963_ADDR, AK8963_CNTL, MAG_FUSE_ROM)
        time.sleep(0.01)

        raw = self._bus.read_i2c_block_data(AK8963_ADDR, AK8963_ASAX, 3)

        # Return sensitivity adjustment values
        self._mag_factory_adjust[0] = (raw[0] - 128) / 256.0 + 1.0
        self._mag_factory_adjust[1] = (raw[1] - 128) / 256.0 + 1.0
        self._mag_factory_adjust[2] = (raw[2] - 128) / 256.0 + 1.0

        # Power down magnetometer again
        self._bus.write_byte_data(AK8963_ADDR, AK8963_CNTL, MAG_POWER_DN)
        time.sleep(0.01)

        # Configure the magnetometer for 16 bit resolution
        # and continuous sampling mode 2 (100hz)
        c = MSCALE_16 | MAG_CONT_MES_2
        self._bus.write_byte_data(AK8963_ADDR, AK8963_CNTL, c)
        time.sleep(0.01)

        return 0

    def __mpu_set_bypass(self, bypass_on):
        tmp = 0
        if self._dmp_en > 0:
            tmp |= FIFO_EN_BIT
        if bypass_on == 0:
            tmp |= I2C_MST_EN
        self._bus.write_byte_data(self._addr, USER_CTRL, tmp)

        time.sleep(0.03)

        # INT_PIN_CFG settings
        tmp = LATCH_INT_EN | INT_ANYRD_CLEAR | ACTL_ACTIVE_LOW
        if bypass_on > 0:
            tmp |= BYPASS_EN
        self._bus.write_byte_data(self._addr, INT_PIN_CFG, tmp)
        if bypass_on > 0:
            self._bypass_en = 1
        else:
            self._bypass_en = 0

        return 0

    def __check_who_am_i(self):
        c = self._bus.read_byte_data(self._addr, 0)
        if c != 0x71:
            return -1

        return 0

    def __mpu_set_sample_rate(self, rate):
        if rate > 1000 or rate < 4:
            raise ValueError('sample rate must be between 4 & 1000')
        div = int(np.uint8(1000 / rate - 1))
        self._bus.write_byte_data(self._addr, SMPLRT_DIV, div)

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

    def mpu_read_mag(self):
        raw = self._bus.read_i2c_block_data(AK8963_ADDR, AK8963_XOUT_L, 6)

        # Turn the MSB and LSB into a signed 16-bit value
        # Data stored as little Endian
        mx_raw = np.int16(np.int16(raw[0] << 8) | raw[1])
        my_raw = np.int16(np.int16(raw[2] << 8) | raw[3])
        mz_raw = np.int16(np.int16(raw[4] << 8) | raw[5])

        # multiply by the sensitivity adjustment and convert to units of uT micro
        # Teslas. Also correct the coordinate system as someone in InvenSense
        # thought it would be bright idea to have the magnetometer coordinate
        # system aligned differently than the accelerometer and gyro.... -__-
        mx = my_raw * self._mag_factory_adjust[1] * MAG_RAW_TO_uT
        my = mx_raw * self._mag_factory_adjust[0] * MAG_RAW_TO_uT
        mz = -mz_raw * self._mag_factory_adjust[2] * MAG_RAW_TO_uT

        return {'mx': mx, 'my': my, 'mz': mz}
