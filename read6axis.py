import beagle as bg
import sys
import time


def main():
    mpu = bg.MPU()
    print('%6s, %6s, %6s, %6s, %6s, %6s' % ('ax', 'ay', 'az', 'gx', 'gy', 'gz'))

    try:
        while True:
            accel = mpu.mpu_read_accel()
            gyro = mpu.mpu_read_gyro()
            sys.stdout.write('\r')
            sys.stdout.write('%6.1f, %6.1f, %6.1f, ' % (accel['ax'], accel['ay'], accel['az']))
            sys.stdout.write('%6.1f, %6.1f, %6.1f, ' % (gyro['gx'], gyro['gy'], gyro['gz']))
            sys.stdout.flush()
            time.sleep(0.01)
    except KeyboardInterrupt:
        print('')

    return 0


if __name__ == '__main__':
    main()
