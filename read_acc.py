import beagle as bg
import sys
import time


if __name__ == '__main__':
    mpu = bg.MPU()
    print('x \t, y \t, z \t')

    while True:
        mpu.mpu_read_accel()
        accel = mpu.accel
        sys.stdout.write('\r')
        sys.stdout.write('%.1f \t, %.1f \t, %.1f' % (accel[0], accel[1], accel[2]))
        sys.stdout.flush()
        time.sleep(0.1)
