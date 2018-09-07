import beagle as bg
import sys
import time
import csv


def main():
    mpu = bg.MPU()

    try:
        with open('output.csv', 'w') as f:
            writer = csv.writer(f, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            writer.writerow(['imu_ax', 'imu_ay', 'imu_az', 'imu_gx', 'imu_gy', 'imu_gz'])
            while True:
                try:
                    accel = mpu.mpu_read_accel()
                    gyro = mpu.mpu_read_gyro()
                    row = [accel['ax'], accel['ay'], accel['az']]
                    row += [gyro['gx'], gyro['gy'], gyro['gz']]
                    writer.writerow(row)

                except KeyboardInterrupt:
                    f.close()
                    break

    except IOError:
        print('Building CSV failed.')

    return 0


if __name__ == '__main__':
    main()
