import beagle as bg
import sys
import time
import csv
from datetime import datetime


def main():
    mpu = bg.MPU()

    ts = time.time()
    st = datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H%M%S')

    try:
        with open(st + '.csv', 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'imu_ax', 'imu_ay', 'imu_az', 'imu_gx', 'imu_gy', 'imu_gz'])
            while True:
                try:
                    ts = time.time()
                    accel = mpu.mpu_read_accel()
                    gyro = mpu.mpu_read_gyro()
                    row = [ts]
                    row += [accel['ax'], accel['ay'], accel['az']]
                    row += [gyro['gx'], gyro['gy'], gyro['gz']]
                    writer.writerow(row)

                    time.sleep(0.01)
                except KeyboardInterrupt:
                    f.close()
                    break

    except IOError:
        print('Building CSV failed.')

    return 0


if __name__ == '__main__':
    main()
