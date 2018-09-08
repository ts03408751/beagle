import beagle as bg
import sys
import time
import csv
from datetime import datetime


def main():
    mpu = bg.MPU()

    ts = time.time()
    st = bg.get_datetime(ts).strftime('%Y-%m-%d_%H%M%S')

    print(st)
    try:
        with open(st + '.csv', 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp',
                             'imu_ax', 'imu_ay', 'imu_az',
                             'imu_gx', 'imu_gy', 'imu_gz',
                             'imu_mx', 'imu_my', 'imu_mz'])
            while True:
                try:
                    ts = time.time()
                    accel = mpu.mpu_read_accel()
                    gyro = mpu.mpu_read_gyro()
                    mag = mpu.mpu_read_mag()
                    row = [ts]
                    row += [accel['ax'], accel['ay'], accel['az']]
                    row += [gyro['gx'], gyro['gy'], gyro['gz']]
                    row += [mag['mx'], mag['my'], mag['mz']]
                    sys.stdout.write('\r')
                    sys.stdout.write(str(row))
                    sys.stdout.flush()

                    writer.writerow(row)

                    time.sleep(0.005)

                except KeyboardInterrupt:
                    f.close()
                    break

    except IOError:
        print('Building CSV failed.')

    return 0


if __name__ == '__main__':
    main()
