import beagle as bg
import argparse
import sys
import time
import csv
import socket
import pickle

def main():

    parser = argparse.ArgumentParser(description='Collect data from BeagleBone Blue')
    parser.add_argument('-6', help='6 Axis', action='store_true')
    parser.add_argument('-10', help='10 Axis', action='store_true')

    args = vars(parser.parse_args())
    axis_10 = args['10']

    # socket init
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(('192.168.1.6', 8001))
    print('conntect sucessful\n')

    mpu = bg.MPU()

    try:
        ts = time.time()
        st = bg.get_datetime(ts).strftime('%Y-%m-%d_%H%M%S')
        start_st = st
        print('[%s] Start reading sensor data...' % st)

        with open(st + '.csv', 'w') as f:
            writer = csv.writer(f)
            if axis_10:
                writer.writerow(['timestamp', 'temp',
                                 'imu_ax', 'imu_ay', 'imu_az',
                                 'imu_gx', 'imu_gy', 'imu_gz',
                                 'imu_mx', 'imu_my', 'imu_mz'])
            else:
                writer.writerow(['timestamp',
                                 'imu_ax', 'imu_ay', 'imu_az',
                                 'imu_gx', 'imu_gy', 'imu_gz'])
            data_rows = 0
            while True:
                try:
                    ts = time.time()
                    accel = mpu.mpu_read_accel()
                    gyro = mpu.mpu_read_gyro()

                    row = [ts]
                    if axis_10:
                        temp = mpu.mpu_read_temp()
                        row += [temp]

                    row += [accel['ax'], accel['ay'], accel['az']]
                    row += [gyro['gx'], gyro['gy'], gyro['gz']]

                    if axis_10:
                        mag = mpu.mpu_read_mag()
                        row += [mag['mx'], mag['my'], mag['mz']]

                    # sys.stdout.write('\r')
                    # sys.stdout.write(str(row))
                    # sys.stdout.flush()

                    writer.writerow(row)
                    # socket test
                    send_data = pickle.dumps(row)
                    sock.sendall(send_data)
                    count = "time: " + str(data_rows)
                    #st=count+ str(row[0])
                    #byt=st.encode()
                    #sock.send(byt)

                    print(count)
                    data_rows += 1
                    if data_rows % 10000 == 0:
                        st = bg.get_datetime(ts).strftime('%Y-%m-%d_%H%M%S')
                        print('[%s] %6d rows have been collected.' % (st, data_rows))

                except KeyboardInterrupt:
                    f.close()
                    sock.send("end".encode())
                    ts = time.time()
                    st = bg.get_datetime(ts).strftime('%Y-%m-%d_%H%M%S')
                    print('[%s] \"%s\" was saved.' % (st, start_st + '.csv'))

                    break
    except IOError:
        print('Building CSV failed.')

    return 0


if __name__ == '__main__':
    main()
