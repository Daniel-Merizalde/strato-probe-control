import time
from multiprocessing import Process, Value, Manager
import adafruit_rplidar
from adafruit_rplidar import RPLidar
import math
from math import floor, cos, sin, radians, sqrt, atan, tan
import numpy as np
import RPi.GPIO as GPIO
from lib_nrf24 import NRF24
import spidev
import serial
import smbus

# region Settings
filename = 'data/lidar_Laser_IMU.txt'

# Setup the RPLidar
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME)

# used to scale data to fit on the screen
max_distance = 0

GPIO.setmode(GPIO.BCM)

# initialize B87A sensor
ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)

power_pin = 18

GPIO.setup(power_pin, GPIO.OUT)
GPIO.output(power_pin, 0)
time.sleep(1)
GPIO.output(power_pin, 1)
time.sleep(0.2)

readOut = 0

print("Starting up B87A laser...")
connected = False
commandToSend = bytearray()  # get the distance in mm
commandToSend.append(0x55)
ser.write(commandToSend)
address = b''

while address == b'':
    address = ser.read()
    print(address)

time.sleep(0.5)

turn_laser_on = bytearray([0xaa, 0x00, 0x01, 0xbe, 0x00, 0x01, 0x00, 0x01, 0xc1])
turn_laser_off = bytearray([0xaa, 0x00, 0x01, 0xbe, 0x00, 0x01, 0x00, 0x00, 0xc0])
continuous_auto = bytearray([0xaa, 0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x04, 0x25])
continuous_slow = bytearray([0xaa, 0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x05, 0x26])
continuous_fast = bytearray([0xaa, 0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x06, 0x27])
one_shot_auto = bytearray([0xaa, 0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x00, 0x21])
one_shot_fast = bytearray([0xaa, 0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x02, 0x23])
continuous_exit = bytearray([0x58])

# Setup IMU GY-91

power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

bus = smbus.SMBus(1)  # bus = smbus.SMBus(0) for Revision 1
address = 0x68  # i2c detect

bus.write_byte_data(address, power_mgmt_1, 0)

gx_channel = 0x43
gy_channel = 0x45
gz_channel = 0x47
ax_channel = 0x3b
ay_channel = 0x3d
az_channel = 0x3f

# Setup NRF modules

pipes = [[0xE8, 0xE8, 0xF0, 0xF0, 0xE1], [0xF0, 0xF0, 0xF0, 0xF0, 0xE1]]

radio = NRF24(GPIO, spidev.SpiDev())
radio.begin(0, 25)

radio.setPayloadSize(32)
radio.setChannel(0x76)
radio.setDataRate(NRF24.BR_1MBPS)
radio.setPALevel(NRF24.PA_MAX)

radio.setAutoAck(True)
radio.enableDynamicPayloads()
radio.enableAckPayload()
radio.setDataRate(NRF24.BR_2MBPS)
radio.setCRCLength(NRF24.CRC_8)
radio.setRetries(5, 15)

radio.openWritingPipe(pipes[0])
radio.openReadingPipe(1, pipes[1])
radio.printDetails()
# endregion

message_length = 32
message = "1"  # the message to be sent
missing_chars_number = message_length - len(message)
message = list(message + '-' * missing_chars_number)

radio.write(message)  # just write the message to radio

# get_z_message = "z"  # the message to be sent
# missing_chars_number = message_length - len(get_z_message)
# get_z_message = list(get_z_message + '-' * missing_chars_number)
# 
# radio.write(get_z_message)

radio.startListening()


def process_data(data, data_array, z, phi):
    for angle in range(360):
        distance = data[angle]
        cal_dist = ((0.9888 * distance) + 18.018)
        if distance > 0:  # ignore initial data points
            current_phi = phi.value
            x = (cal_dist * cos(radians(angle)))
            y = (cal_dist * sin(radians(angle)))
            current_z = z.value * cos(current_phi)
            data_array.append([x, y, current_z])


def get_lidar_data(data_array, z, phi):
    while True:
        try:
            print(lidar.info)
            break
        except adafruit_rplidar.RPLidarException:
            time.sleep(1 / 100)

    scan_data = [0] * 360
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance
        process_data(scan_data, data_array, z, phi)


def parse_measurement(response):
    return int.from_bytes(response[6:10], byteorder='big', signed=False)


def parse_quality(response):
    return int.from_bytes(response[10:12], byteorder='big', signed=False)


def descent_controller(z, phi):
    while True:
        ser.write(continuous_fast)
        for i in range(255):
            response = ser.read(13)
            if response != b'':
                laser_q = parse_quality(response)
                laser_value = parse_measurement(response)
                if 5 < laser_q <= 300:
                    accel_x_out = read_word_2c(ax_channel)
                    accel_y_out = read_word_2c(ay_channel)
                    accel_z_out = read_word_2c(az_channel)

                    accel_x_scaled = accel_x_out / 16384.0
                    accel_y_scaled = accel_y_out / 16384.0
                    accel_z_scaled = accel_z_out / 16384.0

                    yaw = get_z_radians(accel_x_scaled, accel_y_scaled, accel_z_scaled)
                    z.value = laser_value
                    phi.value = yaw - av_yaw


def stop_descent():
    message_length = 32
    message = "0"  # the message to be sent
    missing_chars_number = message_length - len(message)
    message = list(message + '-' * missing_chars_number)
    radio.write(message)


def send_command(command, response_length=9):
    response = b''
    while response == b'':
        ser.write(command)
        time.sleep(0.5)
        response = ser.read(response_length)
        print(response)

    return response


def read_byte(reg):
    return bus.read_byte_data(address, reg)


def read_word(reg):
    h = bus.read_byte_data(address, reg)
    l = bus.read_byte_data(address, reg + 1)
    value = (h << 8) + l
    return value


def read_word_2c(reg):
    val = read_word(reg)
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val


def dist(a, b):
    return math.sqrt((a * a) + (b * b))


def get_z_radians(x, y, z):
    rad = math.atan2(z, dist(x, y))
    return rad


def angle_average():
    angles = []
    measures = 1000
    average_yaw = get_z_radians(accelerometer_x_scaled, accelerometer_y_scaled, accelerometer_z_scaled)

    for i in range(measures):
        angles.append(average_yaw)

    b = np.array(angles)

    return b.mean()


accelerometer_x_out = read_word_2c(ax_channel)
accelerometer_y_out = read_word_2c(ay_channel)
accelerometer_z_out = read_word_2c(az_channel)

accelerometer_x_scaled = accelerometer_x_out / 16384.0
accelerometer_y_scaled = accelerometer_y_out / 16384.0
accelerometer_z_scaled = accelerometer_z_out / 16384.0

av_yaw = angle_average()
print('Average yaw =', av_yaw)


def main():
    manager = Manager()

    send_command(turn_laser_on)

    array = manager.list()
    z_laser = Value('i', 0)
    phi_angle = Value('d', 0)
    input_value = Value('i')
    laser_value = Value('i')
    lidar_process = Process(target=get_lidar_data, args=(array, z_laser, phi_angle))
    descent_process = Process(target=descent_controller, args=(z_laser, phi_angle))

    lidar_process.start()
    descent_process.start()


    input_value = 1
    time.sleep(0.5)

    input_value = int(input('Write 0 to stop: '))
    print("Saving dot cloud...")
    lidar_process.terminate()
    lidar_process.join()
    descent_process.terminate()
    descent_process.join()

    array = np.array(array)
    np.savetxt(filename, array)

    print('Turning off B87A...')
    ser.write(continuous_exit)
    time.sleep(2)
    send_command(turn_laser_off)
    print('B87A is off...')
    time.sleep(0.5)
    lidar.stop()
    lidar.disconnect()


if __name__ == '__main__':
    main()
