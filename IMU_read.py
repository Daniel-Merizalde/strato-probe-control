#!/usr/bin/python
import smbus
import math
from math import atan, sqrt, tan, acos, cos
import time
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from scipy.signal import lfilter

# Register
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c


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


def get_y_rotation(x, y, z):
    radians = math.atan2(x, dist(y, z))
    return -math.degrees(radians)


def get_x_rotation(x, y, z):
    radians = math.atan2(y, dist(x, z))
    return math.degrees(radians)


def get_y_rad(x, y, z):
    radians = math.atan2(x, dist(y, z))
    return radians


def get_x_rad(x, y, z):
    radians = math.atan2(y, dist(x, z))
    return radians


def get_z_rotation(x, y, z):
    radians = math.atan2(z, dist(x, y))
    return math.degrees(radians)


def get_z_rad(x, y, z):
    radians = math.atan2(z, dist(x, y))
    return radians


def get_rotation_matrix(x, y, z):
    return Rotation.from_euler('xyz', [x, y, z], degrees=True).as_matrix().transpose()


def get_inertial_acceleration():
    accelerometer_x_scaled = read_word_2c(ax_channel) / 16384.0
    accelerometer_y_scaled = read_word_2c(ay_channel) / 16384.0
    accelerometer_z_scaled = read_word_2c(az_channel) / 16384.0

    a_vector = np.array([[accelerometer_x_scaled], [accelerometer_y_scaled], [accelerometer_z_scaled]]) * g

    x = get_x_rotation(accelerometer_x_scaled, accelerometer_y_scaled, accelerometer_z_scaled)
    y = get_y_rotation(accelerometer_x_scaled, accelerometer_y_scaled, accelerometer_z_scaled)
    z = get_z_rotation(accelerometer_x_scaled, accelerometer_y_scaled, accelerometer_z_scaled)
    rotation_matrix = get_rotation_matrix(x, y, z)
    return np.matmul(rotation_matrix, a_vector) - g_vector


def average():
    a = []
    measures = 1000
    for i in range(measures):
        a.append(get_inertial_acceleration())

    a = np.array(a)
    return a.mean(axis=0).reshape(3)


def angle_average():
    yaws = []
    rolls = []
    pitches = []
    measures = 1000
    average_roll = get_x_rotation(accelerometer_x_scaled, accelerometer_y_scaled, accelerometer_z_scaled)
    average_pitch = get_y_rotation(accelerometer_x_scaled, accelerometer_y_scaled, accelerometer_z_scaled)
    average_yaw = get_z_rotation(accelerometer_x_scaled, accelerometer_y_scaled, accelerometer_z_scaled)

    for i in range(measures):
        yaws.append(average_yaw)
        rolls.append(average_roll)
        pitches.append(average_pitch)

    av_yaw = np.array(yaws)
    av_roll = np.array(rolls)
    av_pitch = np.array(pitches)

    return av_roll.mean(), av_pitch.mean(), av_yaw.mean()


# def integrate(x, t):
#     return np.trapz(x, t, axis=0)


bus = smbus.SMBus(1)  # bus = smbus.SMBus(0) fuer Revision 1
address = 0x68  # via i2cdetect

# Aktivieren, um das Modul ansprechen zu koennen
bus.write_byte_data(address, power_mgmt_1, 0)

gx_channel = 0x43
gy_channel = 0x45
gz_channel = 0x47
ax_channel = 0x3b
ay_channel = 0x3d
az_channel = 0x3f

g = 9.8
g_vector = np.array([[0], [0], [g]])

gyroscope_x_out = read_word_2c(gx_channel)
gyroscope_y_out = read_word_2c(gy_channel)
gyroscope_z_out = read_word_2c(gz_channel)

gyroscope_x_scaled = (gyroscope_x_out / 131)
gyroscope_y_scaled = (gyroscope_y_out / 131)
gyroscope_z_scaled = (gyroscope_z_out / 131)

accelerometer_x_out = read_word_2c(ax_channel)
accelerometer_y_out = read_word_2c(ay_channel)
accelerometer_z_out = read_word_2c(az_channel)

accelerometer_x_scaled = accelerometer_x_out / 16384.0
accelerometer_y_scaled = accelerometer_y_out / 16384.0
accelerometer_z_scaled = accelerometer_z_out / 16384.0

average_acceleration = average()

absolute_pos = np.array([0., 0., 0.])

angles = angle_average()
average_roll = angles[0]
average_pitch = angles[1]
average_yaw = angles[2]

phi_list = []
filtered_phi_list = []
counter_list = []
rounded_list = []
counter = 0
heights_list = []
sin_phi_list = []

n = 35
b = [1.0 / n] * n
a = 1

# for i in range(2000):
#     accelerometer_x_out = read_word_2c(ax_channel)
#     accelerometer_y_out = read_word_2c(ay_channel)
#     accelerometer_z_out = read_word_2c(az_channel)
#
#     accelerometer_x_scaled = accelerometer_x_out / 16384.0
#     accelerometer_y_scaled = accelerometer_y_out / 16384.0
#     accelerometer_z_scaled = accelerometer_z_out / 16384.0
#
#     gyroscope_x_out = read_word_2c(gx_channel)
#     gyroscope_y_out = read_word_2c(gy_channel)
#     gyroscope_z_out = read_word_2c(gz_channel)
#
#     yaw = get_z_rad(accelerometer_x_scaled, accelerometer_y_scaled, accelerometer_z_scaled)
#     roll = get_x_rad(accelerometer_x_scaled, accelerometer_y_scaled, accelerometer_z_scaled)
#     pitch = get_y_rad(accelerometer_x_scaled, accelerometer_y_scaled, accelerometer_z_scaled)
#
#     phi = acos((cos(roll) * cos(pitch)))
#     phi_degrees = math.degrees(phi)
#     sin_phi = math.sin(phi_degrees)
#     rounded_phi = round(phi_degrees, 3)
#     counter += 1
#     height = 50
#
#     phi_list.append(phi)
#     counter_list.append(counter)
#     rounded_list.append(rounded_phi)
#     heights_list.append(height)
#     sin_phi_list.append(sin_phi)
#     # filtered_phi_list.append(phi_filtered)
#     print(counter, phi_degrees)
#
# phi_filtered = lfilter(b, a, phi_list)
# phi_plot = plt.plot(counter_list, phi_list)
# filtered_plot = plt.plot(counter_list, phi_filtered)
# # total_height = heights_list + phi_filtered
# # rounded_plot = plt.plot(counter_list, rounded_list)
# plt.show()


while True:
    roll = get_x_rotation(accelerometer_x_scaled, accelerometer_y_scaled, accelerometer_z_scaled)
    pitch = get_y_rotation(accelerometer_x_scaled, accelerometer_y_scaled, accelerometer_z_scaled)
    yaw = get_z_rotation(accelerometer_x_scaled, accelerometer_y_scaled, accelerometer_z_scaled)

    # yaw = get_z_rad(accelerometer_x_scaled, accelerometer_y_scaled, accelerometer_z_scaled)
    # roll = get_x_rad(accelerometer_x_scaled, accelerometer_y_scaled, accelerometer_z_scaled)
    # pitch = get_y_rad(accelerometer_x_scaled, accelerometer_y_scaled, accelerometer_z_scaled)

    # print("  Roll:   ", roll, end='')
    # print("  Pitch:   ", pitch, end='')
    # print("  Yaw:   ", yaw)
    time.sleep(1)

    accelerometer_x_out = read_word_2c(ax_channel)
    accelerometer_y_out = read_word_2c(ay_channel)
    accelerometer_z_out = read_word_2c(az_channel)

    accelerometer_x_scaled = accelerometer_x_out / 16384.0
    accelerometer_y_scaled = accelerometer_y_out / 16384.0
    accelerometer_z_scaled = accelerometer_z_out / 16384.0

    gyroscope_x_out = read_word_2c(gx_channel)
    gyroscope_y_out = read_word_2c(gy_channel)
    gyroscope_z_out = read_word_2c(gz_channel)

    gyroscope_x_scaled = (gyroscope_x_out / 131)
    gyroscope_y_scaled = (gyroscope_y_out / 131)
    gyroscope_z_scaled = (gyroscope_z_out / 131)

    tilt_angle = atan(sqrt((tan(roll) * tan(roll)) + (tan(pitch) * tan(pitch))))
    tilt_degrees = math.degrees(tilt_angle)

    real_roll = roll - average_roll
    real_pitch = pitch - average_pitch
    real_yaw = yaw - average_yaw

    print(gyroscope_z_scaled)

    # print('Roll:', real_roll, 'Pitch', real_pitch, 'Yaw:', real_yaw)
