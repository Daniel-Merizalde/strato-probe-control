#!/usr/bin/env python3

import tkinter as tk
import tkinter.ttk as ttk
import tkinter.font
from gpiozero import LED
import RPi.GPIO as GPIO
import time
from serial import Serial
import numpy as np
from threading import Thread
from lib_nrf24 import NRF24
import spidev

serialDevice = "/dev/ttyAMA0"
maxwait = 3

GPIO.setmode(GPIO.BCM)

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

get_z_message = "z"  # the message to be sent
missing_chars_number = message_length - len(get_z_message)
get_z_message = list(get_z_message + '-' * missing_chars_number)

radio.write(get_z_message)
radio.stopListening()

GPIO.setmode(GPIO.BCM)
control_pin = [17, 18, 27, 22]

for pin in control_pin:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, 0)
    
seq = [[1, 0, 0, 0],
       [1, 1, 0, 0],
       [0, 1, 0, 0],
       [0, 1, 1, 0],
       [0, 0, 1, 0],
       [0, 0, 1, 1],
       [0, 0, 0, 1],
       [1, 0, 0, 1],]

def revolution():
    try:
        while True:
            if stop ==1:
                break
            for i in range(16):
                for half_step in range(8):
                    for pin in range(4):
                        GPIO.output(control_pin[pin], seq[half_step][pin])
                    time.sleep(0.005)
                
    except KeyboardInterrupt:
        GPIO.cleanup()
        
def ultrasound_measuring(portName):
    ser = Serial(portName, 9600, 8, 'N', 1, timeout=1)
    timeStart = time.time()
    valueCount = 0
    

    while time.time() < timeStart + maxwait:
        if ser.inWaiting():
            bytesToRead = ser.inWaiting()
            valueCount += 1
            if valueCount < 2: # 1st reading may be partial number; throw it out
                continue
            testData = ser.read(bytesToRead)
            if not testData.startswith(b'R'):
                # data received did not start with R
                continue
            try:
                sensorData = testData.decode('utf-8').lstrip('R')
            except UnicodeDecodeError:
                # data received could not be decoded properly
                continue
            try:
                mm = int(sensorData)
            except ValueError:
                # value is not a number
                continue
            ser.close()
            return(mm)

    ser.close()
    raise RuntimeError("Expected serial data not received")

def distance_average():
    measure_average = []
    measures = 8
    
    for i in range (measures):
            current_measurement = ultrasound_measuring(serialDevice)
            measure_average.append(current_measurement)
            time.sleep(0.03)
        
    average_distance = np.array(measure_average)
    
    distance.set(average_distance.mean())
    
def distance_update():
    measure_average = []
    measures = 8
    
    for i in range (measures):
            current_measurement = ultrasound_measuring(serialDevice)
            measure_average.append(current_measurement)
            time.sleep(0.03)
        
    average_distance = np.array(measure_average)
    dis_average = average_distance.mean()
    
    if 675 > dis_average > 475:
        stop_descent()
    
    distance_tag.config(text=f"Distancia al fondo: {dis_average}")
    app.after(500, distance_update)
    
def stop_descent():
    message_length = 32
    message = "00000000000000000000000000000000"  # the message to be sent
    missing_chars_number = message_length - len(message)
    message = list(message + '-' * missing_chars_number)
    radio.write(message)
    

def start_thread():
    global stop
    stop = 0
    
    t = Thread (target = revolution)
    t.start()
    
def stop():
    global stop
    stop = 1
    
win = tk.Tk()
distance = tk.IntVar()
win.title("Control sonda STRATO")
myFont=tkinter.font.Font(family = 'Helvetica', size = 16, weight = "bold")
myFont_large=tkinter.font.Font(family = 'Helvetica', size = 24, weight = "bold")
win.geometry("800x500+50+50")

app = tk.Frame(win)
app.grid()

start = tk.Button(app, text="ACTIVAR rotación", command=start_thread, bg="green", fg="white", height=6, width=24, font=myFont)
stop = tk.Button(app, text="Detener ROTACIÓN", command=stop, bg="orange", height=6, width=24, font=myFont)
exit_button = tk.Button(app, text="CERRAR", command=win.destroy, bg="red", height=2, width=12, font=myFont)
distance_tag = tk.Label(text = "Iniciando sensor...", font=myFont_large, bg="white")


distance_tag.place(x=350, y=50)
start.grid(row=2, column=1)
stop.grid(row=3, column=1)
exit_button.grid(row = 4, column = 1)

app.after(500, distance_update)

app.mainloop()
