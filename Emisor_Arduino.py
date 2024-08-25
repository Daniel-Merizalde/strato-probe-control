import RPi.GPIO as GPIO  # import gpio
import time      #import time library
import spidev
from lib_nrf24 import NRF24   #import NRF24 library

GPIO.setmode(GPIO.BCM)# set the gpio mode
#    CRC_16 = 2
  # set the pipe address. this address shoeld be entered on the receiver alo
#def begin (self, 24)
#self.spidev.open
#self.spidev.max_hz = 4000000

pipes = [[0xE8, 0xE8, 0xF0, 0xF0, 0xE1], [0xF0, 0xF0, 0xF0, 0xF0, 0xE1]]
radio = NRF24(GPIO, spidev.SpiDev())   # use the gpio pins
radio.begin(0, 25)   # start the radio and set the ce,csn pin ce= GPIO08, csn= GPIO25
radio.setPayloadSize(32)  #set the payload size as 32 bytes
radio.setChannel(0x76) # set the channel as 76 hex
radio.setDataRate(NRF24.BR_1MBPS)    # set radio data rate
radio.setPALevel(NRF24.PA_MIN)  # set PA level

radio.setAutoAck(True)
radio.enableDynamicPayloads()
radio.enableAckPayload()
radio.setDataRate(NRF24.BR_2MBPS)
radio.setCRCLength(NRF24.CRC_8)
radio.setRetries(5, 15)

radio.openWritingPipe(pipes[0])
radio.openReadingPipe(1, pipes[1])     # open the defined pipe for writing
radio.printDetails()

message_length = 32
start = 2

message = "1"  # the message to be sent
missing_chars_number = message_length - len(message)
message = list(message + '-' * missing_chars_number)

radio.write(message)  # just write the message to radio

get_z_message = "z"  # the message to be sent
missing_chars_number = message_length - len(get_z_message)
get_z_message = list(get_z_message + '-' * missing_chars_number)

radio.write(get_z_message)
radio.stopListening()


while True:
    ultrasound_value = 5000
    time.sleep(0.5)
    ultrasound_string = str(ultrasound_value)
    missing_chars_number = message_length - len(ultrasound_string)
    ultrasound_radio = list(ultrasound_string + '-' * missing_chars_number)
    radio.write(ultrasound_radio)
    print(ultrasound_radio)
    
#     while not radio.available(0):
#         time.sleep(1/100)
#         print('not available')
#         if time.time() - start > 2:
#             print("Timed out.")  # print errror message if radio disconnected or not functioning anymore
#             break

    radio.stopListening()     # close radio
#     time.sleep(0.5)  # give delay of 3 seconds