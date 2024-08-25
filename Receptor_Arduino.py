import RPi.GPIO as GPIO
from lib_nrf24 import NRF24
import time
import spidev

GPIO.setmode(GPIO.BCM)

pipes = [[0xE8, 0xE8, 0xF0, 0xF0, 0xE1], [0xF0, 0xF0, 0xF0, 0xF0, 0xE1]]

radio = NRF24(GPIO, spidev.SpiDev())
radio.begin(0, 25)

radio.setPayloadSize(32)
radio.setChannel(0x76)
radio.setDataRate(NRF24.BR_1MBPS)
radio.setPALevel(NRF24.PA_MIN)

radio.setAutoAck(True)
radio.enableDynamicPayloads()
radio.enableAckPayload()

radio.openWritingPipe(pipes[0])
radio.openReadingPipe(1, pipes[1])
radio.printDetails()

message_length = 32
message = "1"  # the message to be sent
missing_chars_number = message_length - len(message)
message = list(message + '-' * missing_chars_number)
print(message)

radio.write(message)  # just write the message to radio

get_z_message = "z"  # the message to be sent
missing_chars_number = message_length - len(get_z_message)
get_z_message = list(get_z_message + '-' * missing_chars_number)

radio.startListening()

while 1:

    while not radio.available(0):
        time.sleep(1 / 100)
        print("no radio availiable")
    receivedMessage = []
    radio.read(receivedMessage, radio.getDynamicPayloadSize())
    print("Received: {}".format(receivedMessage))

    print("Translating the receivedMessage into unicode characters")
    string = ""
    for n in receivedMessage:
                # Decode into standard unicode set
        if 32 <= n <= 126:
            string += chr(n)
    print("Out received message decodes to: {}".format(string))

    # temperature = float.fromhex(''.join(format(x, '02x') for x in recievedMessage))
    # print(temperature)

    # try:
    #     value = (float(dec_message))
    #     print(value)
    # except Exception as e:
    #     print(str(e))
