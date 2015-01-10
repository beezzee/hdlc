import serial





hdlc_escape = 0x7d
hdlc_flag = 0x7e
hdlc_address = 0xff
hdlc_control = 0x03


def transmit_payload(data): 
    
    command = bytearray([hdlc_flag,hdlc_address,hdlc_control]) + data + bytearray([0x00,0x00,hdlc_flag])

    print "transmit "
    formated_frame = ""
    for i in command:
        formated_frame += "{0:02x} ".format(i)

    print formated_frame

    commandPort.write(command)

def open_port():
    commandPort = serial.Serial(port="/dev/ttyACM1",baudrate=115200)
