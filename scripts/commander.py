import serial





hdlc_escape = 0x7d
hdlc_flag = 0x7e
hdlc_address = 0xff
hdlc_control = 0x03


def transmit_payload(port,data): 
    
    command = bytearray([hdlc_flag,hdlc_address,hdlc_control]) + data + bytearray([0x00,0x00,hdlc_flag])

    formated_frame = ""
    for i in command:
        formated_frame += "{0:02x} ".format(i)

    print "transmit: " + formated_frame

    port.write(command)

def open_port():
    return serial.Serial(port="/dev/ttyACM1",baudrate=115200,timeout=3)

def read_port(port,timeout=None):
    port.timeout=timeout
    data = port.read()
    print data

def exchange(port,data,timeout=None):
    transmit_payload(port,data)
    port.timeout = timeout
    lines = port.readlines()
    for line in lines:
        print line


