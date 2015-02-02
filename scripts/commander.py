import serial





hdlc_escape = 0x7d
hdlc_flag = 0x7e
hdlc_address = 0xff
hdlc_control = 0x03

def format_frame(frame):
    formated_frame = ""
    for i in frame:
        formated_frame += "{0:02x} ".format(i)
    return formated_frame

def transmit_payload(port,data,address=hdlc_address,control=hdlc_control): 
    
    command = bytearray([hdlc_flag,address,control]) + data + bytearray([0x00,0x00,hdlc_flag])
    
    formated_frame = format_frame(command)

    print ("transmit: " + formated_frame)

    port.write(command)


def read_frame(port,timeout=None):
    port.timeout = timeout
    data = bytearray()
    while True: 
        if len(port.read()) > 0:
            b = port.read()[0]
            print ("{0}".format(b))
            data.append(b)
            if  (b == hdlc_flag) and len(data) > 2: 
                break

    return data


def open_port():
    return serial.Serial(port="/dev/ttyACM1",baudrate=115200,timeout=3)

def read_port(port,timeout=None):
    port.timeout=timeout
    data = port.read()
    print (data)

def exchange(port,data,address=hdlc_address,control=hdlc_control,timeout=None):
    transmit_payload(port,data,address,control)

    frame = read_frame(port,timeout)

    print ("Received " + format_frame(frame))


