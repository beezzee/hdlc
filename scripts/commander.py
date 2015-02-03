import serial
import imp




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

    print ("<- " + formated_frame)

    port.write(command)


def read_frame(port,timeout=None):
    port.timeout = timeout
    data = port.read(512)

    print ("->" + format_frame(data))
    return data

def hdlc_parse_frame(frame):
    #assume that frame starts and ends with flag
    data = bytearray()
    i = 0

    #forward to start sequence
    while i<len(frame) and frame[i] !=  hdlc_flag:
        i+=1

    #remove start sequence
    i+=1

    #transform escaped bytes
    while i<len(frame) and frame[i] !=  hdlc_flag:
        if frame[i] == hdlc_escape and i+1 < len(frame):
            data.append(frame[i+1] ^ 0x20)
            i+=1
        else: 
            data.append(frame[i])
            
        i+=1

    return data
        

def open_port():
    return serial.Serial(port="/dev/ttyACM1",baudrate=115200,timeout=3)

def read_port(port,timeout=None):
    port.timeout=timeout
    data = port.read()
    print (data)

def exchange(port,data,address=hdlc_address,control=hdlc_control,timeout=None):
    transmit_payload(port,data,address,control)

    frame = hdlc_parse_frame(read_frame(port,timeout))




