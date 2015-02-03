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

def hdlc_encode_frame(data):
    frame = [hdlc_flag]
    for d in data:
        if hdlc_escape == d or hdlc_flag == d:
            frame.append(hdlc_escape)
            frame.append(d ^ 0x20)
        else:
            frame.append(d)

    #append dummy CRC and EOF marker
    frame += [0x00,0x00,hdlc_flag]
    return frame

def transmit_payload(port,data,address=hdlc_address,control=hdlc_control): 
    
    command = [address,control] + data 
    
    hdlc_command = hdlc_encode_frame(command)

    formated_frame = format_frame(command)

    print ("<- " + formated_frame)

    port.write(bytearray(hdlc_command))


def read_frame(port,timeout=None):
    port.timeout = timeout
    data = port.read(512)
    l = list(data)
    print ("->" + format_frame(l))
    return l

def hdlc_parse_frame(frame):
    #assume that frame starts and ends with flag
    data = list()
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
    print("Payload: " + frame[0:-2])
    return frame





