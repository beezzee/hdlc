import serial
import imp




hdlc_escape = 0x7d
hdlc_flag = 0x7e
hdlc_address = 0xff
hdlc_control = 0x03
cmd_timeout = 3
status_ok = 0
status_invalid_response = 1

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

    print ("<- " + format_frame(command) + "(" + format_frame(hdlc_command) + ")")

    port.write(bytearray(hdlc_command))


def read_frame(port,timeout=None):
    port.timeout = timeout
    data = port.read(512)
    l = list(data)
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
        

def open_port(port="/dev/ttyACM1"):
    return serial.Serial(port,baudrate=115200,timeout=3)

def read_port(port,timeout=None):
    port.timeout=timeout
    data = port.read()
    print (data)

def exchange(port,data,address=hdlc_address,control=hdlc_control,timeout=None):
    transmit_payload(port,data,address,control)
    raw = read_frame(port,timeout)

    

    frame = hdlc_parse_frame(raw)
    print ("->" + format_frame(frame)[0:-2] + "(" + format_frame(raw) + ")")
    return frame

def check_response(data):
    if len(data) < 2:
        return status_invalid_response

    return data[1]

def start_timeout(port,time=0,timeout=None):
    request = [time & 0xFF, time >> 8]
    response = exchange(port,request,cmd_timeout,0,timeout)
    status = check_response(response)
    if status_ok != status:
        print("Setting timeout failed with error status {0}".format(status))
        return False

    return True


