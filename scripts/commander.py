import serial
import imp




hdlc_escape = 0x7d
hdlc_flag = 0x7e
hdlc_address = 0xff
hdlc_control = 0x03
cmd_calibrate = 2
cmd_timeout = 3
status_ok = 0
status_invalid_response = 1
status_crc_at_master = 1
status_crc_at_slave = 1<<4

def from_little_endian(l):
    r = 0
    for d in l:
        assert int(d) < 256
        r = r << 8 
        r = r + int(d)

    return d

class status:
    currentTime
    timeout
    currentTemperature
    targetTemperature
    position

    def __init__(self,d):
        assert len(d) > 2*5 

        self.timeout = from_little_endian(d[0:1])
        self.currentTime = from_little_endian(d[2:3])
        self.targetTemperature = from_little_endian(d[4:5])
        self.currentTemperature = from_little_endian(d[6:7])
        self.position = from_little_endian(d[8:9])
        
    def __str__(self):
        s = ""
        s += "Timeout: " + self.timeout + " s\n"
        if self.currentTime == 0:
            s += "Timer: stopped\n"
        else:
            s += "Timer: running, " + self.currentTime + " s from timeout\n"

        s += "Temperature: " self.currentTemperature/1000 + "." (self.currentTemperature % 1000)/100 + " K (" + (self.currentTemperature-271150)/1000 + "." ((self.currentTemperature-271150) % 1000)/100 + "C )\n"

        s += "Position: " + self.position + " mm"
        
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
    assert len(frame) >= 4 , "Expect at least 4 bytes"
    print ("->" + format_frame(frame)[0:-2] + "(" + format_frame(raw) + ")")
    return frame

def check_response(data):
    if len(data) < 4:
        print("Error: at least 4 bytes expected")
        return status_invalid_response

    #dummy CRC check
    if data[-2] != 0 or data[-1] != 0:
        print("CRC error")
        return status_crc_at_master

    if status_ok != data[1]:
        print("Setting timeout failed with error status {0}".format(status))
        return data[1]

    print("Response ok")
    return status_ok

def start_timeout(port,time=0,timeout=None):
    print("Set timeout to " + time)
    request = [time & 0xFF, time >> 8]
    response = exchange(port,request,cmd_timeout,0,timeout)
    return check_response(response)

def calibrate(port,temperature,timeout=None):
    request = [temperature & 0xFF, temperatue >> 8]
    response = exchange(port,request,cmd_calibrate,0,timeout)
    return check_response(response)

def payload(frame):
    #cutt off address, status, and crc 
    return frame[2:-2]

def get_status(port,timeout=None):
    response = exchange(port,[],cmd_status,0,timeout)
    if status_ok == check_response(response):
        return status(payload(response))
