import serial
#import imp




hdlc_escape = 0x7d
hdlc_flag = 0x7e
hdlc_address = 0xff
hdlc_control = 0x03
cmd_echo = 0


status_ok = 0
status_invalid_response = 1
status_crc_at_master = 1
status_crc_at_slave = 1<<4

def from_little_endian(l):
    r = 0
    for d in reversed(l):
        assert int(d) < 256
        r = r << 8
        r = r + int(d)

    return d

class HdlcException(Exception):
    def __init__(self,s):
        self.s = s

    def __str__(self):
        return str(self.s)

class HdlcFrameError(HdlcException):
    pass

class HdlcCrcError(HdlcException):
    pass

class HdlcSlaveError(HdlcException):
    pass


class HdlcFrame(list):
    def __init__(self, payload=[],address=hdlc_address,control=hdlc_control):
        list.__init__(self,payload)
        self.address=address
        self.control=control

    def __str__(self):
        return self.format()

    def format(self,base=16):
        formated_frame = ""
        formated_frame += "addr:{0:02x} ".format(self.address)
        formated_frame += "ctrl:{0:02x} ".format(self.control)
        formated_frame += "data:"
        for i in self:
            formated_frame += "{0:02x} ".format(i)


        return formated_frame

    def encode(self):
        frame = [hdlc_flag,self.address,self.control]
        for d in self:
            if hdlc_escape == d or hdlc_flag == d:
                frame.append(hdlc_escape)
                frame.append(d ^ 0x20)
            else:
                frame.append(d)

    #append dummy CRC and EOF marker
        frame += [0x00,0x00,hdlc_flag]
        return frame

    @classmethod
    def decode(cls,frame):
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

        if len(data) < 4:
            raise HdlcFrameError(len(data))

    #dummy CRC check
        if data[-2] != 0 or data[-1] != 0:
            raise HdlcCrcError(0)

        if status_ok != data[1]:
            raise HdlcSlaveError(data[1])

        return cls(data[2:-2],data[0],data[1])



def transmit_payload(port,data,address=hdlc_address,control=hdlc_control):

    command = HdlcFrame(data,address,control)

    print ("<- " + str(command) + "( " + str(command.encode()) + " )")

    port.write(bytearray(command.encode()))


def read_frame(port,timeout=None):
    port.timeout = timeout
    data = port.read(512)
    l = list(data)
    return l



def open_port(port="/dev/ttyACM1",baudrate=115200):
    return serial.Serial(port,baudrate,timeout=3)

def read_port(port,timeout=None):
    port.timeout=timeout
    data = port.read()
    print (data)

def exchange(port,data,address=hdlc_address,control=hdlc_control,timeout=None,retry=0):

    trial=0
    while True:

        transmit_payload(port,data,address,control)
        raw = read_frame(port,timeout)
        try:
            frame = HdlcFrame.decode(raw)
        except HdlcException as e:
            trial+=1
            print ("exch " + str(trial) + " -> " + str(raw) + " invalid")
            if trial == retry:
                print("give up")
                raise e
        else:
            print ("-> ( " + str(raw) + " ) " + str(frame))
            return frame



def echo(port,payload,timeout=None,trials=10):
    response = exchange(port,payload,cmd_echo,0,timeout,trials)
    return response




