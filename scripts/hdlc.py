import serial
import logging
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


logger = logging.getLogger(__name__)


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
        return self.s

class HdlcFrameError(HdlcException):
    def __str__(self):
        return "HDLC frame error: " + self.s

class HdlcCrcError(HdlcException):
    def __str__(self):
        return "HDLC CRC error: " + self.s

class HdlcStatusError(HdlcException):
    def __str__(self):
        return "HDLC error, slave returned status " + self.s


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

    #remove start sequence
        while i<len(frame) and frame[i] ==  hdlc_flag:
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
            raise HdlcFrameError("frame shorter than 4 bytes")

    #dummy CRC check
        if data[-2] != 0 or data[-1] != 0:
            raise HdlcCrcError(str(data[-2:-1]))

        if status_ok != data[1]:
            raise HdlcStatusError(str(data[1]))

        return cls(data[2:-2],data[0],data[1])



def transmit_payload(port,data,address=hdlc_address,control=hdlc_control):

    command = HdlcFrame(data,address,control)

    logger.debug("<- " + str(command) + "( " + str(command.encode()) + " )")

    port.write(bytearray(command.encode()))


def read_frame(port,timeout=0):
    port.timeout = 1

    data = []

    timer=0
    while True:
        #read until next EOF flag, maximum of 512 bytes, or timeout

        #read one byte at a time to immediately abort at EOF
        chunk = port.read(1)
        timer+=1

        #no timeout:
        if len(chunk) > 0:
            #restart timeout
            timer=0
            data += list(chunk)

        if timer>=timeout:
            logger.debug("Timeout while waiting for data.")
            break

        if len(data) > 1 and data[-1] == hdlc_flag:
            logger.debug("Interpret HDLC boundary flag as EOF")
            break

        if len(data)>=256:
            logger.debug("Reached maximum frame size")
            break

    logger.debug("-> " + str(data))
    return data



def open_port(port="/dev/ttyACM1",baudrate=115200):
    return serial.Serial(port,baudrate,timeout=3)

def read_port(port,timeout=None):
    port.timeout=timeout
    data = port.read()
    logger.debug(data)

def exchange(port,data,address=hdlc_address,control=hdlc_control,timeout=None,retry=0):

    trial=0
    while True:
        transmit_payload(port,data,address,control)
        data = read_frame(port,timeout)

        logger.debug("Try to decode frame")
        
        try:
            frame = HdlcFrame.decode(data)
        except HdlcException as e:
            logger.info("Trial " + str(trial) + ": invalid response")
            trial+=1
            if trial == retry:
                logger.warn("give up data exchange")
                raise e
        else:
            logger.debug(str(frame) + " (" + str(data) + ")")
            return frame



def echo(port,payload,timeout=None,trials=10):
    response = exchange(port,payload,cmd_echo,0,timeout,trials)
    return response




