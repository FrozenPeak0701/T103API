import T103LPDU
import serial, threading, time, datetime, struct


class T103ASDU:
    def __init__(self, port="COM1", baudrate=9600, bytesize=serial.EIGHTBITS, parity=serial.PARITY_EVEN,
                 stopbits=serial.STOPBITS_ONE, timeout=0.1):
        self.m_lpdu = T103LPDU.T103LPDU(port=port, baudrate=baudrate, bytesize=bytesize, parity=parity,
                                        stopbits=stopbits,
                                        timeout=timeout)

    def __del__(self):
        del self.m_lpdu

    def requireResponse(self, response, FCB):
        if response:
            return FCB, 1, 3
        else:
            return 0, 0, 4

    def sendASDU6(self, FCB: int, ADDR: int, ASDUADDR: int, response=True):  # Time synchronization
        databuf = bytes([0x06, 0x81, 0x08, ASDUADDR, 0xff, 0x00])
        su = time.localtime(time.time()).tm_isdst
        year = time.localtime(time.time()).tm_year
        month = time.localtime(time.time()).tm_mon
        mday = time.localtime(time.time()).tm_mday
        wday = time.localtime(time.time()).tm_wday
        # note that Monday is 0 etc. This is usually ignored by receiver so does not really matter
        hour = time.localtime(time.time()).tm_hour
        min = time.localtime(time.time()).tm_min
        sec = time.localtime(time.time()).tm_sec
        msec = time.time() % 1
        secBytes = int((sec + msec) * 1000)
        '''print(SECBytes)
        print((time.time()%60)*1000)'''
        minisValid = 1  # change to an expression if you know what this should be
        minisValid = int(not minisValid)
        databuf = databuf + bytes(
            [secBytes % 256, secBytes // 256, 128 * minisValid + min, su * 128 + hour, (wday << 5) + mday, month,
             year - 2000])
        FCB, FCV, fcode = self.requireResponse(response, FCB)
        self.m_lpdu.variable(FCB, FCV, fcode, ADDR=ADDR, ASDU=databuf)

    def sendASDU7(self, FCB: int, ADDR: int, ASDUADDR: int, SCN: int):  # Initiation of general interrogation
        databuf = bytes([0x07, 0x81, 0x09, ASDUADDR, 0xff, 0x00, SCN])
        self.m_lpdu.variable(FCB, FCV=1, fcode=3, ADDR=ADDR, ASDU=databuf)

    # hexadecimal
    def sendASDU0A(self):  # Generic data
        pass

    def sendASDU14(self, FCB: int, ADDR: int, ASDUADDR: int, FUN: int, INF: int, DCO: int, RII: int,
                   response=True):  # General command
        databuf = bytes([0x14, 0x81, 0x14, ASDUADDR, FUN, INF, DCO, RII])
        FCB, FCV, fcode = self.requireResponse(response, FCB)
        self.m_lpdu.variable(FCB, FCV, fcode, ADDR=ADDR, ASDU=databuf)

    def sendASDU15(self):  # Generic command
        pass

    def sendASDU18(self):  # Order for disturbance data transmission
        pass

    def sendASDU19(self):  # Acknowledgement for disturbance data transmission
        pass


def interpret(raw: list[bytes]) -> dict:
    dict1 = T103LPDU.deformat(raw)
    if dict1['format'] == 'fixed':
        return dict1
    ASDU = dict1['ASDU']
    del dict1['ASDU']
    '''    if len(ASDU) < 7:
        raise Exception("ASDU not enough length")'''
    TYPE = ASDU[0]
    if TYPE == 1:
        dict1.update(receiveASDU1(ASDU))
        return dict1
    elif TYPE == 2:
        dict1.update(receiveASDU2(ASDU))
        return dict1
    elif TYPE == 3:
        dict1.update(receiveASDU3(ASDU))
        return dict1
    elif TYPE == 4:
        dict1.update(receiveASDU4(ASDU))
        return dict1
    elif TYPE == 5:
        dict1.update(receiveASDU5(ASDU))
        return dict1
    elif TYPE == 6:
        dict1.update(receiveASDU6(ASDU))
        return dict1
    elif TYPE == 8:
        dict1.update(receiveASDU8(ASDU))
        return dict1
    elif TYPE == 9:
        dict1.update(receiveASDU9(ASDU))
        return dict1
    elif TYPE == 0xcd:
        dict1.update(receiveASDUCD(ASDU))
        return dict1
    else:
        print("type unsupported yet")
        return
        # raise Exception("type unsupported")


def deformatASDU(ASDU: list[bytes]) -> dict:
    TYPE = ASDU[0]
    VSQ = ASDU[1]
    COT = ASDU[2]
    ASDUADDR = ASDU[3]
    FUN = ASDU[4]
    INF = ASDU[5]
    INFO = ASDU[6:]
    return {'TYPE': TYPE, 'VSQ': VSQ, 'COT': COT, 'ASDUADDR': ASDUADDR, 'FUN': FUN, 'INF': INF, 'INFO': INFO}


def receiveASDU1(ASDU: list[bytes]) -> dict:  # time-tagged message
    dict1 = deformatASDU(ASDU)
    INFO = dict1['INFO']
    del dict1['INFO']
    if dict1['VSQ'] != 0x81 or len(INFO) != 6:
        raise Exception("illegal ASDU1 format")
    DPI = INFO[0]
    SEC = (INFO[1] + INFO[2] * 256) / 1000
    IV = INFO[3] >> 7
    MIN = INFO[3] % 128
    SU = INFO[4] >> 7
    HOUR = INFO[4] % 128
    SIN = INFO[5]

    dict2 = {'DPI': DPI, 'SEC': SEC, 'IV': IV, 'MIN': MIN, 'SU': SU, 'HOUR': HOUR, 'SIN': SIN}
    dict1.update(dict2)
    return dict1


def receiveASDU2(ASDU: list[bytes]) -> dict:  # time tagged message with relative time
    dict1 = deformatASDU(ASDU)
    INFO = dict1['INFO']
    del dict1['INFO']
    if dict1['VSQ'] != 0x81 or dict1['COT'] not in (1, 9) or len(INFO) != 10:
        raise Exception("illegal ASDU2 format")
    DPI = INFO[0]
    RET = (INFO[1] + INFO[2] * 256) / 1000
    FAN1 = INFO[3]
    FAN2 = INFO[4]
    SEC = (INFO[5] + INFO[6] * 256) / 1000
    IV = INFO[7] >> 7
    MIN = INFO[7] % 128
    SU = INFO[8] >> 7
    HOUR = INFO[8] % 128
    SIN = INFO[9]

    dict2 = {'DPI': DPI, 'SEC': SEC, 'IV': IV, 'MIN': MIN, 'SU': SU, 'HOUR': HOUR, 'SIN': SIN, 'RET': RET, 'FAN1': FAN1,
             'FAN2': FAN2}
    dict1.update(dict2)
    return dict1


def receiveASDU3(ASDU: list[bytes]) -> dict:  # measurands 1
    dict1 = deformatASDU(ASDU)
    INFO = dict1['INFO']
    del dict1['INFO']
    VSQ = dict1['VSQ']
    if dict1['COT'] != 2 or len(INFO) != 2 * VSQ:
        raise Exception("illegal ASDU3/9 format")
    Measurements = []
    for i in range(0, VSQ):
        data = (INFO[1 + 2 * i] << 5) + (INFO[2 * i] >> 3)
        if data >> 12 & 1 == 1:
            data = data - 2 ** 13
        OV = INFO[2 * i] & 1
        ER = INFO[2 * i] >> 1 & 1
        msdict = {'Measurement': data, 'OV': OV, 'ER': ER}
        Measurements.append(msdict)
    dict1['Measurements'] = Measurements
    return dict1


def receiveASDU4(ASDU: list[bytes]) -> dict:  # time-tagged measurands with relative timme
    dict1 = deformatASDU(ASDU)
    INFO = dict1['INFO']
    del dict1['INFO']
    if dict1['VSQ'] != 0x81 or dict1['COT'] != 9 or len(INFO) != 12:
        raise Exception("illegal ASDU4 format")
    # F = INFO[0] / (2 ** 23) + INFO[1] / (2 ** 15) + (INFO[2] % 128) / (2 ** 7)
    # S = INFO[3] >> 7
    # E = INFO[3] % 128 * 2 + INFO[2] // 128
    # SCL = (1 - 2 * S) * (1 + F) * 2 ** (E - 127)  # this is equivalent to
    SCL = struct.unpack('f', INFO[0:4])[0]
    RET = (INFO[4] + INFO[5] * 256) / 1000
    FAN1 = INFO[6]
    FAN2 = INFO[7]
    SEC = (INFO[8] + INFO[9] * 256) / 1000
    IV = INFO[10] >> 7
    MIN = INFO[10] % 128
    SU = INFO[11] >> 7
    HOUR = INFO[11] % 128
    dict2 = {'SCL': SCL, 'RET': RET, 'FAN1': FAN1, 'FAN2': FAN2, 'SEC': SEC, 'IV': IV, 'MIN': MIN, 'SU': SU,
             'HOUR': HOUR}
    dict1.update(dict2)
    return dict1


def receiveASDU5(ASDU: list[bytes]) -> dict:  # identification message
    dict1 = deformatASDU(ASDU)
    INFO = dict1['INFO']
    del dict1['INFO']
    COL = INFO[0]
    if dict1['VSQ'] != 0x81 or dict1['COT'] not in (3, 4, 5) or len(INFO) != 13 or dict1['INF'] not in (
            2, 3, 4) or COL not in (2, 3):
        raise Exception("illegal ASDU5 format")
    ASCII = INFO[1:9].decode("utf8")
    VER = INFO[9:13].decode("utf8")
    dict2 = {'COL': COL, 'ASCII': ASCII, 'VER': VER}
    dict1.update(dict2)
    return dict1


def receiveASDU6(ASDU: list[bytes]) -> dict:  # time synchronization
    dict1 = deformatASDU(ASDU)
    INFO = dict1['INFO']
    del dict1['INFO']
    if dict1['VSQ'] != 0x81 or dict1['COT'] !=8 or len(INFO) != 7 or dict1['FUN']!=255 or dict1['INF']!=0:
        raise Exception("illegal ASDU6 format")

    SEC = (INFO[0] + INFO[1] * 256) / 1000
    IV = INFO[2] >> 7
    MIN = INFO[2] % 128
    SU = INFO[3] >> 7
    HOUR = INFO[3] % 128
    WDAY=INFO[4]>>5
    MDAY=INFO[4]%32
    MONTH=INFO[5]%16
    YEAR=INFO[6]%128

    dict2 = {'SEC': SEC, 'IV': IV, 'MIN': MIN, 'SU': SU, 'HOUR': HOUR, 'WDAY':WDAY,'MDAY':MDAY,'MONTH':MONTH,'YEAR':YEAR}
    dict1.update(dict2)
    return dict1

def receiveASDU8(ASDU: list[bytes]) -> dict:  # termination of general interrogation
    dict1 = deformatASDU(ASDU)
    INFO = dict1['INFO']
    del dict1['INFO']
    if dict1['VSQ'] != 0x81 or dict1['COT'] != 0x0a or len(INFO) != 1 or dict1['FUN'] != 0xff or dict1['INF'] != 0x00:
        raise Exception("illegal ASDU8 format")
    SIN = INFO[0]
    dict1['SIN'] = SIN
    return dict1


def receiveASDU9(ASDU: list[bytes]) -> dict:  # measurands 2
    return receiveASDU3(ASDU)


def receiveASDU0A(ASDU: list[bytes]) -> dict:  # generic data
    pass


def receiveASDU0B(ASDU: list[bytes]) -> dict:  # generic identification
    pass


def receiveASDU17(ASDU: list[bytes]) -> dict:  # list of recorded disturbances
    pass


def receiveASDU1A(ASDU: list[bytes]) -> dict:  # ready for transmission of disturbance data
    pass


def receiveASDU1B(ASDU: list[bytes]) -> dict:  # ready for transmission of a channel
    pass


def receiveASDU1C(ASDU: list[bytes]) -> dict:  # ready for transmission of tags
    pass


def receiveASDU1D(ASDU: list[bytes]) -> dict:  # transmission of tags
    pass


def receiveASDU1E(ASDU: list[bytes]) -> dict:  # transmission of disturbance values
    pass


def receiveASDU1F(ASDU: list[bytes]) -> dict:  # end of transmission
    pass


def receiveASDUCD(ASDU: list[bytes]) -> dict:
    dict1 = deformatASDU(ASDU)
    INFO = dict1['INFO']
    del dict1['INFO']
    if dict1['VSQ'] != 0x81 or dict1['COT'] != 1 or len(INFO) != 8:
        raise Exception("illegal ASDUCD format")
    CR = INFO[0] + (INFO[1] << 8) + (INFO[2] << 16) + ((INFO[3] & 0b11111) << 24)
    if INFO[3] >> 4 & 1 == 1:
        CR = CR - 2 ** 21
    um = (INFO[3] >> 5) & 1
    fi = (INFO[3] >> 6) & 1
    fe = (INFO[3] >> 7) & 1
    SEC = (INFO[4] + INFO[5] * 256) / 1000
    IV = INFO[6] >> 7
    MIN = INFO[6] // 128
    SU = INFO[7] >> 7
    HOUR = INFO[7] % 128

    dict2 = {'CR': CR, 'um': um, 'fi': fi, 'fe': fe, 'SEC': SEC, 'IV': IV, 'MIN': MIN, 'SU': SU, 'HOUR': HOUR}
    dict1.update(dict2)

    return dict1
    '''
    def timedReceive(self):
        while timerthread.is_alive():
            received = self.m_lpdu.m_serial.receive()
            if received:
                print(received.hex(" "))


def timer(timing):
    time.sleep(timing)


if __name__ == '__main__':
    h = T103ASDU(port='COM2')
    threadLock = threading.Lock()

    timerthread = threading.Thread(target=timer, args=[10])
    # timerthread.start()

    # thread1 = threading.Thread(target=h.timedReceive)
    # thread1.start()
    '''
