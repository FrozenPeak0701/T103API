# link-protocol data unit layer api intended for t103_api use
# original author: Zeyu Ma

import smart_port
# import Port
import serial, threading, time


class T103LPDU:
    def __init__(self, port="COM1", baudrate=9600, bytesize=serial.EIGHTBITS, parity=serial.PARITY_EVEN,
                 stopbits=serial.STOPBITS_ONE, timeout=0.1):
        self.m_serial = smart_port.smartPort(port=port, baudrate=baudrate, bytesize=bytesize, parity=parity,
                                             stopbits=stopbits,
                                             timeout=timeout)

    # def __del__(self):
    # del self.m_serial # seems to work without it, no idea why

    def enCODE(self, FCB: int, FCV: int, fcode: int) -> int:
        if FCB > 1:
            raise Exception("illegal FCB")
        if FCV > 1:
            raise Exception("illegal FCV")
        if fcode not in (0, 3, 4, 7, 9, 10, 11):
            raise Exception("illegal function code")
        CODE = 64 + (FCB << 5) + (FCV << 4) + fcode
        return CODE

    def fixed(self, FCB: int, FCV: int, fcode: int, ADDR: int):
        CODE = self.enCODE(FCB, FCV, fcode)
        CS = (CODE + ADDR) % 256
        pdubuf = bytes([0x10, CODE, ADDR, CS, 0x16])
        self.m_serial.send(pdubuf, len(pdubuf))
        # print(self.socket.read(1024).hex(" "))

    def variable(self, FCB: int, FCV: int, fcode: int, ADDR: int, ASDU):
        CODE = self.enCODE(FCB, FCV, fcode)
        pdubuf = bytes([0x68, 2 + len(ASDU), 2 + len(ASDU), 0x68, CODE, ADDR]) + ASDU
        CS = (sum(ASDU) + CODE + ADDR) % 256
        pdubuf += bytes([CS, 0x16])
        self.m_serial.send(pdubuf, len(pdubuf))


def deformat(raw) -> dict:
    if len(raw) > 5:
        dict1 = {'format': 'variable'}
        dict1.update(devariable(raw))
        return dict1
    elif len(raw) == 5:
        dict1 = {'format': 'fixed'}
        dict1.update(defixed(raw))
        return dict1
    else:
        raise Exception("illegal frame length")


def defixed(raw) -> dict:
    CS = (raw[1] + raw[2]) % 256
    CODE = int(raw[1])
    ADDR = int(raw[2])
    PRM = (CODE >> 6) & 1
    ACD = (CODE >> 5) & 1
    DFC = (CODE >> 4) & 1
    fcode = CODE % 16
    if raw[0] != 0x10 or raw[-1] != 0x16 or CS != raw[3]:
        raise Exception("received illegal frame of fixed length")
    if PRM != 0:
        raise Exception("PRM received is not 0")
    if fcode not in (0, 1, 8, 9, 11):
        raise Exception("illegal function code")
    if DFC == 1:
        raise Exception("protection equipment buffer full, cannot receive more data")
    if fcode == 1:
        raise Exception("link busy, did not receive message")
    return {'CODE': CODE, 'ACD': ACD, 'DFC': DFC, 'fcode': fcode, 'ADDR': ADDR}


def devariable(raw) -> dict:
    ASDU = raw[6:][:-2]
    CODE = raw[4]
    ADDR = raw[5]
    CS = (sum(ASDU) + CODE + ADDR) % 256
    PRM = (CODE >> 6) & 1
    ACD = (CODE >> 5) & 1
    DFC = (CODE >> 4) & 1
    fcode = CODE % 16
    if raw[0] != raw[3] or raw[0] != 0x68 or raw[1] != raw[2] or raw[1] != len(ASDU) + 2 or CS != raw[-2] or raw[
        -1] != 0x16:
        raise Exception("received illegal frame of variable length")
    if PRM != 0:
        raise Exception("PRM received is not 0")
    if fcode not in (0, 1, 8, 9, 11):
        raise Exception("illegal function code")
    return {'CODE': CODE, 'ACD': ACD, 'DFC': DFC, 'fcode': fcode, 'ADDR': ADDR, 'ASDU': ASDU}
