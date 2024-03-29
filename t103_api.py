# t103_api
# original author: Zeyu Ma

import t103_asdu
import serial, time, threading, eventlet
import serial.tools.list_ports
from functools import wraps

'''def lockThemAll(lock):  # 用于统一加lock的装饰器
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            lock.acquire()
            returned = func(*args, **kwargs)
            lock.release()
            return returned
        return wrapper
    return decorator'''  # not realizable


def singleton(cls):  # 对于每个Port的单例模式装饰器
    _instance = []

    def inner(*args, **kwargs):
        for i in _instance:
            if len(args) > 0 and i.port == args[0]:
                return i
            if i.port == kwargs["port"]:
                return i
        _instance.append(cls(*args, **kwargs))
        return _instance[-1]

    return inner


@singleton
class T103API:
    def __init__(self, port="COM1", baudrate=9600, bytesize=serial.EIGHTBITS, parity=serial.PARITY_EVEN,
                 stopbits=serial.STOPBITS_ONE, timeout=0.05, FCBORCU: bool = 1,
                 ADDR: int = 1, pollinterval: int = 3):  # FCVORCUR: 1: reset CU,0: resetFCB
        self.port = port
        self.ADDR = ADDR
        self.m_ASDU = t103_asdu.T103ASDU(port=port, baudrate=baudrate, bytesize=bytesize, parity=parity,
                                         stopbits=stopbits,
                                         timeout=timeout)
        self.FCB = 0
        self.lock = threading.RLock()
        self.stopevent = threading.Event()
        # self.timeSync(ASDUADDR=self.ADDR, response=False)
        if int(FCBORCU):
            msg = self.resetCU()
        else:
            msg = self.resetFCB()
        # print(msg)

        self.thread1 = threading.Thread(target=self.CyclicPoll, kwargs={"interval": pollinterval}, daemon=True)
        self.thread1.start()

    def __del__(self):
        self.stopevent.set()
        self.thread1.join()
        # del self.m_ASDU # seems to work without it, no idea why

    def disconnect(self):
        self.__del__()

    def send3times(self, sendfunc, **args):  # all send commands should go through this function!!!
        for i in range(1, 4):
            sendfunc(**args, ADDR=self.ADDR)
            received = self.m_ASDU.m_lpdu.m_serial.receive()
            if len(received) != 0:
                self.FCB = not self.FCB
                return received
        raise Exception("receive time out")

    def CyclicPoll(self, interval):
        while True:
            if self.stopevent.is_set():
                self.stopevent.clear()
                break
            self.lock.acquire()
            self.__requestClass2Data()
            self.lock.release()
            time.sleep(interval)

    def repeatRequestClass1Data(self):  # 连续请求一级数据
        self.lock.acquire()
        receivebuffer = []
        while True:
            dict1 = self.__requestClass1Data()
            if dict1 is None:
                continue
            if dict1['format'] != 'fixed':
                receivebuffer.append(dict1)
            if (dict1['ACD'] == 0 and dict1['fcode'] in (8, 9)) or dict1['fcode'] == 9:
                break
        self.lock.release()
        return receivebuffer

    def get_counter(self):  # 持续请求一级数据，直到请求到脉冲量为止
        while True:
            receivebuffer = []
            buffer = self.repeatRequestClass1Data()
            for i in buffer:
                if i is not None and i['format'] == 'variable' and i['TYPE'] == 205:
                    receivebuffer.append(i)
            if len(receivebuffer) != 0:
                return receivebuffer

    def __requestClass1Data(self):  # 单次请求一级数据
        self.lock.acquire()
        received = self.send3times(self.m_ASDU.m_lpdu.fixed, FCB=self.FCB, FCV=1, fcode=10)
        dict1 = t103_asdu.interpret(received)
        if dict1 is None:
            # raise Exception("cannot interpret this frame")
            self.lock.release()
            return
        if dict1['ADDR'] != self.ADDR:
            raise Exception("sent ADDR is not consistent with received ADDR")
        self.lock.release()
        return dict1

    def __requestClass2Data(self):  # 单次请求二级数据/遥测
        self.lock.acquire()
        received = self.send3times(self.m_ASDU.m_lpdu.fixed, FCB=self.FCB, FCV=1, fcode=11)
        dict1 = t103_asdu.interpret(received)
        if dict1['ADDR'] != self.ADDR:
            raise Exception("sent ADDR is not consistent with received ADDR")
        self.lock.release()
        return dict1

    def repeatRequestClass2Data(self, requestc1: bool = False,
                                waitTime: float = 1):  # 请求二级数据/遥测 requestc1为True时：若需要，则继续请求一级数据
        self.lock.acquire()
        receivebuffer = []
        time.sleep(waitTime)
        while True:
            received = self.send3times(self.m_ASDU.m_lpdu.fixed, FCB=self.FCB, FCV=1, fcode=11)
            dict1 = t103_asdu.interpret(received)
            if dict1['ADDR'] != self.ADDR:
                raise Exception("sent ADDR is not consistent with received ADDR")
            if dict1['fcode'] == 9:
                break
            receivebuffer.append(dict1)
        if dict1['ACD'] == 1 and requestc1:
            receivebuffer += self.repeatRequestClass1Data()
        self.lock.release()
        return receivebuffer

    def get_cyclic_measurement(self, requestc1: bool = False,
                        waitTime: float = 2):  # 遥测快捷方式,返回一个以(FUN,INF）为key的字典，便于索引, value仍然为字典
        dict1 = {}
        for i in range(0, 2):
            receivebuffer = self.repeatRequestClass2Data(requestc1=requestc1, waitTime=waitTime)
            for point in receivebuffer:
                asdustr = 'ASDU%d' % point['TYPE']
                details = point.copy()
                if asdustr in dict1:
                    dict2 = dict1[asdustr]
                    dict2[(point['FUN'], point['INF'])] = details
                else:
                    dict2 = {(point['FUN'], point['INF']): details}
                    dict1[asdustr] = dict2
        return dict1

    def __request_spec_class1_data(self, FUN: int, INF: int, timeout=5):  # 持续请求命令相应
        self.lock.acquire()
        eventlet.monkey_patch()
        try:
            with eventlet.Timeout(timeout):
                while True:
                    received = self.send3times(self.m_ASDU.m_lpdu.fixed, FCB=self.FCB, FCV=1, fcode=10)
                    dict1 = t103_asdu.interpret(received)
                    if dict1 is None:
                        # raise Exception("cannot interpret this frame")
                        self.lock.release()
                        return
                    if dict1['ADDR'] == self.ADDR and dict1['format'] == 'variable' and dict1['FUN'] == FUN and dict1[
                        'INF'] == INF:
                        self.lock.release()
                        return dict1
                    # else:
                    #     if dict1['ADDR'] == self.ADDR and dict1['format'] == 'variable':
                    #         print('Other variable datagrams are received, need to check:', dict1)
                    #     continue
        except eventlet.timeout.Timeout:
            self.lock.release()
            print('Time out, no expected response')

    def __get_command_ack(self, FUN: int, INF: int, RII=0, timeout=5):  # 持续请求命令相应
        return self.__request_spec_class1_data(FUN, INF, timeout)

    def get_changed_status(self, FUN: int, INF: int, RII=0, timeout=5):  # 持续请求状态变化
        return self.__request_spec_class1_data(FUN, INF, timeout)

    def general_command(self, FUN: int, INF: int, DCO: int, RII=0, timeout=5) -> dict:  # 遥控
        self.lock.acquire()
        received = self.send3times(self.m_ASDU.sendASDU14, FCB=self.FCB, ASDUADDR=self.ADDR, FUN=FUN, INF=INF,
                                   DCO=DCO, RII=RII)
        dict1 = t103_asdu.interpret(received)
        self.lock.release()
        return self.__get_command_ack(FUN, INF, DCO, timeout)

    def GI(self, ASDUADDR, SCN):  # 总召/遥信  返回3个list，可能有空
        self.lock.acquire()
        received = self.send3times(self.m_ASDU.sendASDU7, FCB=self.FCB, ASDUADDR=ASDUADDR, SCN=SCN)
        dict1 = t103_asdu.interpret(received)
        errorbuffer = []
        gi_receivebuffer = []
        spn_receivebuffer = []
        if dict1['ADDR'] != self.ADDR:
            raise Exception("sent ADDR is not consistent with received ADDR")
        gotbuffer = self.repeatRequestClass1Data()
        for i in gotbuffer:
            if i is None:
                gi_receivebuffer.append(i)
            else:
                if i["COT"] in (9, 10) and i["TYPE"]!=4 and i["SIN"] != SCN:
                    errorbuffer.append(i)
                elif i['COT'] == 1:
                    spn_receivebuffer.append(i)
                else:
                    gi_receivebuffer.append(i)
        if gi_receivebuffer[-1]['TYPE'] != 8:
            raise Exception('Unexpected stop')
        self.lock.release()
        return gi_receivebuffer, spn_receivebuffer, errorbuffer

    def time_sync(self, ASDUADDR, response=True):  # 对时
        self.lock.acquire()
        if not response:
            self.m_ASDU.sendASDU6(FCB=self.FCB, ADDR=self.ADDR, ASDUADDR=ASDUADDR, response=response)
            self.FCB = not self.FCB
            self.lock.release()
            return
        received = self.send3times(self.m_ASDU.sendASDU6, FCB=self.FCB, ASDUADDR=ASDUADDR, response=response)
        dict1 = t103_asdu.interpret(received)
        receivebuffer = []
        if dict1 is None:
            self.lock.release()
            return
        if dict1['ADDR'] != self.ADDR:
            raise Exception("sent ADDR is not consistent with received ADDR")
        if dict1['ACD'] == 1:
            receivebuffer += self.repeatRequestClass1Data()
        self.lock.release()
        return receivebuffer

    def resetFCB(self):  # reset frame count bit复位帧计数位
        self.lock.acquire()
        received = self.send3times(self.m_ASDU.m_lpdu.fixed, FCB=0, FCV=0, fcode=7)
        dict1 = t103_asdu.interpret(received)
        if dict1['fcode'] != 0:
            raise Exception('protection equipment did not confirm')
        self.FCB = 1  # 从站的FCB被设为0，所以下一次主站发送的message的FCB必须是1
        receivebuffer = self.__requestClass1Data()
        self.lock.release()
        return receivebuffer

    def resetCU(self):  # reset communication unit复位通信单元
        self.lock.acquire()
        received = self.send3times(self.m_ASDU.m_lpdu.fixed, FCB=0, FCV=0, fcode=0)
        dict1 = t103_asdu.interpret(received)
        if dict1['fcode'] != 0:
            raise Exception('protection equipment did not confirm')
        self.FCB = 1  # 从站的FCB被设为0，所以下一次主站发送的message的FCB必须是1
        receivebuffer = self.__requestClass1Data()
        self.lock.release()
        return receivebuffer

    def requestLinkState(self):
        self.lock.acquire()
        received = self.send3times(self.m_ASDU.m_lpdu.fixed, FCB=0, FCV=0, fcode=9)
        dict1 = t103_asdu.interpret(received)
        if dict1['fcode'] != 11:
            raise Exception('protection equipment did not respond to request')
        self.lock.release()
        return [dict1]

    def printLog(self, filename):
        self.lock.acquire()
        file = open(filename, mode="a+")
        file.write("---------------------------------------Print----------------------------------------\n")
        file.write("-----------------------------" + time.asctime(
            time.localtime(time.time())) + "-------------------------------\n")

        for i in self.m_ASDU.m_lpdu.m_serial.log:
            file.write(i + "\n")
        file.write("\n")
        file.flush()
        file.close()
        self.lock.release()


def demo():
    # find comport
    portList = list(serial.tools.list_ports.comports())
    if portList is not None:
        comports = str(portList[0]).split('-')

    h = T103API(port=comports[0], timeout=1, FCBORCU=1, ADDR=1)  # FCVORCUR: 1: reset CU, 0: reset FCB

    while True:
        print("\n")
        print("----------------------------------------------")
        print("0.Request Class 1 data only one time")
        print("1.Request Class 1 data")
        print("2.Request Class 2 data")
        print("3.GetPulse")
        print("6.Time Synchronization")
        print("7.General Interrogation")
        print("8.Reset Frame Count Bit")
        print("9.Reset Communication Unit")
        print("10.Request Link State")
        print("11.Measurement")
        print("20.General Command")
        print("100.Print log to file")

        num = int(input("Type your input:\n"))
        if num == 0:
            print(h.__requestClass1Data())
        elif num == 1:
            list1 = h.repeatRequestClass1Data()
            for i in list1:
                print(i)
        elif num == 2:
            list1 = h.repeatRequestClass2Data()
            for i in list1:
                print(i)
        elif num == 3:
            list1 = h.getPulse()
            for i in list1:
                print(i)
        elif num == 6:
            list1 = h.timeSync(ASDUADDR=1)
            for i in list1:
                print(i)
        elif num == 7:
            list1, unexpectedlist, errorlist = h.GI(ASDUADDR=1, SCN=4)
            for i in list1:
                print(i)
            print('Spontaneous:', len(unexpectedlist))
            for i in unexpectedlist:
                print(i)
            print('Errors:', len(errorlist))
            for i in errorlist:
                print(i)
        elif num == 8:
            list1 = h.resetFCB()
            print(list1)
        elif num == 9:
            list1 = h.resetCU()
            print(list1)
        elif num == 10:
            list1 = h.requestLinkState()
            print(list1)
        elif num == 11:
            dict1 = h.measurement()
            for key in dict1:
                print(key, dict1[key])
        elif num == 20:
            list1 = h.generalCommand(ASDUADDR=1, FUN=192, INF=19, DCO=2, RII=3)
            for i in list1:
                print(i)
        elif num == 100:
            filename = input("Input file name to write log to:\n")
            h.printLog(filename)

        else:
            break


if __name__ == "__main__":
    p = T103API(port='COM2', timeout=0.1, pollinterval=10)
    a = p.get_measurement()
    for eve in a:
        print(eve, ":", a[eve])
    time.sleep(3)
    assert 1==2

    # a=p.generalCommand(FUN=240, INF=160, DCO=2,response=True)
    # a = p.getPulse()
    # b = p.repeatRequestClass1Data()

    # print(a)
    # print(b)

    # try:
    #     demo()
    #
    # except Exception as  e:
    #     print(e)
    #
