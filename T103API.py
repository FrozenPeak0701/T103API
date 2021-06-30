import T103ASDU
import serial, time, threading
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
def singleton(cls):    #单例模式装饰器
    _instance = {}
    def inner(*args,**kwargs):
        if cls not in _instance:
            _instance[cls] = cls(*args,**kwargs)
        return _instance[cls]
    return inner

@singleton
class T103API:
    def __init__(self, port="COM1", baudrate=9600, bytesize=serial.EIGHTBITS, parity=serial.PARITY_EVEN,
                 stopbits=serial.STOPBITS_ONE, timeout=0.05, FCBORCU: bool = 1,
                 ADDR: int = 1, pollinterval: int = 1):  # 1: reset CU,0: resetFCB
        self.port=port
        self.m_ASDU = T103ASDU.T103ASDU(port=port, baudrate=baudrate, bytesize=bytesize, parity=parity,
                                        stopbits=stopbits,
                                        timeout=timeout)
        self.FCB = 0
        self.lock = threading.RLock()
        if int(FCBORCU):
            msg = self.resetCU(ADDR)
        else:
            msg = self.resetFCB(ADDR)
        print(msg)

        self.startCyclicPoll(interval=pollinterval, ADDR=ADDR)

    def __del__(self):

        del self.m_ASDU
        del self.FCB

    def send3times(self, sendfunc, **args):  # all send commands should go through this function!!!
        for i in range(1, 4):
            sendfunc(**args)
            received = self.m_ASDU.m_lpdu.m_serial.receive()
            if len(received) != 0:
                self.FCB = not self.FCB
                return received
        raise Exception("receive time out")

    def startCyclicPoll(self, interval, ADDR):
        thread1 = threading.Thread(target=self.CyclicPoll, kwargs={"interval": interval, "ADDR": ADDR})
        thread1.start()

    def CyclicPoll(self, interval, ADDR):
        while True:
            self.lock.acquire()
            self.requestClass2Data(ADDR=ADDR)
            self.lock.release()
            time.sleep(interval)

    def repeatRequestClass1Data(self, ADDR: int):  # 连续请求一级数据
        self.lock.acquire()
        receivebuffer = []
        while True:
            dict1 = self.requestClass1Data(ADDR)
            receivebuffer.append(dict1)
            if dict1 is None:
                continue
            if dict1['ACD'] == 0 and dict1['fcode'] in (8, 9):
                break
        self.lock.release()
        return receivebuffer

    def requestClass1Data(self, ADDR: int):  # 单次请求一级数据
        self.lock.acquire()
        received = self.send3times(self.m_ASDU.m_lpdu.fixed, FCB=self.FCB, FCV=1, fcode=10, ADDR=ADDR)
        dict1 = T103ASDU.interpret(received)
        if dict1 is None:
            # raise Exception("cannot interpret this frame")
            self.lock.release()
            return
        if dict1['ADDR'] != ADDR:
            raise Exception("sent ADDR is not consistent with received ADDR")
        self.lock.release()
        return dict1

    def requestClass2Data(self, ADDR: int):  # 单次请求二级数据/遥测
        self.lock.acquire()
        received = self.send3times(self.m_ASDU.m_lpdu.fixed, FCB=self.FCB, FCV=1, fcode=11, ADDR=ADDR)
        dict1 = T103ASDU.interpret(received)
        if dict1['ADDR'] != ADDR:
            raise Exception("sent ADDR is not consistent with received ADDR")
        self.lock.release()
        return dict1

    def repeatRequestClass2Data(self, ADDR: int):  # 请求二级数据/遥测 (若需要，则继续请求一级数据)
        self.lock.acquire()
        received = self.send3times(self.m_ASDU.m_lpdu.fixed, FCB=self.FCB, FCV=1, fcode=11, ADDR=ADDR)
        dict1 = T103ASDU.interpret(received)
        if dict1['ADDR'] != ADDR:
            raise Exception("sent ADDR is not consistent with received ADDR")
        receivebuffer = [dict1]
        if dict1['ACD'] == 1:
            receivebuffer += self.repeatRequestClass1Data(ADDR)
        self.lock.release()
        return receivebuffer

    def generalCommand(self, ADDR: int, ASDUADDR: int, FUN: int, INF: int, DCO: int, RII=0, response=True):  # 遥控
        self.lock.acquire()
        received = self.send3times(self.m_ASDU.sendASDU14, FCB=self.FCB, ADDR=ADDR, ASDUADDR=ASDUADDR, FUN=FUN, INF=INF,
                                   DCO=DCO, RII=RII, response=response)
        dict1 = T103ASDU.interpret(received)
        receivebuffer = []
        if dict1 is None:
            self.lock.release()
            return
        if dict1['ADDR'] != ADDR:
            raise Exception("sent ADDR is not consistent with received ADDR")
        receivebuffer += self.repeatRequestClass1Data(ADDR)
        self.lock.release()
        return receivebuffer

    def GI(self, ADDR, ASDUADDR, SCN):  # 总召/遥信  返回3个list，可能有空
        self.lock.acquire()
        received = self.send3times(self.m_ASDU.sendASDU7, FCB=self.FCB, ADDR=ADDR, ASDUADDR=ASDUADDR, SCN=SCN)
        dict1 = T103ASDU.interpret(received)
        errorbuffer = []
        expectedreceivebuffer = []
        unexpectedreceivebuffer = []
        if dict1['ADDR'] != ADDR:
            raise Exception("sent ADDR is not consistent with received ADDR")
        gotbuffer = self.repeatRequestClass1Data(ADDR)
        for i in gotbuffer:
            if i is None:
                expectedreceivebuffer.append(i)
            else:
                if i["COT"] in (9, 10) and i["TYPE"] != 4 and i["SIN"] != SCN:
                    errorbuffer.append(i)
                elif i['COT'] == 1:
                    unexpectedreceivebuffer.append(i)
                else:
                    expectedreceivebuffer.append(i)
        if expectedreceivebuffer[-1]['TYPE'] != 8:
            raise Exception('Unexpected stop')
        self.lock.release()
        return expectedreceivebuffer, unexpectedreceivebuffer, errorbuffer

    def timeSync(self, ADDR, ASDUADDR, response=True):  # 对时
        self.lock.acquire()
        received = self.send3times(self.m_ASDU.sendASDU6, FCB=self.FCB, ADDR=ADDR, ASDUADDR=ASDUADDR, response=response)
        dict1 = T103ASDU.interpret(received)
        receivebuffer = []
        if dict1 is None:
            self.lock.release()
            return
        if dict1['ADDR'] != ADDR:
            raise Exception("sent ADDR is not consistent with received ADDR")
        if dict1['ACD'] == 1:
            receivebuffer += self.repeatRequestClass1Data(ADDR)
        self.lock.release()
        return receivebuffer

    def resetFCB(self, ADDR):  # reset frame count bit复位帧计数位
        self.lock.acquire()
        received = self.send3times(self.m_ASDU.m_lpdu.fixed, FCB=0, FCV=0, fcode=0, ADDR=ADDR)
        dict1 = T103ASDU.interpret(received)
        if dict1['fcode'] != 0:
            raise Exception('protection equipment did not confirm')
        self.FCB = 1  # 从站的FCB被设为0，所以下一次主站发送的message的FCB必须是1
        receivebuffer = self.requestClass1Data(ADDR)
        self.lock.release()
        return receivebuffer

    def resetCU(self, ADDR):  # reset communication unit复位通信单元
        self.lock.acquire()
        received = self.send3times(self.m_ASDU.m_lpdu.fixed, FCB=0, FCV=0, fcode=7, ADDR=ADDR)
        dict1 = T103ASDU.interpret(received)
        if dict1['fcode'] != 0:
            raise Exception('protection equipment did not confirm')
        self.FCB = 1  # 从站的FCB被设为0，所以下一次主站发送的message的FCB必须是1
        receivebuffer = self.requestClass1Data(ADDR)
        self.lock.release()
        return receivebuffer

    def requestLinkState(self, ADDR):
        self.lock.acquire()
        received = self.send3times(self.m_ASDU.m_lpdu.fixed, FCB=0, FCV=0, fcode=9, ADDR=ADDR)
        dict1 = T103ASDU.interpret(received)
        if dict1['fcode'] != 11:
            raise Exception('protection equipment did not respond to request')
        self.lock.release()

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
    h = T103API(port='COM2', timeout=1, FCBORCU=1, ADDR=1)  # FCVORCUR: 1: reset CU, 0: reset FCB

    #
    #
    while True:
        print("\n")
        print("----------------------------------------------")
        print("0.Request Class 1 data only one time")
        print("1.Request Class 1 data")
        print("2.Request Class 2 data")
        print("6.Time Synchronization")
        print("7.General Interrogation")
        print("8.Reset Frame Count Bit")
        print("9.Reset Communication Unit")
        print("10.Request Link State")
        print("20.General Command")
        print("100.Print log to file")

        num = int(input("Type your input:\n"))
        if num == 0:
            print(h.requestClass1Data(ADDR=1))
        elif num == 1:
            list1 = h.repeatRequestClass1Data(ADDR=1)
            for i in list1:
                print(i)
        elif num == 2:
            list1 = h.repeatRequestClass2Data(ADDR=1)
            for i in list1:
                print(i)
        elif num == 6:
            list1 = h.timeSync(ADDR=1, ASDUADDR=1, )
            for i in list1:
                print(i)
        elif num == 7:
            list1, unexpectedlist, errorlist = h.GI(ADDR=1, ASDUADDR=1, SCN=4)
            for i in list1:
                print(i)
            print('Spontaneous:', len(unexpectedlist))
            for i in unexpectedlist:
                print(i)
            print('Errors:', len(errorlist))
            for i in errorlist:
                print(i)
        elif num == 8:
            list1 = h.resetFCB(ADDR=1)
            print(list1)
        elif num == 9:
            list1 = h.resetCU(ADDR=1)
            print(list1)
        elif num == 10:
            list1 = h.requestLinkState(ADDR=1)
            print(list1)
        elif num == 20:
            list1 = h.generalCommand(ADDR=1, ASDUADDR=1, FUN=192, INF=19, DCO=2, RII=3)
            for i in list1:
                print(i)
        elif num == 100:
            filename = input("Input file name to write log to:\n")
            h.printLog(filename)

        else:
            break


if __name__ == "__main__":
    # p = T103API(port='COM2', timeout=0.1)
    demo()
    '''h=T103API(port='COM2', timeout=0.1)
    p=T103API(port='COM3', timeout=0.1)
    print(h.FCB)
    print(p.FCB)
    h.FCB=not h.FCB
    print(h.FCB)
    print(p.FCB)'''


