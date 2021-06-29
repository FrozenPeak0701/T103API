import serial, time


class smartPort:
    def __init__(self, port="COM1", baudrate=9600, bytesize=serial.EIGHTBITS, parity=serial.PARITY_EVEN,
                 stopbits=serial.STOPBITS_ONE, timeout=0.05):
        self.m_serialport = serial.Serial(port=port, baudrate=baudrate, bytesize=bytesize, parity=parity,
                                          stopbits=stopbits, timeout=timeout)
        self.log = []

    def send(self, databuf, length):
        print("Sent: ", databuf.hex(" "))  # this is temporary, or not...
        self.log.append("Sent: " + databuf.hex(" "))
        if (self.m_serialport.write(databuf) != length):
            print("Send Error")

    def receive(self):
        received = self.m_serialport.read()
        if received is not None:
            while True:
                if self.m_serialport.inWaiting() > 0:
                    #print(1)
                    received += self.m_serialport.read(self.m_serialport.inWaiting())
                else:
                    time.sleep(
                        0.016)  # arbitrary, represent the maximum time interval between two bytes in transmission
                    # also, the sleep function has accuracy around 0.01~0.02 sec so this is just signaling the program to wait for a really small amount of time
                    if self.m_serialport.inWaiting() == 0:
                        break

        print("Received: ", received.hex(" "))  # this is temporary, or not...
        self.log.append("Received: " + received.hex(" "))
        # if (self.m_serialport.inWaiting()>0):
        return received

    def __del__(self):
        self.m_serialport.close()

