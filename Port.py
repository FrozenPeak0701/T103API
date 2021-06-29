import serial


class Port:
    def __init__(self, port="COM1", baudrate=9600, bytesize=serial.EIGHTBITS, parity=serial.PARITY_EVEN,
                 stopbits=serial.STOPBITS_ONE, timeout=0.1):
        self.m_serialport = serial.Serial(port=port, baudrate=baudrate, bytesize=bytesize, parity=parity,
                                          stopbits=stopbits, timeout=timeout)
        self.log = []

    def send(self, databuf, length):
        print("Sent: ", databuf.hex(" "))  # this is temporary, or not...
        self.log.append("Sent: " + databuf.hex(" "))
        if (self.m_serialport.write(databuf) != length):
            print("Send Error")

    def receive(self):
        received = self.m_serialport.read(1024)
        print("Received: ", received.hex(" "))  # this is temporary, or not...
        self.log.append("Received: " + received.hex(" "))
        # if (self.m_serialport.inWaiting()>0):
        return received

    def __del__(self):
        self.m_serialport.close()
