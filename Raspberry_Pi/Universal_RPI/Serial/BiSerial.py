# requires the pyserial module to be install inorder to function
import serial
import time


class InitSerial():
    def __init__(self, *args):
        self.port_names = ("/dev/tty.usbserial-A9007UX1",  # MacOSX
                           "/dev/ttyACM0",  # Raspberry Pi
                           "/dev/ttyUSB0",  # Linux
                           "COM8",
                           "COM3")  # Window
        if args:
            for line in args:
                self.port_names.append(line)

        self.validport_address = None
        self.ser = None
        self.connected = 0

    def establishedPort(self, port):
        self.validport_address = port

    def connect_Serial(self):
        #if data is None:
            print('Connecting to Arduino.')

            for try_port in self.port_names:
                try:
                    self.ser = serial.Serial(try_port, 9600, timeout=0.05)
                    # print("Connecting to Arduino using open port {}.").format(try_port)
                except Exception as e:
                    print ("Connection failed. Retrying")
                    print(e)
                else:
                    print("Connected")
                    #print("Connected to {}").format(try_port)
                    self.establishedPort(try_port)
                    self.connected = 1

    def serial_event(self, data_type):
        if self.connected is 1:
            if 0 < self.ser.inWaiting():
                data = self.ser.readline()
                if data and self.valid_data(data) is True:
                    try:
                        if data_type is str:
                                # try:
                            data = str(data.decode('utf-8'))
                        elif data_type is float:
                                # try:
                            data = float(data.decode('utf-8'))
                        elif data_type is int:
                                # try:
                            data = int(data.decode('utf-8'))
                    except Exception as e:
                        print("Readline Error")
                        print(e)
                    return data

    # closes port to prevent port locking
    def close_serial(self):
        while self.ser.is_open:
            try:
                self.ser.__del__()
                self.connected = 0
            except Exception as e:
                print("Error {} has occured while closing serial.").format(e)

        def _force():
            print("Force closing")
            self.ser.close()

    # force oMsg through the output buffer
    def outputStream(self, oMsg, force):
        if self.ser.out_waiting > 0 and force is True:
            self.ser.reset_output_buffer()
            print(str(oMsg))
            try:
                self.ser.write(oMsg)
            except Exception as e:
                print(e)
        else:
            try:
                self.ser.write(oMsg)
            except Exception as e:
                print(e)

    # not implemented yet, might not be necessary
    def inputStream(self):
        pass

    # checks for a valid value, invalid values are sent as a * byte
    def valid_data(self, data):
        if b'*' in data:
            return False
        else:
            return True

    def isConnected(self):
        if self.connected == 1:
            return True
        else:
            return False
