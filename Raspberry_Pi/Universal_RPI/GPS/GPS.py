import BiSerial
import SpeedCalculation
import time


# class initGPS(threading.Thread):
class InitGPS:
    def __init__(self):
        # threading.Thread.__init__(self)

        # number of satellites detected by GPS
        self.SatN = None
        # current latitude and longitude values
        self.c_lat_value = None
        self.c_lon_value = None
        # old latitude and longitude values
        self.o_lat_value = None
        self.o_lon_value = None
        # once start default to turned on
        self.collect = True

    # used to disable polling for GPS values, only necessary for polling timeouts
    def disable_GPS(self):
        collect = False

    def get_current_GPS_cord(self):
        return self.c_lat_value, self.c_lon_value

    def get_current_num_SatN(self):
        return self.SatN

    # check data for string, handshakes for certain data stream within serial buffer
    # otherwise incorrect data can be passed through causing invalid coordinate assignment
    # could possible be expanded on
    def check_instance(self, data):
        if data:
            if "Sat" in data:
                data = data.replace("Sat", "")
                self.SatN = float(data)
                # there's a problem with how Sat data is being sent from Arduino
                # should be unnecessary in the future
                # data = data[:-6]
                # self.SatN = float(data)
            elif "Lat" in data:
                # print ("lat")
                data = data.replace("Lat", "")
                self.c_lat_value = float(data)
            elif "Lon" in data:
                # print ("Lon")
                data = data.replace("Lon", "")
                self.c_lon_value = float(data)

    # store passed valid cordial data for speed calculation
    def old_instance(self):
        self.o_lat_value = self.c_lat_value
        self.o_lon_value = self.c_lon_value

    # checks for repeated cord, prevents useless iteration of the speed calculation module
    def isStale(self):
        if self.c_lat_value == self.o_lat_value or self.c_lon_value == self.o_lon_value:
            return True
        else:
            return False

    # deletes/resets the current cordial data
    def reset_c(self):
        self.c_lat_value = None
        self.c_lon_value = None


# Main module for GPS interfacing, could be moved to a separate modulate for possible threading implementation
def useGPS():
    # initialize variables
    _serial = BiSerial.InitSerial()
    GPS = InitGPS()

    # connects to serial if serial is NOT connected
    if _serial.connected == 0:
        _serial.connect_Serial()

    while GPS.collect is True:
        #
        t_stamp1 = time.time()

        # resets GPS values if GPS values exist
        if GPS.c_lat_value and GPS.c_lon_value:
            GPS.reset_c()

        # main loop that reads GPS values
        while GPS.c_lat_value is None or GPS.c_lon_value is None:
            event_data = _serial.serial_event(str)
            if event_data is None:
                pass
            else:
                GPS.check_instance(event_data)
                # first time stamp to be stored, time.time() is used for linux polling
                t_stamp1 = time.time()
                #time.sleep(.5)

        # print(GPS.c_lat_value)
        # print(GPS.c_lon_value)
        # print(GPS.SatN)

        if GPS.o_lat_value is None and GPS.o_lon_value is None:
            GPS.old_instance()

        # if GPS.isStale is False:
        distance = SpeedCalculation.geod_distance(GPS.c_lat_value, GPS.c_lat_value, GPS.o_lon_value,
                                                  GPS.o_lon_value)

        t_stamp2 = time.time()
        SpeedCalculation.print_speed(t_stamp1, t_stamp2, distance)
        GPS.old_instance()
        # print(GPS.get_current_num_SatN)


# debugging
if __name__ == '__main__':
    useGPS()