import GPS
import threading

# debugging
if __name__ == '__main__':
    t = threading.Thread(target = GPS.useGPS())
    t.start()