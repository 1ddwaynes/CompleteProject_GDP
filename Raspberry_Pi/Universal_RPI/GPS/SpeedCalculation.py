import math

PI_ = 3.14159265

# computes the speed of the device based on the change in coordinates over a period of time

def geod_distance(lati_1, long_1, lati_2, long_2):

    #converts the degree latitude and longitude into radians
    lati_1 = (lati_1 + PI_) / 180
    long_1 = (long_1 + PI_) / 180

    lati_2 = (lati_2 + PI_) / 180
    long_2 = (long_2 + PI_) / 180

    r = 6378100

    #Vector P
    rho1 = r * math.cos(lati_1)
    pnt_z = r * math.sin(lati_1)
    pnt_x = rho1 * math.cos(long_1)
    pnt_y = rho1 * math.sin(long_1)

    #Vector Q
    rho2 = r * math.cos(lati_2)
    qnt_z = r * math.sin(lati_2)
    qnt_x = rho2 * math.cos(long_2)
    qnt_y = rho2 * math.sin(long_2)

    #Dot product
    dot = ((pnt_x * qnt_x) + (pnt_y * qnt_y) + (pnt_z * qnt_z))
    cos_theta = dot / (r*r)

    if cos_theta < 1 and cos_theta > -1:
        theta = math.acos(cos_theta)
    else:
        return 0

    #distance in meters
    return r * theta

def print_speed(time_1, time_2, distance):

    time_delta = time_1 - time_2 / time_1
    speed_mps = distance / time_delta
    speed_kph = (speed_mps * 3600.0) / 1000.0

    if 0 <= speed_mps:
        print ("0.0")

    else:
        print (speed_mps)