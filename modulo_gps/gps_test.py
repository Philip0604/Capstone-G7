import serial
from geopy import distance
import time

port = "COM3"  # "/dev/ttyACM0" para linux
baud = 9600
gps = serial.Serial(port, baud)
interval = 1 #

m_s = 0
lat = 0
lon = 0
old_pos = (0, 0)
new_pos = (0, 0)
old_msg = ""


def ddm_to_dd(lat_lon, coord):  # coord es lat o lon [ddmm.sssss, (direccion)]
    sign = 1
    if coord[1] in ["W", "S"]:
        sign = -1

    if lat_lon == "lat":
        deg = coord[0][0:2]
        minutes = coord[0][2:]
    elif lat_lon == "lon":
        deg = coord[0][0:3]
        minutes = coord[0][3:]

    dd = (float(deg) + float(minutes)/60) * sign
    return dd


while True:
    msg = gps.readline().decode("utf-8").strip().split(",")

    if msg[0] == "$GPVTG":
        m_s = round(float(msg[7])/3.6, 5) # velocidad aproximada en m/s
    if msg[0] == "$GPGGA":
        old_pos = (lat, lon)
        lat = ddm_to_dd("lat", msg[2:4])
        lon = ddm_to_dd("lat", msg[4:6])
        new_pos = (lat, lon)

    advance = round(distance.distance(old_pos, new_pos).m, 5)
    if msg[0] == "$GPGGA":
        print(advance, m_s)