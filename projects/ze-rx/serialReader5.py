from influxdb import InfluxDBClient
from datetime import datetime
import influxdb
import logging
import struct
import serial
import sys

logging.basicConfig(filename="log",
                    filemode='a',
                    format='%(asctime)s,%(msecs)d %(name)s %(levelname)s %(message)s',
                    datefmt='%d-%m-%Y|%H:%M:%S',
                    level=logging.DEBUG)

try:
    serialPort = serial.Serial(port=f'/dev/ttyUSB{sys.argv[1]}', baudrate=115200)
except serial.SerialException as e:
    logging.error(e)
    print(e)
    exit()


def storeData(serialMessage : bytes):

 #    eui48, counter, txMode, txCounter, csma_retries, csma_rssi = struct.unpack(
    eui48, counter, t, h, p, txMode, txCounter, csma_retries, csma_rssi, rssi = struct.unpack(">6sIhhhbbBbb", serialMessage[1:22])
#    rssi, _ = struct.unpack(">bb", message[22:23])
    t = t/100.0
    h = h/100.0
    p = p/100.0

    data = [
        {
            "measurement": sys.argv[2],
            "tags": {
                "openmoteID": str(eui48),
                "location": sys.argv[2]
            },
            "fields": {
                "counter": counter,
                "Temperature": t,
                "Humidity": h,
                "Pressure": p,
                "txMode": txMode,
                "txCounter": txCounter,
                "rssi": rssi,
                "csma_retries": csma_retries,
                "csma_rssi": csma_rssi

   
             }
        }
    ]

    print(data)

    done = clientTest.write_points(data, protocol="json")


clientTest = InfluxDBClient(
    host='127.0.0.1', port=8086, database='final')

message = bytes(37)
buffer = bytes(1)

while True:
    try:
        buffer += serialPort.read(1)

        if buffer[-1] == 126:
            message = buffer
            buffer = bytes(1)
            if len(message) > 2:
                storeData(message)
    except Exception as e:
        logging.error(e)
        print(e)
