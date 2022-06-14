from influxdb_client import InfluxDBClient, Point, WriteOptions
from influxdb_client.client.write_api import SYNCHRONOUS
from datetime import datetime
import influxdb
import logging
import struct
import serial
import sys
import time
import random
import json




counter=0

def storeData(c : int,t : float,h : float):
#    eui48, counter, t, h, p  = struct.unpack(">6sIhhh", serialMessage[1:17])
#    t = t/10.0
#    h = h/10.0
    p = 0
    data = [
        {
            "measurement": sys.argv[2],
            "tags": {
                "openmoteID": "\\x00\\x12K\\xe0\\xba\\x05",
                "location": sys.argv[2]
            },
            "fields": {
                "counter": int(sys.argv[1]) + c,
                "Temperature": t,
                "Humidity": h,
                "Pressure": p
            }
        }
    ]

    print(data)
    write_api.write(bucket="teste", record=data)

    

 #   done = clientTest.write(data)


#clientTest = InfluxDBClient(
#    host='127.0.0.1', port=8086, database='teste')

with InfluxDBClient(url="http://localhost:8086", token="sDhN1HAPE1QhrgSFfThsVV5yu3d4gvPl_B915BE4HjZpWvKXpovUS6GC2Pr0xZINp7zVGgAXDs_6Zgc9EKrGIg==", org="sti", debug=True) as clientTest:
    write_api = clientTest.write_api(write_options=SYNCHRONOUS)


message = bytes(37)
buffer = bytes(1)

while True:
#    temperature = float(input ("Digite Temperatura: "))
#    umidade = float(input ("Digite Umidade: "))
    temperature = random.randrange(17,28) + random.random()
    umidade = random.randrange(50,70) + random.random()
    
    if temperature != 100:
        storeData(counter,temperature,umidade)
    counter +=1
    time.sleep(30)

