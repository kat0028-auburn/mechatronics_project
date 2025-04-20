#!/usr/bin/env python3
from serial import Serial   

portname = '/dev/ttyACM0'
baudrate = 115200

port = Serial(port=portname, baudrate=baudrate)

val1 = -2048
val2 = -2048
csv_line = f"{val1},{val2}\n"
port.write(csv_line.encode('utf-8'))
