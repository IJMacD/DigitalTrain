#!/usr/bin/env python
f = open("main.py")
data = f.read().replace("\\", "\\\\").replace("\"", "\\\"")

print ("Writing:\n" + data)

import serial
s = serial.Serial("/dev/ttyS5", 115200)
s.write("f = open(\"main.py\")\n")
s.write("f.write(\" + data + \")")
s.close()