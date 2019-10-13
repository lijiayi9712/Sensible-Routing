import subprocess
import argparse
import os
import sys
import atexit
import time

import datetime
import pandas as pd
from xml.etree import ElementTree

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit('please declare environment variable "SUMO_HOME"')

import traci
import sumolib
from sumolib import checkBinary
import numpy as np

net = sumolib.net.readNet('mini.net.xml')
lane = 'DC03_0'

shape_info = net.getLane(lane).getShape(lane)
#print(shape_info)
if (shape_info[-1][0] - shape_info[0][0]) < 0 or (shape_info[-1][1] - shape_info[0][1]) < 0:
	direction = -1 
else:
	direction = 1 #left to right or bottom to up

delta_x = shape_info[-1][0] - shape_info[0][0]
delta_y = shape_info[-1][1] - shape_info[0][1]


if delta_y == 0: 
	vertical = 0
else:
	vertical = 1 

length = max(delta_x * direction, delta_y * direction)
length = int(length)
unit_size= 8
n = int((length - 30) // unit_size) # number of street parking slots that need to be assigned 
a = 90

tree = ElementTree.parse("mini.sumocfg")
root = tree.getroot()
inputs = root.find('./input')
additionals = inputs.find('./additional-files').attrib['value']


print("start building bounding_boxes")

print(direction)
print(vertical)
for i in range(n):
	P_name = 'P' + lane + str(i)
	if vertical == 0:
		a = 90
		if direction == -1:
			x, y = shape_info[-1] # (x, y) is the starting point
			x, y = int(x), int(y)
			y_min = y + 5
			y_max = y + 10
			x_min = x + 12 + i*unit_size + 10
			x_max = x + 12 + (i+1)*unit_size + 10
			s = length - (15 + (i+1)*unit_size)
			e = length - (15 + i*unit_size)
		else:
			x, y =  shape_info[0]
			x, y = int(x), int(y)
			y_min = y - 5
			y_max = y 
			x_min = x + 12 + i*unit_size + 10
			x_max = x + 12 + (i+1)*unit_size + 10
			s = 12 + i*unit_size
			e = 12 + (i+1)*unit_size
	else:
		a = 0
		if direction == 1:
			x, y =  shape_info[0]
			x, y = int(x), int(y)
			x_min = x
			x_max = x + 5
			y_min = y + 12 + i*unit_size + 10
			y_max = y + 12 + (i+1)*unit_size + 10
			s = 12 + i*unit_size 
			e = 12 + (i+1)*unit_size
		else:
			x, y =  shape_info[-1]
			x, y = int(x), int(y)
			x_min = x - 5
			x_max = x
			y_min = y + i*unit_size + 10
			y_max = y + (i+1)*unit_size + 10
			s = length - (15 + (i+1)*unit_size)
			e = length - (15 +(i)*unit_size)


	bounding_box = str(int(x_min)) + "," + str(int(y_min)) + "," + str(int(x_max)) + "," + str(int(y_max))
	additionals += ", parking_" + P_name + ".add.xml"
	arguments = "-i " + str(P_name) + " -x " + str(x) + " -y " + str(y) + " -b " + str(bounding_box) + " -n 1 -c " + lane + " -a " + str(a) + " -s " + str(s) + " -e " + str(e)
	print("python generateParkingLots.py " + arguments)
	os.system("python generateParkingLots.py " + arguments)

inputs.find('./additional-files').set('value', additionals)
tree.write('mini.sumocfg')
#print(inputs.find('./additional-files').attrib['value'])
# print(c.get('begin'))
#c.set('additional-files','Completed')

#root.insert(1, c)
	#python generateParkingLots.py -i 'P5' -x 10 -y 0 -b "20,5,35,15" -n 1 -c lane -a 90 -s 70 -e 80

