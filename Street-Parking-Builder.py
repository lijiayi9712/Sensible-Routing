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
edge = 'CD03'#
lane = edge +  "_0"

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

length = max(delta_x * direction, delta_y * direction) - 15
length = int(length)
unit_size= 6
delta = 3
n = int((length - 20) // (unit_size+delta)) # number of street parking slots that need to be assigned 
a = 90


tree = ElementTree.parse("mini.sumocfg")
root = tree.getroot()
inputs = root.find('./input')
additionals = inputs.find('./additional-files').attrib['value']


print("start building bounding_boxes")
print(n)
print(shape_info)

for i in range(n):
	if vertical == 0:
		a = 90
		if direction == -1:
			curr_i = n-1-i
			P_name = 'P' + lane + str(curr_i)
			x, y = shape_info[-1] # (x, y) is the starting point
			x, y = int(x), int(y)
			y_min = y + 5
			y_max = y + 10
			x_min = x + 12 + i*unit_size + 2
			x_max = x + 12 + (i+1)*unit_size + 2
			s = length - (15 + (i+1)*unit_size)
			e = length - (15 + i*unit_size)
		else:
			P_name = 'P' + lane + str(i)
			x, y =  shape_info[0]
			x, y = int(x), int(y)
			y_min = y - 5
			y_max = y 
			x_min = x + 12 + i*unit_size + 13 + i*delta
			x_max = x_min + unit_size
			s = 12 + i*unit_size + i*delta
			e = s + unit_size
	else:
		a = 0
		if direction == 1:
			P_name = 'P' + lane + str(i)
			x, y =  shape_info[0]
			x, y = int(x), int(y)
			x_min = x + 5
			x_max = x + 10
			y_min = y + 12 + i*unit_size + 10
			y_max = y + 12 + (i+1)*unit_size + 10
			s = 12 + i*unit_size 
			e = 12 + (i+1)*unit_size
		else:
			curr_i = n-1-i
			P_name = 'P' + lane + str(curr_i)
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



"""
=============================================================================================================================
Setting up rerouter for street parking on current lane
"""
from xml.etree import ElementTree

# if os.path.exists("mini.rerouters.add.xml") == False:

tree = ElementTree.parse("mini.rerouters.add.xml")
root = tree.getroot()
def rerouter_builder(n, lane, tree, root):
	for i in range(n):
	    b = ElementTree.SubElement(root, 'rerouter')
	    id_i = 'P' + str(lane) + str(i)
	    b.set('id', id_i)
	    b.set('edges', edge)
	    curr_bi = 'b_{}'.format(str(i))
	    curr_bi = ElementTree.SubElement(b, 'interval')
	    curr_bi.set('begin', '0.0')
	    curr_bi.set('end', '20000')
	    for j in range(n):
	        b_ij = ElementTree.SubElement(curr_bi, 'parkingAreaReroute')
	        curr_slot = (i+j)%n
	        b_ij.set('id', 'P' + lane + str(curr_slot))
	        b_ij.set('visible', 'true')
	        tree.write('mini.rerouters.add.xml')

import xml.dom.minidom as minidom

rerouter_builder(n, lane, tree, root)

def prettify(elem):
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="\t")


#prettify once in the end

# pretty_elem = prettify(root)
# with open("mini.rerouters.add.xml", "w") as f:
#     f.write(pretty_elem)
