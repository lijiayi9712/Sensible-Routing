import subprocess
import argparse
import os
import sys
import atexit
import time

import datetime
import pandas as pd


if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit('please declare environment variable "SUMO_HOME"')

import traci
import sumolib
from sumolib import checkBinary
import numpy as np


def init():
    netconvert_cmd = [
        'netconvert',
        '--node-files=mini.nod.xml',
        '--edge-files=mini.edg.xml',
        '--type-files=mini.type.xml',
        '--output-file=mini.net.xml'
    ]
    subprocess.run(netconvert_cmd)
    sumoBinary = checkBinary('sumo-gui')
    step_size = 0.5
    config = [
        sumoBinary,
        '-c', 'mini.sumocfg',
        '--step-length', '{:.2f}'.format(step_size),
        '--no-step-log'
    ]
    traci.start(config)

def close():
	traci.close()
	sys.stdout.flush()

if __name__ == '__main__':
	init()
	veh_id='veh01'
	route_id='init'
	route=['BB03_rev']
	traci.route.add(route_id, route)
	traci.vehicle.addFull(
            vehID=veh_id,
            routeID=route_id,
            departSpeed=0,
    )
	traci.vehicle.setParkingAreaStop(
            vehID=veh_id, stopID='PBB03_rev_02', duration=2)
	traci.vehicle.setParkingAreaStop(
            vehID=veh_id, stopID='PBB03_rev_04', duration=2)
	traci.vehicle.setParkingAreaStop(
            vehID=veh_id, stopID='PBB03_rev_05', duration=1)
	for t in range(100):
		traci.simulationStep()
	close()