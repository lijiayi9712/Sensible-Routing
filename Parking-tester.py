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

    route_id='init'
    route=['CD03']
    traci.route.add(route_id, route)
    for i in range(20):  
        veh_idi='veh0'+str(i)
        traci.vehicle.addFull(
                vehID=veh_idi,
                routeID=route_id,
                departSpeed=0,
        )
        traci.vehicle.setParkingAreaStop(
                vehID=veh_idi, stopID='PCD03_00', duration=5)
        
        traci.vehicle.setParkingAreaStop(
                vehID=veh_idi, stopID='PCD03_01', duration=5)
        
        traci.vehicle.setParkingAreaStop(
                vehID=veh_idi, stopID='PCD03_02', duration=5)
        traci.vehicle.setParkingAreaStop(
                vehID=veh_idi, stopID='PCD03_04', duration=5)
        
        # traci.vehicle.setParkingAreaStop(
        #         vehID=veh_idi, stopID='PCD03_07', duration=5)
        for t in range(5):
            traci.simulationStep()

    # traci.vehicle.setParkingAreaStop(
    #         vehID=veh_id1, stopID='PDC03_02', duration=3)

    

    # traci.vehicle.addFull(
    #         vehID=veh_id2,
    #         routeID=route_id,
    #         departSpeed=0,
    # )
    # traci.vehicle.setParkingAreaStop(
    #         vehID=veh_id2, stopID='PDC03_07', duration=5)
    # traci.vehicle.setParkingAreaStop(
    #         vehID=veh_id2, stopID='PDC03_06', duration=5)
    # traci.vehicle.setParkingAreaStop(
    #         vehID=veh_id2, stopID='PDC03_06', duration=3)

    # for t in range(4):
    #     traci.simulationStep()

    # traci.vehicle.addFull(
    #         vehID=veh_id3,
    #         routeID=route_id,
    #         departSpeed=0,
    # )
    # traci.vehicle.setParkingAreaStop(
    #         vehID=veh_id3, stopID='PDC03_00', duration=5)
    # traci.vehicle.setParkingAreaStop(
    #         vehID=veh_id3, stopID='PDC03_04', duration=5)
    # traci.vehicle.setParkingAreaStop(
    #         vehID=veh_id3, stopID='PDC03_06', duration=3)


    for t in range(1000):
	    traci.simulationStep()
    close()
