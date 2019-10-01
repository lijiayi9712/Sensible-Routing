import traci
import sumolib
from sumolib import checkBinary
import numpy as np
import subprocess
import os
import sys
import atexit
import time

# export SUMO_HOME="/Users/LiJiayi/sumo"
np.random.seed(42)  # make tests reproducible 
SIM_PHASE = 1800 #each simulation runs 30 minutes
parkingArea = {'CB01': 'P4', 'AA02': 'P1', 'BC04': 'P2', 'DD02_rev': 'P3'}
shortest_paths = {}
shortest_paths[('P4', 'P1')] = [
    ['CB01', 'BB01', 'BA02', 'AA02'], 
    ['CB01', 'BA01', 'AA01', 'AA02']
]
shortest_paths[('P4', 'P2')] = [
    ['CB01', 'BB01', 'BB02', 'BB03', 'BC04']
]
shortest_paths[('P4', 'P3')] = [
    ['CB01', 'BB01', 'BB02', 'BC03', 'CD03', 'DD02_rev']
]
shortest_paths[('P1', 'P2')] = [
    ['AA02', 'AA03', 'AB04', 'BC04'], 
    ['AA02', 'AB03', 'BB03', 'BC04']
]
shortest_paths[('P1', 'P3')] = [
    ['AA02', 'AB03', 'BC03', 'CD03', 'DD02_rev']
]
shortest_paths[('P1', 'P4')] = [
    ['AA02', 'AB03', 'BC03', 'CC02_rev', 'CC01_rev','CB01'], 
    ['AA02', 'AB03', 'BB02_rev', 'BC02', 'CC01_rev','CB01']
]
shortest_paths[('P2', 'P3')] = [
    ['BC04', 'CC03_rev', 'CD03', 'DD02_rev'], 
    ['BC04', 'CD04', 'DD03_rev', 'DD02_rev']
]
shortest_paths[('P2', 'P1')] = [
    ['BC04', 'CC03_rev', 'CB03', 'BB02_rev', 'BA02', 'AA02'], 
    ['BC04', 'CC03_rev', 'CC02_rev', 'CB02', 'BA02', 'AA02']
]
shortest_paths[('P2', 'P4')] = [
    ['BC04', 'CC03_rev', 'CC02_rev', 'CC01_rev', 'CB01']
]
shortest_paths[('P3', 'P2')] = [
    ['DD02_rev', 'DC02', 'CB02', 'BB02', 'BB03', 'BC04']
]
shortest_paths[('P3', 'P1')] = [
    ['DD02_rev', 'DC02', 'CB02', 'BA02', 'AA02']
]
shortest_paths[('P3', 'P4')] = [
    ['DD02_rev', 'DC02', 'CC01_rev', 'CB01'], 
    ['DD02_rev', 'DD01_rev', 'DC01', 'CB01']
]


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


def step(time, missions, idle_vehs, working_vehs):
    random_num = np.random.uniform(0, 1)
    if random_num <= 0.1 and len(idle_vehs) > 0:
        mission_index = np.random.choice(len(missions))
        mission = missions[mission_index]
        print(mission)
        mission_veh = idle_vehs.pop()
        working_vehs.append(mission_veh)
        route = choose_route(mission[0], mission[1])[:-1] + \
                choose_route(mission[1], mission[2])[:-1] + \
                choose_route(mission[2], mission[3])
        route_id = str(mission_veh) + \
                   mission[0] + '_' + \
                   mission[1] + '_' + \
                   mission[2] + '_' + \
                   mission[3] + '_' + \
                   str(time)
        traci.route.add(route_id, route)
        traci.vehicle.setRoute(mission_veh, route)
        duration_func = lambda x: 20
        traci.vehicle.setParkingAreaStop(
            vehID=mission_veh, 
            stopID=mission[3], 
            duration=0,
        )
        for _ in range(20):
            traci.simulationStep()
        traci.vehicle.setParkingAreaStop(
            vehID=mission_veh, 
            stopID=mission[1], 
            duration=duration_func(mission[1]),
        )
        traci.vehicle.setParkingAreaStop(
            vehID=mission_veh, 
            stopID=mission[2], 
            duration=duration_func(mission[2]),
        )
        traci.vehicle.setParkingAreaStop(
            vehID=mission_veh, 
            stopID=mission[3], 
            duration=duration_func(mission[3]),
        )
    traci.simulationStep()
        

def choose_route(origin, destination):
    routes = shortest_paths[(origin, destination)]
    route_index = np.random.choice(len(routes))
    return routes[route_index]
    

def close():
    traci.close()
    sys.stdout.flush()


if __name__ == '__main__':
    print('Initializing...')
    init()
    route_id = 'init'
    route = ['CB01']
    traci.route.add(route_id, route)
    p4_occupancy = traci.simulation.getParameter('P4', 'parkingArea.occupancy')
    p4_occupancy = int(p4_occupancy)
    idle_vehs = []    
    for veh_index in range(100):
        veh_id = 'veh_' + str(veh_index)
        traci.vehicle.addFull(
            vehID=veh_id,
            routeID=route_id,
            departSpeed=0,
        )
        idle_vehs.append(veh_id)
        traci.vehicle.setParkingAreaStop(
            vehID=veh_id, stopID='P4', duration=10000)
        p4_instoccupancy = traci.simulation.getParameter(
            'P4', 'parkingArea.occupancy')
        p4_instoccupancy = int(p4_instoccupancy)    
        while p4_instoccupancy < p4_occupancy + 1:
            traci.simulationStep()
            p4_instoccupancy = traci.simulation.getParameter(
                'P4', 'parkingArea.occupancy')
            p4_instoccupancy = int(p4_instoccupancy)    
        p4_occupancy = p4_occupancy + 1
    print("Initialization completed.")
    missions = [
        ("P4", "P1", "P2", "P4"), 
        ("P4", "P1", "P3", "P4"), 
        ("P4", "P2", "P1", "P4"), 
        ("P4", "P2", "P3", "P4"), 
        ("P4", "P3", "P1", "P4"), 
        ("P4", "P3", "P2", "P4")
    ]
    duration = 5000
    working_vehs = []
    for time in range(duration):
        step(time, missions, idle_vehs, working_vehs)
    close()

