
import subprocess
import os
import sys
import atexit
import time


if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit('please declare environment variable "SUMO_HOME"')

import traci
import sumolib
from sumolib import checkBinary
import numpy as np



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

veh_states = {} #mapping each vehicle to a tuple (stop_counter, stop_set, route_set, mission)


def step(time, missions, idle_vehs, working_vehs):
    random_num = np.random.uniform(0, 1)

    if random_num <= 0.5 and len(idle_vehs) > 0:
        mission_index = np.random.choice(len(missions))
        mission = missions[mission_index]
        mission_veh = idle_vehs.pop()
        working_vehs.append(mission_veh)
        # route = choose_route(mission[0], mission[1])[:-1] + \
        #         choose_route(mission[1], mission[2])[:-1] + \
        #         choose_route(mission[2], mission[3])

        
        # route_id = str(mission_veh) + \
        #            mission[0] + '_' + \
        #            mission[1] + '_' + \
        #            mission[2] + '_' + \
        #            mission[3] + '_' + \
        #            str(time)

        veh_states[mission_veh] = (0, 0, 0, mission)

        traci.vehicle.setParkingAreaStop(
            vehID=mission_veh, 
            stopID=mission[0], 
            duration=2,
        )

        for t in range(1):
            traci.simulationStep()
        finished = []

    for veh in working_vehs: 
        counter = veh_states[veh][0]
        stop_set = veh_states[veh][1]
        route_set = veh_states[veh][2]
        veh_stops = veh_states[veh][3]
        
        try: 
            traci.vehicle.isStoppedParking(veh)
 
        except traci.exceptions.TraCIException:
            print(veh)
            print(veh_stops)

            print(counter)
            print(stop_set)
            print(route_set)

        if traci.vehicle.isStoppedParking(veh) and stop_set == 0 and route_set == 0 and counter < 3:
            earlier_stop = veh_stops[counter] 
            new_stop = veh_stops[counter+1]
            earlier_stop = veh_stops[counter]
            new_stop = veh_stops[counter+1]

            route_seg = choose_route(earlier_stop, new_stop)
            route_id = str(veh) + \
                   earlier_stop + '_' + \
                   new_stop + '_' + \
                   str(time)
            traci.route.add(route_id, route_seg)
            traci.vehicle.setRoute(veh, route_seg)
            traci.simulationStep()
            route_set = 1

            duration_func = lambda x: 20 if x != 'P4' else 1000000
            traci.vehicle.setParkingAreaStop(
                vehID=veh, 
                stopID=new_stop,
                duration = duration_func(new_stop), 
            )
            stop_set = 1
            counter += 1
        # if stop_set == 0 and traci.vehicle.isStoppedParking(veh) == False and counter < 3:
            
        
        if traci.vehicle.isStoppedParking(veh) == False:
            stop_set = 0
            route_set = 0
        veh_states[veh] = (counter, stop_set, route_set, veh_stops)

        
        # if traci.vehicle.isStoppedParking(veh):
        #     veh_states[veh] = (counter, stop_set, veh_stops)
        # if traci.vehicle.isStoppedParking(veh) == False and stop_set == 0 and counter < 3: 
        #     print("set stop 1")
        #     earlier_stop = veh_stops[counter]
        #     new_stop = veh_stops[counter+1]

        #     route_seg = choose_route(earlier_stop, new_stop)


        #     route_id = str(veh) + \
        #            earlier_stop + '_' + \
        #            new_stop + '_' + \
        #            str(time)
            
        #     traci.route.add(route_id, route_seg)

        #     traci.vehicle.setRoute(veh, route_seg)

        #     print("set stop 2")
        #     print(route_seg)
        #     traci.simulationStep()
            
        #     duration_func = lambda x: 20 if x != 'P4' else 1000000

            

        #     traci.vehicle.setParkingAreaStop(
        #         vehID=veh, 
        #         stopID=new_stop,
        #         duration = duration_func(new_stop), 
        #     )
        #     print("set stop 3")
        #     veh_states[veh] = (counter+1, 1, 1, veh_stops)

        if traci.vehicle.getSpeed(veh) == 0.0 and traci.vehicle.getRouteIndex(veh) == 'CB01':
            finished.append(veh)
            idle_vehs.append(veh)
            veh_states[veh] = (0, 0, 0, veh_route)
        working_vehs = [veh for veh in working_vehs if veh not in idle_vehs]
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
            vehID=veh_id, stopID='P4', duration=1000000)
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
    duration = 50000
    working_vehs = []
    for time in range(duration):
        step(time, missions, idle_vehs, working_vehs)
    close()

