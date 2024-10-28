# Copyright 2014 Modelling, Simulation and Design Lab (MSDL) at 
# McGill University and the University of Antwerp (http://msdl.cs.mcgill.ca/)
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np

# Import code for DEVS model representation:
from pypdevs.DEVS import CoupledDEVS
from pypdevs.infinity import INFINITY

# Import all models to couple
from atomics.qsstools import evaluate_poly
from atomics.router import Router

# our coupled models
from coupled.robot import Robot

from utils.files import (
    read_json_file,
    append_csv_file,
    robot_id_to_index
)


class MultiRobotSystem(CoupledDEVS):
    def __init__(self, name='MultiRobotSystem', logpath='./', debug=False):
        """
        Multi robot system composed of N robots.
        """
        # Always call parent class' constructor FIRST:
        CoupledDEVS.__init__(self, name)

        self.logpath = logpath
        self.debug = debug

        self.current_time = 0.0 # TODO: time cannot be managed as in the other coupled/atomic models

        robots_config = read_json_file('robots.json')

        self.router = self.addSubModel(
            Router(agents_ids=[robot['name'] for robot in robots_config.values()], 
                   name='Router',
                   debug=self.debug
                   )
        )

        self.robots_states = {}
        self.robots = {}
        for robot in robots_config.values():
            robot_id = robot['name']
            i = robot_id_to_index(robot_id)
            self.robots[robot_id] = self.addSubModel(
                Robot(
                    **robot,
                    logpath=self.logpath, 
                    debug=self.debug
                )
            )
            self.connectPorts(self.robots[robot_id].OUT_router_token, self.router.in_agent_token[robot_id])
            self.connectPorts(self.router.out_agent_token[robot_id], self.robots[robot_id].IN_router_token)

        self.distance_meas_stddev = 2.0

        if (self.debug):
            print("t: 0 s, Coupled name: {}, Init Function".format(self.name))

    def globalTransition(self, e_g, x_b_micro, *args, **kwargs):
        # self.current_time += e_g
        micro_id, data = x_b_micro
        try:
            self.robots_states[micro_id] = data.copy()
            current_time = data['Time'].copy()
        except AttributeError:
            self.robots_states[micro_id] = data
            current_time = data['Time']
        
        # print("Coupled name: {}, Global Transition Function, micro_id: {}".format(self.name,micro_id))
        if (self.debug):
            print(
                 "t: {} s, Coupled name: {}, Global Transition Function, x_b_micro: {}, global state: {}"
                 .format(current_time, self.name,x_b_micro,self.robots_states)
                 )

        # log new value of micro_states
        log = [micro_id, data['Time']]
        log += [data['Pose'][0][0],data['Pose'][1][0]]
        log += [data['CommRange']]
        log += [
            neighbor_id
            for neighbor_id in self.robots_states.keys()
            if self.connected(micro_id, neighbor_id, 0.0)
        ]
        append_csv_file(self.logpath + 'global.csv', log)

    def getContextInformation(self, robot_1_id, current_time):
        # need to know the current time to make the polynomial advance in time
        previous_time = self.robots_states[robot_1_id]['Time']
        delta_time = current_time - previous_time
        return [
            (robot_2_id, self.distance_measurement(robot_1_id, robot_2_id, delta_time))
            for robot_2_id in self.robots_states.keys()
            if self.connected(robot_1_id, robot_2_id, delta_time)
        ]

    def select(self, immChildren):
        """
        Choose a model to transition from all possible models.
        """
        # Doesn't really matter, as they don't influence each other
        return immChildren[0]

    def distance(self, robot_1_id, robot_2_id, delta_time):
        robot_1_pose = self.robots_states[robot_1_id]['Pose']
        robot_2_pose = self.robots_states[robot_2_id]['Pose']

        x1 = evaluate_poly(robot_1_pose[0], delta_time)
        y1 = evaluate_poly(robot_1_pose[1], delta_time)
        x2 = evaluate_poly(robot_2_pose[0], delta_time)
        y2 = evaluate_poly(robot_2_pose[1], delta_time)

        return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def distance_measurement(self, robot_1_id, robot_2_id, delta_time):
        return np.random.normal(
            loc=self.distance(robot_1_id, robot_2_id, delta_time),
            scale=self.distance_meas_stddev    
        ) 

    def connected(self, robot_1_id, robot_2_id, delta_time):
        if robot_1_id == robot_2_id:
            return False

        distance = self.distance(robot_1_id, robot_2_id, delta_time)

        trasmiter_range = self.robots_states[robot_1_id]['CommRange']
        receiver_range = self.robots_states[robot_2_id]['CommRange']

        return (distance < trasmiter_range) and (distance < receiver_range)
