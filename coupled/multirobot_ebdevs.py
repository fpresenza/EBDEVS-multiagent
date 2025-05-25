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

# Import all models to couple
from atomics.integrators.qss1tools import evaluate_poly, pad_zeros
from atomics.communication.router import Router
from atomics.misc.logger import Logger

# our coupled models
from coupled.robot import Robot
from coupled.target import Target

from utils.files import (
    read_json_file,
    append_csv_file
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

        # TODO: time cannot be managed as in the other coupled/atomic models
        self.current_time = 0.0

        robots_config = read_json_file('robots.json')
        targets_config = read_json_file('targets.json')

        self.router = self.addSubModel(Router(
            robots_ids=list(robots_config.keys()),
            targets_ids=list(targets_config.keys()),
            name='Router',
            debug=self.debug
        ))
        self.logger = self.addSubModel(Logger(
            period=0.1,
            name='Logger',
            logpath=self.logpath,
            debug=self.debug
        ))

        self.agents_states = {}
        self.robots = {}
        for robot_id, config in robots_config.items():
            self.robots[robot_id] = self.addSubModel(Robot(
                config,
                name=robot_id,
                logpath=self.logpath,
                debug=self.debug
            ))
            self.connectPorts(
                self.robots[robot_id].outPorts['radio'],
                self.router.inPorts[robot_id]
            )
            self.connectPorts(
                self.router.outPorts[robot_id],
                self.robots[robot_id].inPorts['radio']
            )

        self.targets = {}
        for target_id, config in targets_config.items():
            self.targets[target_id] = self.addSubModel(Target(
                config,
                name=target_id,
                debug=self.debug
            ))
            self.connectPorts(
                self.targets[target_id].outPorts['radio'],
                self.router.inPorts[target_id]
            )  # target -> router
            self.connectPorts(
                self.router.outPorts[target_id],
                self.targets[target_id].inPorts['radio']
            )  # router -> target

            # targets_states must be initialized at the very beginning
            self.agents_states[target_id] = {
                'time': 0.0,
                'pose': [pad_zeros(coord) for coord in config["position"]],
                'comm_range': config["comm_range"],
                'status': 'active',
            }

        if (self.debug):
            print("t: 0 s, Coupled name: {}, Init Function".format(self.name))

    def globalTransition(self, e_g, x_b_micro, *args, **kwargs):
        # self.current_time += e_g
        if len(x_b_micro) == 1:
            micro_id, data = x_b_micro[0]
        else:
            micro_id, data = x_b_micro

        self.agents_states[micro_id] = data.copy()

        # log new value of micro_states
        log = [micro_id, data['time']]
        log += [data['pose'][0][0], data['pose'][1][0]]
        log += [data['comm_range']]

        if micro_id.startswith('Robot'):
            log += [
                neighbor_id
                for neighbor_id in self.agents_states.keys()
                if self.in_range(micro_id, neighbor_id, 0.0)
                # checks and registers current neighboring robots
            ]
        elif micro_id.startswith('Target'):
            log += [data['status']]

        append_csv_file(self.logpath + 'global.csv', log)

        if (self.debug):
            print(
                "t: {} s, Coupled name: {}, Global Transition Function,\
                x_b_micro: {}, global state: {}"
                .format(data['time'], self.name, x_b_micro, self.agents_states)
                )

    def getNeighbors(self, agent_1_id, current_time):
        # need to know the current time to make the polynomial advance in time
        return [
            agent_2_id
            for agent_2_id in self.agents_states.keys()
            if self.in_range(agent_1_id, agent_2_id, current_time)
        ]

    def getRobotPositions(self, current_time):
        ids = []
        positions = []
        comm_ranges = []
        status = []
        for robot, state in self.agents_states.items():
            ids.append(robot)
            comm_ranges.append(state['comm_range'])
            try:
                status.append(state['status'])
            except KeyError:
                status.append('')
            previous_time = state['time']
            delta_time = current_time - previous_time
            position_poly = state['pose']
            x = evaluate_poly(position_poly[0], delta_time)
            y = evaluate_poly(position_poly[1], delta_time)
            positions += [x, y]
        return ids, positions, comm_ranges, status

    def getRobotDistances(self, robot_1, robot_2, current_time):
        pose = self.agents_states[robot_1]['pose']
        delta_time = current_time - self.agents_states[robot_1]['time']
        x1 = evaluate_poly(pose[0], delta_time)
        y1 = evaluate_poly(pose[1], delta_time)

        pose = self.agents_states[robot_2]['pose']
        delta_time = current_time - self.agents_states[robot_2]['time']
        x2 = evaluate_poly(pose[0], delta_time)
        y2 = evaluate_poly(pose[1], delta_time)

        return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def select(self, immChildren):
        """
        Choose a model to transition from all possible models.
        """
        # Doesn't really matter, as they don't influence each other
        return immChildren[0]

    def in_range(self, agent_1_id, agent_2_id, current_time):
        # agent_1_id might be robot or target
        if agent_1_id == agent_2_id:
            return False

        # tweak to improve performance since target-target comm is not needed
        if agent_1_id.startswith('Target') and agent_2_id.startswith('Target'):
            return False

        distance = self.getRobotDistances(
            agent_1_id, agent_2_id, current_time
        )
        trasmitter_range = self.agents_states[agent_1_id]['comm_range']

        return distance < trasmitter_range
