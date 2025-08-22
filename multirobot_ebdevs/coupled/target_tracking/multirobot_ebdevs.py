#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

# Import code for DEVS model representation:
from pypdevs.DEVS import CoupledDEVS

# Import all models to couple
from multirobot_ebdevs.atomics.integrators.qss1tools import evaluate_poly
from multirobot_ebdevs.atomics.communication.transmission_medium import (
    TransmissionMedium
)
from multirobot_ebdevs.atomics.misc.logger import Logger

# our coupled models
from multirobot_ebdevs.coupled.target_tracking.hunter import Hunter


class MultiRobotSystem(CoupledDEVS):
    def __init__(
            self,
            world_config,
            simu_config,
            robots_config,
            name='MultiRobotSystem',
            log_period=None,
            log_path='./',
            debug=False
            ):
        """
        Multi robot system composed of N robots.
        """
        # Always call parent class' constructor FIRST:
        CoupledDEVS.__init__(self, name)

        self.log_path = log_path
        self.debug = debug

        # TODO: time cannot be managed as in the other coupled/atomic models
        self.current_time = 0.0

        router = self.addSubModel(TransmissionMedium(
            robots_ids=list(robots_config.keys()),
            name='TransmissionMedium',
            debug=self.debug
        ))
        self.addSubModel(Logger(
            period=log_period,
            name='Logger',
            log_path=self.log_path,
            debug=self.debug
        ))

        self.robot_states = {}
        self.target_states = {}
        self.adjacency_list = {}
        for name in world_config.keys():
            if name.startswith('Hunter'):
                robot = self.addSubModel(Hunter(
                    world_config[name],
                    simu_config[name],
                    robots_config[name],
                    name=name,
                    log_path=self.log_path,
                    debug=self.debug
                ))
                self.connectPorts(
                    robot.outPorts['radio'],
                    router.inPorts[name]
                )
                self.connectPorts(
                    router.outPorts[name],
                    robot.inPorts['radio']
                )
                self.adjacency_list[name] = []

            elif name.startswith('Target'):
                # target_states must be initialized at the very beginning
                self.target_states[name] = {
                    'time': 0.0,
                    'pose': world_config[name]['position'],
                    'collect_range': world_config[name]['collect_range'],
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

        self.robot_states[micro_id] = data.copy()

        robot_pos = [[coord[0]] for coord in data['pose']]
        for target in self.target_states.values():
            sq_dist = np.sum(np.square(np.subtract(robot_pos, target['pose'])))
            if sq_dist < target['collect_range']**2:
                target['status'] = 'passive'

        if (self.debug):
            print(
                "t: {} s, Coupled name: {}, Global Transition Function,\
                x_b_micro: {}, global state: {}"
                .format(data['time'], self.name, x_b_micro, self.robot_states)
                )

    def getGlobalState(self, current_time):
        robot_data = []
        for robot_id in self.robot_states.keys():
            position = self.getRobotPosition(robot_id, current_time)
            robot_data += [coord[0] for coord in position]

        target_data = []
        for target_id in self.target_states.keys():
            state = self.target_states[target_id]
            target_data += [coord[0] for coord in state['pose']]
            target_data += [1.0 if state['status'] == 'active' else 0.0]

        adjacency_list = self.adjacency_list

        return robot_data, target_data, adjacency_list

    def getRobotPosition(self, robot_id, current_time):
        state = self.robot_states[robot_id]
        return [
            [evaluate_poly(poly, current_time - t)]
            for t, poly in zip(state['time'], state['pose'])
        ]

    def getNeighbors(self, robot_1_id, current_time):
        # need to know the current time to make the polynomial advance in time
        robot_1_pos = self.getRobotPosition(robot_1_id, current_time)
        robot_1_comm_range = self.robot_states[robot_1_id]['comm_range']

        neighbors = [
            robot_2_id
            for robot_2_id in self.robot_states.keys()
            if robot_1_id != robot_2_id and
            self.inRange(
                robot_1_pos, robot_1_comm_range, robot_2_id, current_time
            )
        ]
        self.adjacency_list[robot_1_id] = neighbors

        return neighbors

    def inRange(
            self, robot_1_pos, robot_1_comm_range, robot_2_id, current_time
            ):
        robot_2_pos = self.getRobotPosition(robot_2_id, current_time)
        sq_dist = np.sum(np.square(np.subtract(robot_1_pos, robot_2_pos)))

        return sq_dist < robot_1_comm_range**2

    def getNearestTarget(self, robot_id, current_time):
        robot_pos = self.getRobotPosition(robot_id, current_time)
        try:
            target_pos = min([
                (
                    np.sum(np.square(np.subtract(robot_pos, target['pose']))),
                    target['pose']
                )
                for target in self.target_states.values()
                if target['status'] == 'active'
            ])[1]
        except ValueError:
            target_pos = None

        return target_pos

    def select(self, immChildren):
        """
        Choose a model to transition from all possible models.
        """
        # Doesn't really matter, as they don't influence each other
        return immChildren[0]
