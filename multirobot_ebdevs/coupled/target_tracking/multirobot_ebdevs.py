#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

# Import code for DEVS model representation:
from pypdevs.DEVS import CoupledDEVS

# Import all models to couple
from multirobot_ebdevs.atomics.integrators.qss1tools import (
    evaluate_poly_q, pad_zeros_q
)
from multirobot_ebdevs.atomics.communication.transmission_medium import (
    TransmissionMedium
)
from multirobot_ebdevs.atomics.misc.logger import Logger

# our coupled models
from multirobot_ebdevs.coupled.target_tracking.hunter import Hunter
from multirobot_ebdevs.utils.files import append_csv_file


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

        self.hunters_ids = [
            idx for idx in robots_config.keys() if idx.startswith('Hunter')
        ]
        self.targets_ids = [
            idx for idx in robots_config.keys() if idx.startswith('Target')
        ]

        self.router = self.addSubModel(TransmissionMedium(
            robots_ids=self.hunters_ids,
            name='TransmissionMedium',
            debug=self.debug
        ))
        self.logger = self.addSubModel(Logger(
            period=log_period,
            name='Logger',
            log_path=self.log_path,
            debug=self.debug
        ))

        self.robots_states = {}
        self.hunters = {}
        for hunter_id in self.hunters_ids:
            self.hunters[hunter_id] = self.addSubModel(Hunter(
                world_config[hunter_id],
                simu_config[hunter_id],
                robots_config[hunter_id],
                name=hunter_id,
                log_path=self.log_path,
                debug=self.debug
            ))
            self.connectPorts(
                self.hunters[hunter_id].outPorts['radio'],
                self.router.inPorts[hunter_id]
            )
            self.connectPorts(
                self.router.outPorts[hunter_id],
                self.hunters[hunter_id].inPorts['radio']
            )

        for target_id in self.targets_ids:
            # targets_states must be initialized at the very beginning
            self.robots_states[target_id] = {
                'time': 0.0,
                'pose': [
                    pad_zeros_q(coord) for coord
                    in world_config[target_id]['position']
                ],
                'collect_range': world_config[target_id]['collect_range'],
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

        self.robots_states[micro_id] = data.copy()

        for target_id in self.targets_ids:
            distance = np.sqrt(np.sum(np.square(np.subtract(
                self.robots_states[micro_id]['pose'],
                self.robots_states[target_id]['pose']
            ))))
            if distance < self.robots_states[target_id]['collect_range']:
                self.robots_states[target_id]['status'] = 'passive'

        # log new value of micro_states
        micro_pos = [data['pose'][0][0], data['pose'][1][0]]
        log = [micro_id, data['time'], micro_pos, data['comm_range']]

        if micro_id.startswith('Hunter'):
            log += [
                neighbor_id
                for neighbor_id in self.robots_states.keys()
                if self.in_range(micro_id, micro_pos, neighbor_id, 0.0)
                # checks and registers current neighboring hunters
            ]
        elif micro_id.startswith('Target'):
            log += [data['status']]

        append_csv_file(self.log_path + 'global.csv', log)

        if (self.debug):
            print(
                "t: {} s, Coupled name: {}, Global Transition Function,\
                x_b_micro: {}, global state: {}"
                .format(data['time'], self.name, x_b_micro, self.robots_states)
                )

    def getGlobalState(self, current_time):
        robot_data = []
        target_data = []
        for robot, state in self.robots_states.items():
            position = self.getRobotPosition(robot, current_time)
            if robot.startswith('Hunter'):
                robot_data += [coord[0] for coord in position]
            if robot.startswith('Target'):
                target_data += [coord[0] for coord in position]
                target_data += [1.0 if state['status'] == 'active' else 0.0]

        return robot_data, target_data

    def getRobotPosition(self, robot_id, current_time):
        state = self.robots_states[robot_id]
        delta_time = current_time - state['time']
        return [[evaluate_poly_q(poly, delta_time)] for poly in state['pose']]

    def getNeighbors(self, robot_1_id, current_time):
        # need to know the current time to make the polynomial advance in time
        robot_1_pos = self.getRobotPosition(robot_1_id, current_time)

        return [
            robot_2_id
            for robot_2_id in self.hunters_ids
            if robot_1_id != robot_2_id and
            self.in_range(robot_1_id, robot_1_pos, robot_2_id, current_time)
        ]

    def in_range(self, robot_1_id, robot_1_pos, robot_2_id, current_time):
        robot_2_pos = self.getRobotPosition(robot_2_id, current_time)
        distance = np.sqrt(np.sum(np.square(np.subtract(
            robot_1_pos, robot_2_pos
        ))))
        trasmitter_range = self.robots_states[robot_1_id]['comm_range']

        return distance < trasmitter_range

    def getNearestTarget(self, robot_id, current_time):
        robot_pos = self.getRobotPosition(robot_id, current_time)
        try:
            target_pos = min([
                (
                    np.sum(np.square(np.subtract(
                        robot_pos, self.robots_states[target_id]['pose']
                    ))),
                    self.robots_states[target_id]['pose']
                )
                for target_id in self.targets_ids
                if self.robots_states[target_id]['status'] == 'active'
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
