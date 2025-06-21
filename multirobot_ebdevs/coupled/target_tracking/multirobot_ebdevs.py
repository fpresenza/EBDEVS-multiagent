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
from multirobot_ebdevs.coupled.target_tracking.target import Target
from multirobot_ebdevs.utils.files import (
    read_json_file,
    append_csv_file
)


class MultiRobotSystem(CoupledDEVS):
    def __init__(
            self,
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

        world_config = read_json_file('world.json')
        simu_config = read_json_file('simu.json')
        robots_config = read_json_file('robots.json')

        hunters_ids = [
            idx for idx in robots_config.keys() if idx.startswith('Hunter')
        ]
        targets_ids = [
            idx for idx in robots_config.keys() if idx.startswith('Target')
        ]

        self.router = self.addSubModel(TransmissionMedium(
            robots_ids=hunters_ids + targets_ids,
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
        for hunter_id in hunters_ids:
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

        self.targets = {}
        for target_id in targets_ids:
            self.targets[target_id] = self.addSubModel(Target(
                world_config[target_id],
                simu_config[target_id],
                robots_config[target_id],
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
            self.robots_states[target_id] = {
                'time': 0.0,
                'pose': [
                    pad_zeros_q(coord) for coord
                    in world_config[target_id]["position"]
                ],
                'comm_range': world_config[target_id]["comm_range"],
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

        # log new value of micro_states
        log = [micro_id, data['time']]
        log += [data['pose'][0][0], data['pose'][1][0]]
        log += [data['comm_range']]

        if micro_id.startswith('Hunter'):
            log += [
                neighbor_id
                for neighbor_id in self.robots_states.keys()
                if self.in_range(micro_id, neighbor_id, 0.0)
                # checks and registers current neighboring hunters
            ]
        elif micro_id.startswith('Target'):
            log += [data['status']]

        append_csv_file(self.logpath + 'global.csv', log)

        if (self.debug):
            print(
                "t: {} s, Coupled name: {}, Global Transition Function,\
                x_b_micro: {}, global state: {}"
                .format(data['time'], self.name, x_b_micro, self.robots_states)
                )

    def getGlobalState(self, current_time):
        ids = []
        positions = []
        comm_ranges = []
        status = []
        for robot, state in self.robots_states.items():
            ids.append(robot)
            comm_ranges.append(state['comm_range'])
            try:
                status.append(state['status'])
            except KeyError:
                status.append('')
            previous_time = state['time']
            delta_time = current_time - previous_time
            position_poly = state['pose']
            x = evaluate_poly_q(position_poly[0], delta_time)
            y = evaluate_poly_q(position_poly[1], delta_time)
            positions += [x, y]
        return ids, positions, comm_ranges, status

    def getRobotPosition(self, robot_id, current_time):
        state = self.robots_states[robot_id]
        delta_time = current_time - state['time']
        return [evaluate_poly_q(poly, delta_time) for poly in state['pose']]

    def getNeighbors(self, robot_1_id, current_time):
        # need to know the current time to make the polynomial advance in time
        robot_1_pos = self.getRobotPosition(robot_1_id, current_time)

        return [
            robot_2_id
            for robot_2_id in self.robots_states.keys()
            if self.in_range(robot_1_id, robot_1_pos, robot_2_id, current_time)
        ]

    def in_range(self, robot_1_id, robot_1_pos, robot_2_id, current_time):
        # robots might be hunter or target
        # tweak to improve performance since target-target comm is not needed
        if robot_1_id.startswith('Target') and robot_2_id.startswith('Target'):
            return False

        robot_2_pos = self.getRobotPosition(robot_2_id, current_time)
        distance = np.sqrt(np.sum(np.square(np.subtract(
            robot_1_pos, robot_2_pos
        ))))
        trasmitter_range = self.robots_states[robot_1_id]['comm_range']

        return distance < trasmitter_range

    def select(self, immChildren):
        """
        Choose a model to transition from all possible models.
        """
        # Doesn't really matter, as they don't influence each other
        return immChildren[0]
