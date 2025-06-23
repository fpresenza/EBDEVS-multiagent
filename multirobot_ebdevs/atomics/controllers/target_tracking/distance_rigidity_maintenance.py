#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

from uvnpy.distances.control import RigidityMaintenance
from uvnpy.control.core import CollisionAvoidanceVanishing
from uvnpy.control.targets import TargetTracking

from multirobot_ebdevs.atomics.controllers.controller import Controller


class DRMControl(object):
    def __init__(self, robot_id, config):
        self.robot_id = robot_id
        self.dim = config['dim']
        self.weights = config['weights']
        self.obstacles = []
        self.subframework = {}
        self.external_action = np.zeros((self.dim, 1), dtype=float)
        self.tracking = TargetTracking(**config['tracking'])
        self.collision = CollisionAvoidanceVanishing(**config['collision'])
        self.rigidity = RigidityMaintenance(**config['rigidity'])

    def clear(self):
        self.subframework.clear()
        self.obstacles.clear()
        self.external_action[:] = 0.0

    def compute_action(self, target_position):
        coordination_data = {}

        if self.robot_id in self.subframework:
            position = self.subframework[self.robot_id]

            # target collection
            if target_position is not None:
                target_action = self.tracking.update(
                    position, np.ravel(target_position)
                ).reshape(-1, 1)
                target_action *= self.weights['tracking']
            else:
                target_action = np.zeros((self.dim, 1), dtype=float)

            # obstacle avoidance
            if len(self.obstacles) > 0:
                collision_action = self.collision.update(
                    position, self.obstacles
                ).reshape(-1, 1)
                collision_action *= self.weights['collision']
            else:
                collision_action = np.zeros((self.dim, 1), dtype=float)

            # rigidity maintenance
            rigidity_action = self.external_action

            if len(self.subframework) > 1:
                subframework_ids, subframework_positions = list(zip(
                    *self.subframework.items()
                ))
                subframework_actions = self.rigidity.update(np.array(
                    subframework_positions
                ))
                subframework_actions = {
                    node_id: action.reshape(-1, 1)
                    for node_id, action
                    in zip(subframework_ids, subframework_actions)
                }
                rigidity_action += subframework_actions.pop(
                    self.robot_id
                )
                rigidity_action *= self.weights['rigidity']
                coordination_data = subframework_actions

            # compose control action
            control_action = target_action + collision_action + rigidity_action
        else:
            control_action = np.zeros((self.dim, 1), dtype=float)

        return control_action, coordination_data


class DistanceRigidityMaintenance(Controller):
    def set_control(self, robot_id, config):
        #
        #    define controller here
        #
        return DRMControl(robot_id, config)

    def set_in_port_names(self):
        #
        #    define the list of input ports name here
        #
        return [
            'position', 'other_position', 'external_action'
        ]

    def process_inputs(self, sigma, current_time, control, port_name, data):
        #
        #    process inputs here
        #
        if port_name == 'position':
            control.subframework[self.robot_id] = data.ravel()

        elif port_name == 'other_position':
            node_id, other_position, hops = data
            control.subframework[node_id] = other_position.ravel()
            if hops == 1:
                control.obstacles.append(other_position.ravel())

        elif port_name == 'external_action':
            control.external_action += data[1]

        return control

    def compute_action(self):
        #
        #    compute control action here`
        #
        sigma, current_time, control = self.state.get()

        target_position = self.parent.parent.getNearestTarget(
            self.robot_id, current_time + sigma
        )
        control_action, coordination_data = control.compute_action(
            target_position
        )

        return control_action, coordination_data, None
