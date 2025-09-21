#!/usr/bin/env python
# -*- coding: utf-8 -*-

from pypdevs.DEVS import CoupledDEVS

from multirobot_ebdevs.atomics.integrators.qss1tools import (
    evaluate_poly, pad_zeros
)
from multirobot_ebdevs.atomics.controllers.target_tracking\
    .distance_rigidity_maintenance import DistanceRigidityMaintenance
from multirobot_ebdevs.atomics.communication. communication_module import (
    CommunicationModule
)
from multirobot_ebdevs.atomics.coordination\
    .target_tracking.hunter_coordinator import HunterCoordinator
from multirobot_ebdevs.atomics.localization.distance_kalman_filter import (
    DistanceBasedKalmanFilter
)
from multirobot_ebdevs.atomics.sensors.gpssensor import GPSSensor
from multirobot_ebdevs.coupled.robot_dynamics.kinematic_unicyle import (
    RobotDynamics
)


class Hunter(CoupledDEVS):
    def __init__(
            self,
            world_config,
            simu_config,
            hunter_config,
            name='Hunter',
            log_path='./',
            debug=False):
        """
        A robot model composed of the robot's pysics.
        """
        # Always call parent class' constructor FIRST:
        CoupledDEVS.__init__(self, name)

        # Parameters
        position = world_config['position']
        orientation = world_config['orientation']
        comm_range = world_config['comm_range']
        self.debug = debug

        pose = [pad_zeros(coord) for coord in position]
        pose.append(pad_zeros([orientation]))

        self.y_up = [
            self.name,
            {
                'time': [0.0, 0.0, 0.0],
                'pose': pose,
                'comm_range': comm_range
            }
        ]
        self.current_time = 0

        dynamics = RobotDynamics(
            orientation=orientation,
            position=position,
            config=simu_config['qss'],
            debug=self.debug
        )
        controller = DistanceRigidityMaintenance(
            robot_id=self.name,
            config=hunter_config['controller'],
            log_path=log_path,
            debug=self.debug
        )
        communication_module = CommunicationModule(
            robot_id=self.name,
            config=hunter_config['communication'],
            params=world_config['range_sensor'],
            debug=self.debug,
        )
        coordinator = HunterCoordinator(
            robot_id=self.name,
            config=hunter_config['coordinator'],
            debug=self.debug,
        )
        localization = DistanceBasedKalmanFilter(
            robot_id=self.name,
            config=hunter_config['localization'],
            log_path=log_path,
            debug=self.debug
        )

        self.dynamics = self.addSubModel(dynamics)
        self.controller = self.addSubModel(controller)
        self.communication_module = self.addSubModel(communication_module)
        self.coordinator = self.addSubModel(coordinator)
        self.localization = self.addSubModel(localization)

        # Declare the coupled model's output ports:
        self.outPorts = {'radio': self.addOutPort(name="out_radio")}
        self.inPorts = {'radio': self.addInPort(name="in_radio")}

        self.connectPorts(
            self.inPorts['radio'],
            self.communication_module.inPorts['radio']
        )
        self.connectPorts(
            self.communication_module.outPorts['radio'],
            self.outPorts['radio']
        )

        self.connectPorts(
            self.communication_module.outPorts['token'],
            self.coordinator.inPorts['token']
        )
        self.connectPorts(
            self.coordinator.outPorts['token'],
            self.communication_module.inPorts['token']
        )

        self.connectPorts(
            self.coordinator.outPorts['neighbors_positions'],
            self.localization.inPorts['neighbors_positions']
        )
        self.connectPorts(
            self.localization.outPorts['estimation'],
            self.coordinator.inPorts['position']
        )

        self.connectPorts(
            self.coordinator.outPorts['other_position'],
            self.controller.inPorts['other_position']
        )
        self.connectPorts(
            self.coordinator.outPorts['external_action'],
            self.controller.inPorts['external_action']
        )
        self.connectPorts(
            self.controller.outPorts['coordination_data'],
            self.coordinator.inPorts['others_actions']
        )

        self.connectPorts(
            self.localization.outPorts['estimation'],
            self.controller.inPorts['position']
        )

        self.connectPorts(
            self.controller.outPorts['action'],
            self.dynamics.inPorts['control_input']
        )

        self.connectPorts(
            self.controller.outPorts['action'],
            self.localization.inPorts['velocity_measurement']
        )

        if hunter_config['gps_sensor'] is True:
            gps_sensor = GPSSensor(
                params=world_config['gps_sensor'],
                debug=self.debug
            )
            self.gps_sensor = self.addSubModel(gps_sensor)
            self.connectPorts(
                self.gps_sensor.outPorts['measurement'],
                self.localization.inPorts['position_measurement']
            )

        if (self.debug):
            print("t: 0 s, Coupled name: {}, Init Function".format(self.name))

    def globalTransition(self, e_g, x_b_micro, *args, **kwargs):
        # self.current_time += e_g

        if len(x_b_micro) == 1:
            micro_id, data = x_b_micro[0]
        else:
            micro_id, data = x_b_micro

        self.y_up[1]['time'] = data['time']
        self.y_up[1]['pose'] = data['pose'].copy()

        if (self.debug):
            print(
                "t: {:.2f} s, Coupled name: {}, \
                Global Transition Function, x_b_micro: {}"
                .format(data['time'], self.name, x_b_micro)
            )

    def getRobotPosition(self, current_time):
        state = self.y_up[1]
        return [
            [evaluate_poly(poly, current_time - t)]
            for t, poly in zip(state['time'], state['pose'])
        ]

    def select(self, immChildren):
        """
        Choose a model to transition from all possible models.
        """
        # Doesn't really matter, as they don't influence each other
        return immChildren[0]
