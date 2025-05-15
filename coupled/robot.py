import numpy as np

from pypdevs.DEVS import CoupledDEVS

from atomics.controllers.distance_rigidity_maintenance import Controller
from atomics.communication.radio_module import RadioModule
from atomics.coordination.token_handlers import RobotCoordinator
from atomics.estimation.distance_kalman_filter import StateEstimator
from atomics.sensors.speedsensor import SpeedSensor
from atomics.sensors.gpssensor import PositioningSystem
from coupled.robot_dynamics import RobotDynamics


class Robot(CoupledDEVS):
    def __init__(
            self, 
            config,
            name='Robot',
            logpath='./',
            debug=False):
        """
        A robot model composed of the robot's pysics.
        """
        # Always call parent class' constructor FIRST:
        CoupledDEVS.__init__(self, name)

        # Parameters
        position = config['position']
        comm_range = config['comm_range']
        self.debug = debug

        self.y_up = [
            self.name, 
            {
                'time': 0.0, 
                'pose': [coord + [0.0] * 9 for coord in position], 
                'comm_range': comm_range
            }
        ]
        self.current_time = 0

        dynamics = RobotDynamics(
            position=position,
            config=config['dynamics'],
            debug=self.debug
        )
        controller = Controller(
            robot_id=self.name,
            config=config['controller'],
            logpath=logpath,
            debug=self.debug
        )
        radio_module = RadioModule(
            robot_id=self.name,
            debug=self.debug,
        )
        coordinator = RobotCoordinator(
            robot_id=self.name,
            config=config['coordinator'],
            debug=self.debug,
        )
        state_estimator = StateEstimator(
            robot_id=self.name,
            config=config['state_estimator'],
            logpath=logpath,
            debug=self.debug
        )
        speed_sensor = SpeedSensor(
            config=config['speed_sensor'],
            debug=self.debug
        )

        self.dynamics = self.addSubModel(dynamics)
        self.controller = self.addSubModel(controller)
        self.radio_module = self.addSubModel(radio_module)
        self.coordinator = self.addSubModel(coordinator)
        self.state_estimator = self.addSubModel(state_estimator)
        self.speed_sensor = self.addSubModel(speed_sensor)

        # Declare the coupled model's output ports:
        self.outPorts = {'radio': self.addOutPort(name="out_radio")}
        self.inPorts  = {'radio': self.addInPort(name="in_radio")}

        self.connectPorts(self.inPorts['radio'], self.radio_module.inPorts['radio'])
        self.connectPorts(self.radio_module.outPorts['radio'], self.outPorts['radio'])

        self.connectPorts(self.radio_module.outPorts['token'], self.coordinator.inPorts['token'])
        self.connectPorts(self.coordinator.outPorts['token'], self.radio_module.inPorts['token'])
        
        self.connectPorts(self.coordinator.outPorts['neighbors_positions'], self.state_estimator.inPorts['neighbors_positions'])
        self.connectPorts(self.state_estimator.outPorts['position'], self.coordinator.inPorts['position'])

        self.connectPorts(self.coordinator.outPorts['other_position'], self.controller.inPorts['other_position'])
        self.connectPorts(self.coordinator.outPorts['external_action'], self.controller.inPorts['external_action'])
        self.connectPorts(self.coordinator.outPorts['target_position'], self.controller.inPorts['target_position'])
        self.connectPorts(self.controller.outPorts['others_actions'], self.coordinator.inPorts['others_actions'])

        self.connectPorts(self.state_estimator.outPorts['position'], self.controller.inPorts['position'])
        
        self.connectPorts(self.controller.outPorts['own_action'], self.dynamics.inPorts['control_input'])

        self.connectPorts(self.dynamics.outPorts['position_polynomial'], self.speed_sensor.inPorts['position_polynomial'])
        
        self.connectPorts(self.speed_sensor.outPorts['velocity_measurement'], self.state_estimator.inPorts['velocity_measurement'])

        if config['gps_sensor']['enabled']:
            gps_sensor = PositioningSystem(
                config=config['gps_sensor'],
                debug=self.debug
            )
            self.gps_sensor = self.addSubModel(gps_sensor)
            self.connectPorts(self.dynamics.outPorts['position_polynomial'], self.gps_sensor.inPorts['position_polynomial'])
            self.connectPorts(self.gps_sensor.outPorts['position_measurement'], self.state_estimator.inPorts['position_measurement'])

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
                "t: {:.2f} s, Coupled name: {}, Global Transition Function, x_b_micro: {}"
                .format(data['time'], self.name, x_b_micro)
            )

    def select(self, immChildren):
        """
        Choose a model to transition from all possible models.
        """
        # Doesn't really matter, as they don't influence each other
        return immChildren[0]
