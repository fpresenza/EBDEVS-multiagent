from pypdevs.DEVS import CoupledDEVS

from atomics.integrators.qss1tools import pad_zeros
from atomics.controllers.distance_rigidity_maintenance import \
    DistanceRigidityMaintenance
from atomics.communication.communication_module import CommunicationModule
from atomics.coordination.token_handlers import RobotCoordinator
from atomics.localization.distance_kalman_filter import DistanceKalmanFilter
from atomics.sensors.speedsensor import SpeedSensor
from atomics.sensors.gpssensor import GPSSensor
from coupled.robot_dynamics import RobotDynamics


class Robot(CoupledDEVS):
    def __init__(
            self,
            world_config,
            simu_config,
            robot_config,
            name='Robot',
            logpath='./',
            debug=False):
        """
        A robot model composed of the robot's pysics.
        """
        # Always call parent class' constructor FIRST:
        CoupledDEVS.__init__(self, name)

        # Parameters
        position = world_config['position']
        comm_range = world_config['comm_range']
        self.debug = debug

        self.y_up = [
            self.name,
            {
                'time': 0.0,
                'pose': [pad_zeros(coord) for coord in position],
                'comm_range': comm_range
            }
        ]
        self.current_time = 0

        dynamics = RobotDynamics(
            position=position,
            config=simu_config['qss'],
            debug=self.debug
        )
        controller = DistanceRigidityMaintenance(
            robot_id=self.name,
            config=robot_config['controller'],
            logpath=logpath,
            debug=self.debug
        )
        communication_module = CommunicationModule(
            robot_id=self.name,
            batch=True,
            debug=self.debug,
        )
        coordinator = RobotCoordinator(
            robot_id=self.name,
            config=robot_config['coordinator'],
            debug=self.debug,
        )
        localization = DistanceKalmanFilter(
            robot_id=self.name,
            config=robot_config['localization'],
            logpath=logpath,
            debug=self.debug
        )
        speed_sensor = SpeedSensor(
            config=world_config['speed_sensor'],
            debug=self.debug
        )

        self.dynamics = self.addSubModel(dynamics)
        self.controller = self.addSubModel(controller)
        self.communication_module = self.addSubModel(communication_module)
        self.coordinator = self.addSubModel(coordinator)
        self.localization = self.addSubModel(localization)
        self.speed_sensor = self.addSubModel(speed_sensor)

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
            self.coordinator.outPorts['target_position'],
            self.controller.inPorts['target_position']
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
            self.dynamics.outPorts['position_polynomial'],
            self.speed_sensor.inPorts['internal_state']
        )

        self.connectPorts(
            self.speed_sensor.outPorts['measurement'],
            self.localization.inPorts['velocity_measurement']
        )

        if robot_config['gps_sensor'] is True:
            gps_sensor = GPSSensor(
                config=world_config['gps_sensor'],
                debug=self.debug
            )
            self.gps_sensor = self.addSubModel(gps_sensor)
            self.connectPorts(
                self.dynamics.outPorts['position_polynomial'],
                self.gps_sensor.inPorts['external_state']
            )
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

    def select(self, immChildren):
        """
        Choose a model to transition from all possible models.
        """
        # Doesn't really matter, as they don't influence each other
        return immChildren[0]
