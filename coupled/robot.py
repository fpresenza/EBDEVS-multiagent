import numpy as np

from pypdevs.DEVS import CoupledDEVS

from atomics.controller import Controller
from atomics.token_handler import TokenHandler
from atomics.kalman_filter import KalmanFilter
from coupled.robot_dynamics import RobotDynamics
# from atomics.speedsensor import SpeedSensor
from atomics.stochastic_systems import ZeroOrderLinearSystem
from atomics.gpssensor import PositioningSystem


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
            debug=self.debug
        )
        token_handler = TokenHandler(
            robot_id=self.name,
            config=config['token_handler'],
            debug=self.debug,
        )
        kalman_filter = KalmanFilter(
            robot_id=self.name,
            config=config['kalman_filter'],
            logpath=logpath,
            debug=self.debug
        )
        speed_sensor = ZeroOrderLinearSystem(
            input_matrix=np.eye(2, dtype=float),
            noise_mean=np.zeros((2, 1), dtype=float),
            noise_covariance=np.array([[0.15, 0.0], [0.0, 0.15]]),
            debug=self.debug
        )

        self.dynamics = self.addSubModel(dynamics)
        self.controller = self.addSubModel(controller)
        self.token_handler = self.addSubModel(token_handler)
        self.kalman_filter = self.addSubModel(kalman_filter)
        self.speed_sensor = self.addSubModel(speed_sensor)

        # Declare the coupled model's output ports:
        # self.IN_vx_vy = self.addInPort(name="robot_vx_vy")
        self.OUT_router_token = self.addOutPort(name="out_router")
        self.IN_router_token  = self.addInPort(name="in_router")

    
        self.connectPorts(self.IN_router_token, self.token_handler.in_router_token)

        self.connectPorts(self.token_handler.out_router_token, self.OUT_router_token) 
        self.connectPorts(self.token_handler.out_kalman_extpos, self.kalman_filter.in_handler_extpos)
        self.connectPorts(self.token_handler.out_controller_extpos, self.controller.in_handler_extpos)
        self.connectPorts(self.token_handler.out_controller_extact, self.controller.in_handler_extact)

        self.connectPorts(self.kalman_filter.out_handler_intpos, self.token_handler.in_kalman_intpos)
        self.connectPorts(self.kalman_filter.out_control_intpos, self.controller.in_kalman_intpos)
        
        self.connectPorts(self.controller.out_handler_intact, self.token_handler.in_controller_intact)        
        self.connectPorts(self.controller.out_dynamics_intact, self.dynamics.IN_control_input)
        self.connectPorts(self.controller.out_dynamics_intact, self.speed_sensor.input)
        
        self.connectPorts(self.speed_sensor.output, self.kalman_filter.in_dynamics_velmeas)

        if config['enable_GPS']:
            gps_sensor = PositioningSystem(
                config={
                    'noise_mean': np.zeros((2, 1), dtype=float),
                    'noise_covariance':np.array([[25.0, 0.0], [0.0, 25.0]]),
                    'period': 1.0
                },
                debug=self.debug
            )
            self.gps_sensor = self.addSubModel(gps_sensor)
            self.connectPorts(self.dynamics.OUT_position, self.gps_sensor.input)
            self.connectPorts(self.gps_sensor.output, self.kalman_filter.in_gps_posmeas)



        if (self.debug):
            print("t: 0 s, Coupled name: {}, Init Function".format(self.name))

    def globalTransition(self, e_g, x_b_micro, *args, **kwargs):
        # self.current_time += e_g

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
