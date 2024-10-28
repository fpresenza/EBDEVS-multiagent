import numpy as np

from pypdevs.DEVS import CoupledDEVS

from atomics.controller import Controller
from atomics.token_handler import TokenHandler
from atomics.kalman_filter import KalmanFilter
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
                'Time': 0.0, 
                'Pose': [[p] + [0.0] * 9 for p in position], 
                'CommRange': comm_range
            }
        ]
        self.current_time = 0

        dynamics = RobotDynamics(
            position=position,
            config=config['dynamics'],
            enable_GPS=config['enable_GPS'],
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

        self.dynamics      = self.addSubModel(dynamics)
        self.controller    = self.addSubModel(controller)
        self.token_handler = self.addSubModel(token_handler)
        self.kalman_filter = self.addSubModel(kalman_filter)

        # Declare the coupled model's output ports:
        # self.IN_vx_vy = self.addInPort(name="robot_vx_vy")
        self.OUT_x    = self.addOutPort(name="robot_x")
        self.OUT_y    = self.addOutPort(name="robot_y")
        self.OUT_router_token = self.addOutPort(name="out_router")
        self.IN_router_token  = self.addInPort(name="in_router")

        # Connect coupled model's ports with atomic models' ports
        self.connectPorts(self.dynamics.OUT_dynamics_x, self.OUT_x)
        self.connectPorts(self.dynamics.OUT_dynamics_y, self.OUT_y)
        # self.connectPorts(self.IN_vx_vy, self.splitter_gen.in_splitter_msgs)
        # self.connectPorts(self.splitter_gen.out_splitter_in, self.dynamics.IN_dynamics_vx_vy)
        
        self.connectPorts(self.controller.out_dynamics_intact, self.dynamics.IN_dynamics_vx_vy)
        self.connectPorts(self.IN_router_token, self.token_handler.in_router_token)
        self.connectPorts(self.token_handler.out_router_token, self.OUT_router_token) 
        self.connectPorts(self.controller.out_handler_intact, self.token_handler.in_controller_intact)

        self.connectPorts(self.kalman_filter.out_control_intpos, self.controller.in_kalman_intpos)
        self.connectPorts(self.kalman_filter.out_handler_intpos, self.token_handler.in_kalman_intpos)
        # self.connectPorts(self.controller.out_kalman_intact, self.kalman_filter.in_control_intact)
        self.connectPorts(self.token_handler.out_kalman_extpos, self.kalman_filter.in_handler_extpos) 
        self.connectPorts(self.dynamics.OUT_measured_v, self.kalman_filter.in_control_intact)

        if (self.debug):
            print("t: 0 s, Coupled name: {}, Init Function".format(self.name))

    def globalTransition(self, e_g, x_b_micro, *args, **kwargs):
        # self.current_time += e_g

        micro_id, data = x_b_micro
        try:
            self.y_up[1]['Time'] = data['t'].copy()
            self.y_up[1]['Pose'][0] = data['x'].copy()
            self.y_up[1]['Pose'][1] = data['y'].copy()
            current_time = data['t'].copy()
        except AttributeError:
            self.y_up[1]['Time'] = data['t']
            self.y_up[1]['Pose'][0] = data['x']
            self.y_up[1]['Pose'][1] = data['y']
            current_time = data['t']

        if (self.debug):
            print(
                "t: {:.2f} s, Coupled name: {}, Global Transition Function, x_b_micro: {}"
                .format(current_time, self.name, x_b_micro)
            )

    def select(self, immChildren):
        """
        Choose a model to transition from all possible models.
        """
        # Doesn't really matter, as they don't influence each other
        return immChildren[0]
