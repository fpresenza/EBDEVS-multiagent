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

# Import code for DEVS model representation:
from pypdevs.DEVS import *
from pypdevs.infinity import INFINITY

# Import all models to couple
from atomics.qssintegrators import *
from atomics.misc import *
from atomics.router import *
from atomics.controller import Controller
from atomics.token_handler import TokenHandler
from atomics.kalman_filter import KalmanFilter
from atomics.speedsensor import SpeedSensor

from utils import (
    read_json_file,
    write_json_file,
    append_csv_file
)

import sys

#-----------------------------------
# QSS_Integrator with Y_up: QSS integrator with micro-macro state communication with its parent.
# This class derives from QSSIntegrator => it's only necessary to reimplement the input and output transition functions.
#-----------------------------------
class QSSIntegrator_Yup(QSSIntegrator):
    """
    QSS1 integrator atomic model
    """
    def __init__(self, name=None, dQMin=1e-6, dQRel=1e-3, gain=1, x0=0, debug=False):
        """
        Constructor (parameterizable).
        """
        # Always call parent class' constructor FIRST:
        QSSIntegrator.__init__(self, 
                               name=name, 
                               dQMin=dQMin, 
                               dQRel=dQRel, 
                               gain=gain,
                               x0 = x0, 
                               debug = debug
                               )
        self.y_up  = [self.name, 0.0, None]

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    def intTransition(self): # re-implement this function
        """
        Internal Transition Function.
        """
        q, xprev, sigma, current_time = self.state.get()

        current_time += sigma
        # x = advance_time(xprev,sigma,1) # p: x, dt: sigma, order: 1
        x = [xprev[0] + sigma * xprev[1], xprev[1]]
        q = x[0]

        self.dQ = max(self.dQRel * abs(x[0]), self.dQMin)

        if (x[1]==0):
            sigma = INFINITY
        else:
            sigma = abs(self.dQ/x[1])

        if (sigma<0):
            raise DEVSException(\
                  "invalid state sigma <%f> in internal transition function"\
                  % sigma)

        if (self.debug):
            print("t: {:.2f} s, Atomic name: {}, Internal Transition Function, xprev: {}, x: {}, q: {}, sigma: {}".format(current_time,self.name,xprev,x,q,sigma))

        # shares information to the parent to compute the Global Transition function
        try:
            self.y_up[2] = q.copy()
            self.y_up[1] = current_time.copy()
        except AttributeError:
            self.y_up[2] = q
            self.y_up[1] = current_time

        return QSSState(q,x,sigma,current_time)


    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        # Received a new event, so start processing it
        derx     = inputs[self.IN_dx][0]
        derx_val = derx * self.gain # [dx[i] * self.gain for i in range(len(dx))]

        # if (x.port==0) {
        q, x, sigma, current_time = self.state.get()
        current_time += self.elapsed

        if self.IN_dx in inputs:
            # update polynomial x
            x[0] =  x[0] + x[1] * self.elapsed
            x[1] = derx_val # dx[0]

            diffxq = [0 for i in range(len(x))]

            if (sigma>0):
                # inferior delta crossing
                # diffxq = q - x - dQ = {q[0] - x[0] - dQ, -x[1]} 
                diffxq[1] = -x[1]
                diffxq[0] =  q - x[0] - self.dQ
                sigma     = minposroot(diffxq, 1) # coeff: diffxq, order: 1
                sigma_lo  = sigma

                # superior delta difference
                # diffxq = q - x + dQ = {q[0] - x[0] + dQ, -x[1]} 
                diffxq[0] =  q - x[0] + self.dQ
                sigma_up  = minposroot(diffxq, 1) # coeff: diffxq, order: 1

                # keep the smallest one
                if (sigma_up < sigma):
                    sigma = sigma_up

                if (abs(x[0] - q) > self.dQ):
                    sigma = 0

                if (self.debug):
                    print("t: {:.2f} s, Atomic name: {}, External Transition Function, dx: {}, x: {}, sigma: {}, sigma_lo: {}, sigma_up: {}"\
                            .format(current_time,self.name,derx_val,x,sigma,sigma_lo,sigma_up))

        else:
            x[0] = derx_val
            sigma = 0

        # shares information to the parent to compute the Global Transition function
        try:
            self.y_up[2] = q.copy()
            self.y_up[1] = current_time.copy()
        except AttributeError:
            self.y_up[2] = q
            self.y_up[1] = current_time

        return QSSState(q,x,sigma,current_time)

#----------------------------
# Dynamics for Robot i
#----------------------------
class RobotDynamics(CoupledDEVS):
    def __init__(self, name='RobotDynamics', dQMin=1e-6, dQRel=1e-3, x0=0.0, y0=0.0, gainx=1, gainy=1, debug=False):
        """
        Robot's dynamic model composed of two integrators for x and y and a splitter.
        """
        # Always call parent class' constructor FIRST:
        CoupledDEVS.__init__(self, name)

        # Parameters
        self.dQMin = dQMin
        self.dQRel = dQRel
        self.x0    = x0
        self.y0    = y0
        self.gainx = gainx
        self.gainy = gainy
        self.debug = debug

        # dictionary to save childrens' states
        self.y_up = [self.name, {'t': 0.0, 'x': x0, 'y': y0}]
        self.current_time = 0

        # Declare childrens: splitterx2, QSS integ x 2
        splitter     = Splitter(name="splitter",
                                numoutputs=2,
                                debug=self.debug
                                )
        integrator_x = QSSIntegrator_Yup(name="x", 
                                         dQMin=self.dQMin, 
                                         dQRel=self.dQRel, 
                                         gain=self.gainx, 
                                         x0=self.x0, 
                                         debug=self.debug
                                         )
        integrator_y = QSSIntegrator_Yup(name="y",
                                         dQMin=self.dQMin, 
                                         dQRel=self.dQRel, 
                                         gain=self.gainy, 
                                         x0=self.y0, 
                                         debug=self.debug
                                         )
        speed_sensor = SpeedSensor(name="vmeas",
                                   noisestd=0.0,
                                   bias=np.ones((2,1)),
                                   trans=np.eye(2),
                                   debug=True
                                   )
        self.splitter     = self.addSubModel(splitter)
        self.integrator_x = self.addSubModel(integrator_x)
        self.integrator_y = self.addSubModel(integrator_y)
        self.speed_sensor = self.addSubModel(speed_sensor)

        # Declare the coupled model's output ports:
        self.OUT_dynamics_x    = self.addOutPort(name="OUT_dynamics_x")
        self.OUT_dynamics_y    = self.addOutPort(name="OUT_dynamics_y")
        self.OUT_measured_v    = self.addOutPort(name="OUT_measured_v")
        self.IN_dynamics_vx_vy = self.addInPort( name="IN_dynamics_vx_vy")

        # Connect coupled model's ports with atomic models' ports
        self.connectPorts(self.IN_dynamics_vx_vy, self.splitter.in_splitter_msgs)
        self.connectPorts(self.splitter.out_splitter_msgs[0], self.integrator_x.IN_dx)
        self.connectPorts(self.splitter.out_splitter_msgs[1], self.integrator_y.IN_dx)
        self.connectPorts(self.integrator_x.OUT_q, self.OUT_dynamics_x)
        self.connectPorts(self.integrator_y.OUT_q, self.OUT_dynamics_y)
        self.connectPorts(self.IN_dynamics_vx_vy, self.speed_sensor.in_commanded_speed)
        self.connectPorts(self.speed_sensor.out_measured_speed, self.OUT_measured_v)

        if (self.debug):
            print("t: 0 s, Coupled name: {}, Init Function".format(self.name))

    def globalTransition(self, e_g, x_b_micro, *args, **kwargs):
        # update each coordinate separatedly, since the events in the integrators for the same robot do not need to be simultaneous
        # self.current_time += e_g

        micro_id, children_time, data = x_b_micro[0]
        try:
            self.y_up[1][micro_id] = data.copy()
            self.y_up[1]['t'] = children_time.copy()
        except AttributeError:
            self.y_up[1][micro_id] = data
            self.y_up[1]['t'] = children_time

        if (self.debug):
            # print("t: {} ms, I'm {} and I received this micro state {}".format(self.current_time,self.name,x_b_micro))
            print("t: {:.2f} s, Coupled name: {}, Global Transition Function, x_b_micro: {}".format(children_time,self.name,x_b_micro))

    def select(self, immChildren):
        """
        Choose a model to transition from all possible models.
        """
        # Doesn't really matter, as they don't influence each other
        return immChildren[0]

#----------------------------
# Robot i
#----------------------------
class Robot(CoupledDEVS):
    def __init__(self, name='Robot', dQMin=1e-6, dQRel=1e-3, x0=0.0, y0=0.0, gainx=1, gainy=1, comm_range=np.inf, debug=False):
        """
        A robot model composed of the robot's pysics.
        """
        # Always call parent class' constructor FIRST:
        CoupledDEVS.__init__(self, name)

        # Parameters
        self.dQMin  = dQMin
        self.dQRel  = dQRel
        self.x0     = x0
        self.y0     = y0
        self.gainx  = gainx
        self.gainy  = gainy
        self.comm_range = comm_range
        self.debug  = debug

        self.y_up = [self.name, {'Time': 0.0, 'Pose': [x0,  y0], 'CommRange': comm_range}]
        self.current_time = 0

        dynamics = RobotDynamics(name="RobotDynamics",
                          dQMin=self.dQMin,
                          dQRel=self.dQRel,
                          x0=self.x0,
                          y0=self.y0,
                          gainx=self.gainx,
                          gainy=self.gainy,
                          debug=self.debug
                         )
        controller    = Controller(robot_id=self.name, 
                                   name='Controller'
                                   )
        token_handler = TokenHandler(robot_id=self.name,
                                     name='Token_Handler',
                                     debug=self.debug
                                     )
        kalman_filter = KalmanFilter(robot_id=self.name,
                                     name='Kalman_Filter',
                                     debug=self.debug
                                     )

        self.dynamics       = self.addSubModel(dynamics)
        # self.splitter_gen  = self.addSubModel(splitter_gen)
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

        if (self.debug):
            print("t: 0 s, Coupled name: {}, Init Function".format(self.name))

    def globalTransition(self, e_g, x_b_micro, *args, **kwargs):
        # self.current_time += e_g

        micro_id, data = x_b_micro
        try:
            self.y_up[1]['Pose'][0] = data['x'].copy()
            self.y_up[1]['Pose'][1] = data['y'].copy()
            self.y_up[1]['Time'] = data['t'].copy()
            current_time = data['t'].copy()
        except AttributeError:
            self.y_up[1]['Pose'][0] = data['x']
            self.y_up[1]['Pose'][1] = data['y']
            self.y_up[1]['Time'] = data['t']
            current_time = data['t']

        if (self.debug):
            print("t: {:.2f} s, Coupled name: {}, Global Transition Function, x_b_micro: {}".format(current_time,self.name,x_b_micro))

    def select(self, immChildren):
        """
        Choose a model to transition from all possible models.
        """
        # Doesn't really matter, as they don't influence each other
        return immChildren[0]

class MultiRobotSystem(CoupledDEVS):
    def __init__(self, name='MultiRobotSystem', debug=False):
        """
        Multi robot system composed of N robots.
        """
        # Always call parent class' constructor FIRST:
        CoupledDEVS.__init__(self, name)

        self.debug = debug

        self.current_time = 0 # TODO: time cannot be managed as in the other coupled/atomic models

        self.robots = {}
        robots_config = read_json_file('robots.json')
        for robot, config in robots_config.items():
            self.robots[robot] = self.addSubModel(Robot(**config, debug=self.debug))

        # log new value of micro_states
        summary = {'robot_ids': list(self.robots.keys())}
        write_json_file('output/summary.json', summary)

        self.robots_states = {}

        self.router = self.addSubModel(
            Router(agents_ids=list(robots_config.keys()), 
                   name='Router',
                   debug=self.debug
                   )
        )
        self.connectPorts(self.robots['Robot_0'].OUT_router_token, self.router.in_agent_token['Robot_0'])
        self.connectPorts(self.robots['Robot_1'].OUT_router_token, self.router.in_agent_token['Robot_1'])
        self.connectPorts(self.robots['Robot_2'].OUT_router_token, self.router.in_agent_token['Robot_2'])
        self.connectPorts(self.robots['Robot_3'].OUT_router_token, self.router.in_agent_token['Robot_3'])
        self.connectPorts(self.router.out_agent_token['Robot_0'], self.robots['Robot_0'].IN_router_token)
        self.connectPorts(self.router.out_agent_token['Robot_1'], self.robots['Robot_1'].IN_router_token)
        self.connectPorts(self.router.out_agent_token['Robot_2'], self.robots['Robot_2'].IN_router_token)
        self.connectPorts(self.router.out_agent_token['Robot_3'], self.robots['Robot_3'].IN_router_token)

        if (self.debug):
            print("t: 0 s, Coupled name: {}, Init Function".format(self.name))

    def globalTransition(self, e_g, x_b_micro, *args, **kwargs):
        # self.current_time += e_g
        micro_id, data = x_b_micro
        try:
            self.robots_states[micro_id] = data.copy()
            current_time = data['Time'].copy()
        except AttributeError:
            self.robots_states[micro_id] = data
            current_time = data['Time']
        
        # print("Coupled name: {}, Global Transition Function, micro_id: {}".format(self.name,micro_id))
        if (self.debug):
            print(
                 "t: {} s, Coupled name: {}, Global Transition Function, x_b_micro: {}, global state: {}"
                 .format(current_time, self.name,x_b_micro,self.robots_states))

        # log new value of micro_states
        log = [micro_id, data['Time']]
        log += data['Pose']
        log += [data['CommRange']]
        log += [
            neighbor_id
            for neighbor_id in self.robots_states.keys()
            if self.connected(micro_id, neighbor_id)
        ]
        append_csv_file('output/data.csv', log)

    def getContextInformation(self, transmitter_id):
        return [
            (receiver_id, None)
            for receiver_id, _ in self.robots_states.items()
            if self.connected(transmitter_id, receiver_id)
        ]

    def select(self, immChildren):
        """
        Choose a model to transition from all possible models.
        """
        # Doesn't really matter, as they don't influence each other
        return immChildren[0]

    def distance(self, p_1, p_2):
        return np.sqrt((p_1[0] - p_2[0])**2 + (p_1[1] - p_2[1])**2)

    def connected(self, transmitter_id, receiver_id):
        if transmitter_id == receiver_id:
            return False

        transmitter_pose = self.robots_states[transmitter_id]['Pose']
        receiver_pose = self.robots_states[receiver_id]['Pose']

        distance = self.distance(transmitter_pose, receiver_pose)
        trasmiter_range = self.robots_states[transmitter_id]['CommRange']
        receiver_range = self.robots_states[receiver_id]['CommRange']

        return (distance < trasmiter_range) and (distance < receiver_range)
