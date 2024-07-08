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
        self.y_up  = [self.name, None]

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

        if self.debug:
            print("Internal Transition Function @ {} - t: {}, xprev: {}, x: {}, q: {}, sigma: {}".format(self.name,current_time,xprev,x,q,sigma))

        # shares information to the parent to compute the Global Transition function
        try:
            self.y_up[1] = q.copy()
        except AttributeError:
            self.y_up[1] = q

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

                if self.debug:
                    print("External Transition Function @ {} - t: {}, dx: {}, x: {}, sigma: {}, sigma_lo: {}, sigma_up: {}"\
                            .format(self.name,current_time,derx_val,x,sigma,sigma_lo,sigma_up))

        else:
            x[0] = derx_val
            sigma = 0

        # shares information to the parent to compute the Global Transition function
        try:
            self.y_up[1] = q.copy()
        except AttributeError:
            self.y_up[1] = q

        return QSSState(q,x,sigma,current_time)

#----------------------------
# Physics for Robot i
#----------------------------
class Physics(CoupledDEVS):
    def __init__(self, name='Physics', dQMin=1e-6, dQRel=1e-3, x0=0.0, y0=0.0, gainx=1, gainy=1):
        """
        Robot's physics submodel composed of two integrators for x and y and a splitter.
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

        # dictionary to save childrens' states
        self.y_up = [self.name, {'x': x0, 'y': y0}]
        self.current_time = 0

        # Declare childrens: splitterx2, QSS integ x 2
        splitter     = Splitter(name="splitter",numoutputs=2)
        integrator_x = QSSIntegrator_Yup(name="x", 
                                         dQMin=self.dQMin, 
                                         dQRel=self.dQRel, 
                                         gain=self.gainx, 
                                         x0=self.x0, 
                                         debug=False
                                         )
        integrator_y = QSSIntegrator_Yup(name="y",
                                         dQMin=self.dQMin, 
                                         dQRel=self.dQRel, 
                                         gain=self.gainy, 
                                         x0=self.y0, 
                                         debug=False
                                         )
        self.splitter     = self.addSubModel(splitter)
        self.integrator_x = self.addSubModel(integrator_x)
        self.integrator_y = self.addSubModel(integrator_y)

        # Declare the coupled model's output ports:
        self.OUT_physics_x    = self.addOutPort(name="OUT_physics_x")
        self.OUT_physics_y    = self.addOutPort(name="OUT_physics_y")
        self.IN_physics_vx_vy = self.addInPort( name="IN_physics_vx_vy")

        # Connect coupled model's ports with atomic models' ports
        self.connectPorts(self.IN_physics_vx_vy, self.splitter.in_splitter_msgs)
        self.connectPorts(self.splitter.out_splitter_msgs[0], self.integrator_x.IN_dx)
        self.connectPorts(self.splitter.out_splitter_msgs[1], self.integrator_y.IN_dx)
        self.connectPorts(self.integrator_x.OUT_q, self.OUT_physics_x)
        self.connectPorts(self.integrator_y.OUT_q, self.OUT_physics_y)

    def globalTransition(self, e_g, x_b_micro, *args, **kwargs):
        # update each coordinate separatedly, since the events in the integrators for the same robot do not need to be simultaneous
        self.current_time += e_g

        micro_id, data = x_b_micro[0]
        try:
            self.y_up[1][micro_id] = data.copy()
        except AttributeError:
            self.y_up[1][micro_id] = data

        print("t: {} ms, I'm {} and I received this micro state {}".format(self.current_time,self.name,x_b_micro))

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
    def __init__(self, name='Robot', dQMin=1e-6, dQRel=1e-3, x0=0.0, y0=0.0, gainx=1, gainy=1):
        """
        A robot model composed of the robot's pysics.
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

        self.y_up = [self.name, {'Physics': [x0,  y0]}]
        self.current_time = 0

        physics = Physics(name="Physics",
                          dQMin=self.dQMin,
                          dQRel=self.dQRel,
                          x0=self.x0,
                          y0=self.y0,
                          gainx=self.gainx,
                          gainy=self.gainy
                         )
        splitter_gen = SplitterGenerator(period=1,
                                         name='Splitter_Gen'
                                        )
        self.physics      = self.addSubModel(physics)
        self.splitter_gen = self.addSubModel(splitter_gen)

        # Declare the coupled model's output ports:
        # self.IN_vx_vy = self.addInPort(name="robot_vx_vy")
        self.OUT_x    = self.addOutPort(name="robot_x")
        self.OUT_y    = self.addOutPort(name="robot_y")

        # Connect coupled model's ports with atomic models' ports
        self.connectPorts(self.physics.OUT_physics_x, self.OUT_x)
        self.connectPorts(self.physics.OUT_physics_y, self.OUT_y)
        # self.connectPorts(self.IN_vx_vy, self.splitter_gen.in_splitter_msgs)
        self.connectPorts(self.splitter_gen.out_splitter_in, self.physics.IN_physics_vx_vy) 

    def globalTransition(self, e_g, x_b_micro, *args, **kwargs):
        self.current_time += e_g

        micro_id, data = x_b_micro
        # if micro_id == 'Physics'
        try:
            self.y_up[1]['Physics'][0] = data['x'].copy()
            self.y_up[1]['Physics'][1] = data['y'].copy()
        except AttributeError:
            self.y_up[1]['Physics'][0] = data['x']
            self.y_up[1]['Physics'][1] = data['y']


        print("t: {} ms, I'm {} and I received this micro state {}".format(self.current_time,self.name,x_b_micro))

    def select(self, immChildren):
        """
        Choose a model to transition from all possible models.
        """
        # Doesn't really matter, as they don't influence each other
        return immChildren[0]


# class Environment(CoupledDEVS):
#     def __init__(self, name=None):
#        """
#        A simple 
#        """
        # Always call parent class' constructor FIRST:
#        CoupledDEVS.__init__(self, name)
        # Sg: estructura que almacena los coeficientes 

    # Funcion que se ejecuta cuando un atomico ejecuta su Y_up()
#    def globalTransition(self, e_g, x_b_micro, *args, **kwargs):
#        super(Environment, self).globalTransition(e_g, x_b_micro, *args, **kwargs)
        # almacena los coeficientes de los polinomios de las posiciones de los agentes

    # Funcion que se ejecuta cuando un atomico pide informacion al estado global: Y_down()
#    def getContextInformation(self, property, *args, **kwargs):
#        super(Environment, self).getContextInformation(property)
        # calculo de los vecinos de un atomico a pedido del router

class MultiRobotSystem(CoupledDEVS):
    def __init__(self, name='MultiRobotSystem', number=4):
        """
        Multi robot system composed of N robots.
        """
        # Always call parent class' constructor FIRST:
        CoupledDEVS.__init__(self, name)

        self.micro_states = {'Physics': {}}
        # for i in range(number):
        #     self.micro_states['x'+str(i)] = 0
        #     self.micro_states['y'+str(i)] = 0
        self.current_time = 0
        self.max_dist = 6.0

        robot1 = Robot(name="Robot_1",
                       dQMin=1e-6,
                       dQRel=1e-3,
                       x0=0.0,
                       y0=0.0,
                       gainx=1,
                       gainy=1
                      )
        robot2 = Robot(name="Robot_2",
                       dQMin=1e-6,
                       dQRel=1e-3,
                       x0=5.0,
                       y0=3.0,
                       gainx=1,
                       gainy=1
                      )
        robot3 = Robot(name="Robot_3",
                       dQMin=1e-6,
                       dQRel=1e-3,
                       x0=-2.0,
                       y0=-2.0,
                       gainx=1,
                       gainy=1
                      )
        robot4 = Robot(name="Robot_4",
                       dQMin=1e-6,
                       dQRel=1e-3,
                       x0=5.0,
                       y0=-3.0,
                       gainx=1,
                       gainy=1
                      )
        self.robot1 = self.addSubModel(robot1)
        self.robot2 = self.addSubModel(robot2)
        self.robot3 = self.addSubModel(robot3)
        self.robot4 = self.addSubModel(robot4)
        # robots' position evolution from initial conditions (no external source)

        # Declare the coupled model's output ports => no output ports

    def globalTransition(self, e_g, x_b_micro, *args, **kwargs):
        self.current_time += e_g

        micro_id, data = x_b_micro
        try:
            self.micro_states['Physics'][micro_id] = data['Physics'].copy()
        except AttributeError:
            self.micro_states['Physics'][micro_id] = data['Physics']

        print("t: {} ms, I'm {} and the state of all my children is {}".format(self.current_time,self.name,self.micro_states['Physics']))

    def getContextInformation(self, transmitter):
        # return self.micro_states[]
        physics = self.micro_states['Physics']
        p = physics[transmitter]
        return [
            (robot_id, self.distance(p, q))
            for robot_id, q in physics.items()
            if self.connected(p, q)
        ]

    def select(self, immChildren):
        """
        Choose a model to transition from all possible models.
        """
        # Doesn't really matter, as they don't influence each other
        return immChildren[0]

    def distance(self, p, q):
        return np.sqrt((p[0] - q[0])**2 + (p[1] - q[1])**2)

    def connected(self, p, q):
        return self.distance(p, q) < self.max_dist