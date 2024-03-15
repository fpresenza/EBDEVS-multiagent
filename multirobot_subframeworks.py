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

from pypdevs.DEVS import CoupledDEVS

# Import all models to couple
# from generators import PeriodicGenerator, SinusoidalGenerator 
from atomics.qssintegrators import QSSIntegrator 
from atomics.collector import CollectorX8
from atomics.generators import SinusoidalGenerator
from atomics.misc import Gain

class Robot(CoupledDEVS):
    def __init__(self, name='Robot', dQMin=1e-6, dQRel=1e-3, x0=0, y0=0, gainx=1, gainy=1):
        """
        A robot model composed of two integrators.
        """
        # Always call parent class' constructor FIRST:
        CoupledDEVS.__init__(self, name)

        # Parameters
        self.dQMin = dQMin
        self.dQRel = dQRel
        self.x0 = x0
        self.y0 = y0
        self.gainx = gainx
        self.gainy = gainy

        # Declare atomics
        integ1 = QSSIntegrator(name="x", dQMin=self.dQMin, dQRel=self.dQRel, gain=self.gainx, x0=self.x0, debug=False)
        integ2 = QSSIntegrator(name="y", dQMin=self.dQMin, dQRel=self.dQRel, gain=self.gainy, x0=self.y0, debug=False)

        # Declare the coupled model's sub-models:
        self.integrator1 = self.addSubModel(integ1)
        self.integrator2 = self.addSubModel(integ2)

        # Declare the coupled model's output ports:
        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.OUT_x = self.addOutPort(name="robot_x")
        self.OUT_y = self.addOutPort(name="robot_y")
        self.IN_vx = self.addInPort(name="robot_vx")
        self.IN_vy = self.addInPort(name="robot_vy")

        # Connect coupled model's ports with atomic models' ports
        self.connectPorts(self.IN_vx, self.integrator1.IN_dx)
        self.connectPorts(self.IN_vy, self.integrator2.IN_dx)
        self.connectPorts(self.integrator1.OUT_q, self.OUT_x)
        self.connectPorts(self.integrator2.OUT_q, self.OUT_y)

class MultiRobotSystem(CoupledDEVS):
    def __init__(self, name='MySystem'):
        """
        A simple oscillator composed of two integrators (equivalent to a mass-spring system with m=1 and k=1).
        """
        # Always call parent class' constructor FIRST:
        CoupledDEVS.__init__(self, name)

        # Declare the coupled model's output ports:
        # Autonomous, so no output ports

        # Declare the coupled model's sub-models:
        self.collector = self.addSubModel(CollectorX8(name="Collector",filename="output/output.csv"))
        self.gain10     = self.addSubModel(Gain(name="Gain10", gain=1))
        self.gain11     = self.addSubModel(Gain(name="Gain11", gain=1))
        self.gain20     = self.addSubModel(Gain(name="Gain20", gain=1))
        self.gain21     = self.addSubModel(Gain(name="Gain21", gain=1))
        self.gain30     = self.addSubModel(Gain(name="Gain30", gain=1))
        self.gain31     = self.addSubModel(Gain(name="Gain31", gain=1))
        self.gain40     = self.addSubModel(Gain(name="Gain40", gain=1))
        self.gain41     = self.addSubModel(Gain(name="Gain41", gain=1))
        # self.singen    = self.addSubModel(SinusoidalGenerator(name="SinGen", freq=1, amp=1, phi=0, samp=0.01))
        # self.cosgen    = self.addSubModel(SinusoidalGenerator(name="SinGen", freq=0.1, amp=2, phi=1.57, samp=0.01))

        
        self.robot1 = self.addSubModel(Robot(name="Robot_1", dQMin=1e-6, dQRel=1e-3, x0=1, y0=-1, gainx=-1, gainy=1))
        self.robot2 = self.addSubModel(Robot(name="Robot_2", dQMin=1e-6, dQRel=1e-3, x0=2, y0=-2, gainx=-1, gainy=1))
        self.robot3 = self.addSubModel(Robot(name="Robot_3", dQMin=1e-6, dQRel=1e-3, x0=3, y0=-3, gainx=-1, gainy=1))
        self.robot4 = self.addSubModel(Robot(name="Robot_4", dQMin=1e-6, dQRel=1e-3, x0=4, y0=-4, gainx=-1, gainy=1))

        # Only connect ...
        # self.connectPorts(self.singen.OUT, self.robot1.IN_vx)
        # self.connectPorts(self.cosgen.OUT, self.robot1.IN_vy)

        # no soporta la conexión directa de puertos de entrada y salida,
        # self.connectPorts(self.robot1.OUT_x, self.robot1.IN_vy)
        # self.connectPorts(self.robot1.OUT_y, self.robot1.IN_vx)
        
        # para solucionarlo uso un bloque ganancia
        self.connectPorts(self.robot1.OUT_x, self.gain10.IN)
        self.connectPorts(self.gain10.OUT, self.robot1.IN_vy)
        self.connectPorts(self.robot1.OUT_y, self.gain11.IN)
        self.connectPorts(self.gain11.OUT, self.robot1.IN_vx)

        self.connectPorts(self.robot2.OUT_x, self.gain20.IN)
        self.connectPorts(self.gain20.OUT, self.robot2.IN_vy)
        self.connectPorts(self.robot2.OUT_y, self.gain21.IN)
        self.connectPorts(self.gain21.OUT, self.robot2.IN_vx)
        
        self.connectPorts(self.robot3.OUT_x, self.gain30.IN)
        self.connectPorts(self.gain30.OUT, self.robot3.IN_vy)
        self.connectPorts(self.robot3.OUT_y, self.gain31.IN)
        self.connectPorts(self.gain31.OUT, self.robot3.IN_vx)
        
        self.connectPorts(self.robot4.OUT_x, self.gain40.IN)
        self.connectPorts(self.gain40.OUT, self.robot4.IN_vy)
        self.connectPorts(self.robot4.OUT_y, self.gain41.IN)
        self.connectPorts(self.gain41.OUT, self.robot4.IN_vx)

        self.connectPorts(self.robot1.OUT_x, self.collector.in1_event)
        self.connectPorts(self.robot1.OUT_y, self.collector.in2_event)
        self.connectPorts(self.robot2.OUT_x, self.collector.in3_event)
        self.connectPorts(self.robot2.OUT_y, self.collector.in4_event)
        self.connectPorts(self.robot3.OUT_x, self.collector.in5_event)
        self.connectPorts(self.robot3.OUT_y, self.collector.in6_event)
        self.connectPorts(self.robot4.OUT_x, self.collector.in7_event)
        self.connectPorts(self.robot4.OUT_y, self.collector.in8_event)

#    def select(self, immChildren):
#        """
#        Choose a model to transition from all possible models.
#        """
#        # Policeman has priority over the traffic light
#        if self.policeman in immChildren:
#            return self.policeman
#        else:
#            # Doesn't really matter, as they don't influence each other
#            return immChildren[0]

