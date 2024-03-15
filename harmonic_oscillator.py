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
from atomics.collector import Collector

class OscillatorSystem(CoupledDEVS):
    def __init__(self, name='MySystem'):
        """
        A simple oscillator composed of two integrators (equivalent to a mass-spring system with m=1 and k=1).
        """
        # Always call parent class' constructor FIRST:
        CoupledDEVS.__init__(self, name)

        # Declare the coupled model's output ports:
        # Autonomous, so no output ports

        # Declare the coupled model's sub-models:
        # self.periodicgenerator = self.addSubModel(PeriodicGenerator(name="periodic", peri=10))
        # self.singenerator = self.addSubModel(SinusoidalGenerator(name="sin", freq=1, amp=1, phi=0, samp=0.1))
        self.integrator1 = self.addSubModel(QSSIntegrator(name="qss1", dQMin=1e-6, dQRel=1e-3, gain=-1, x0=0, debug=False))
        self.integrator2 = self.addSubModel(QSSIntegrator(name="qss2", dQMin=1e-6, dQRel=1e-3, gain= 1, x0=1, debug=False))
        self.collector   = self.addSubModel(Collector(name="Collector",filename="output/output.csv"))

        # Only connect ...
        self.connectPorts(self.integrator1.OUT_q, self.integrator2.IN_dx)
        self.connectPorts(self.integrator2.OUT_q, self.integrator1.IN_dx)
        self.connectPorts(self.integrator1.OUT_q, self.collector.in1_event)
        self.connectPorts(self.integrator2.OUT_q, self.collector.in2_event)

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

