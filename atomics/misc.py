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

import sys
import numpy as np

# Import code for DEVS model representation:
from pypdevs.DEVS import *
from pypdevs.infinity import INFINITY

#################### 
# Gain atomic model 
####################
class Gain(AtomicDEVS):
    """
    A gain block 
    """
  
    def __init__(self, name=None, gain=1):
        """
        Constructor (parameterizable).
        """
        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)
    
        # STATE:
        #  Define 'state' attribute (initial sate):
        self.gain  = gain
        self.u = [0.0, 0.0]
        self.sigma = INFINITY

        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0
        # with elapsed time initially 1.5 and initially in 
        # state "red", which has a time advance of 60,
        # there are 60-1.5 = 58.5time-units  remaining until the first 
        # internal transition 
    
        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.OUT = self.addOutPort(name="OUT")
        self.IN  = self.addInPort(name="IN")

    def __lt__(self, other):
        return self.name < other.name

    def intTransition(self):
        """
        Internal Transition Function.
        The policeman works forever, so only one mode. 
        """
        self.sigma = INFINITY
        return self.sigma

    def extTransition(self, inputs):
        """
        External Transition Function.
        """

        # Received a new event, so start processing it
        in0    = inputs[self.IN][0]
        self.u = [in0, 0.0]
        self.sigma  = 0.0
        return self.sigma 

    def outputFnc(self):
        """
        Output Funtion.
        """
   
        # A colourblind observer sees "grey" instead of "red" or "green".
 
        # BEWARE: ouput is based on the OLD state
        # and is produced BEFORE making the transition.
        # We'll encode an "observation" of the state the
        # system will transition to !

        # Send messages (events) to a subset of the atomic-DEVS' 
        # output ports by means of the 'poke' method, i.e.:
        # The content of the messages is based (typically) on current State.
 
        y = [self.u[0] * self.gain, 0.0]
        return {self.OUT: y}
    
    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        return self.sigma

######################## 
# Splitter atomic model
########################

class SplitterGeneratorState:
    """
    Encapsulates the system's state
    """

    def __init__(self, sigmaval=0.1, tval=0.0):
        """
        Constructor (parameterizable).
        """
        self.set(sigmaval, tval)

    def set(self, sigmavalue, tvalue):
        self._sigma  = sigmavalue
        self._tvalue = tvalue

    def get(self):
        return self._sigma, self._tvalue

class SplitterGenerator(AtomicDEVS):
    def __init__(self,name=None,period=1):
        """
        Atomic model for generating the splitter inputs
        """

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # STATE:
        #  Define 'state' attribute (initial sate):
        _time0  = 0.0
        _sigma0 = 0
        self.state = SplitterGeneratorState(_sigma0,_time0) 
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.out_splitter_in = self.addOutPort(name="out_splitter_in")
 
        # Parameters
        self.msgs = [
            {self.out_splitter_in: np.array([-1.0, 1.0])},
            {self.out_splitter_in: np.array([1.2, 5.6])},
            {self.out_splitter_in: np.array([-4.8, 2.3])},
            {self.out_splitter_in: np.array([-1.7, 9.8])},
            {self.out_splitter_in: np.array([-5.2, 7.5])},
        ]
        self.period = period

    def __lt__(self, other):
        return self.name < other.name

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        # it should never be executed
        sigma, current_time = self.state.get()
        current_time += self.elapsed
        return SplitterGeneratorState(sigma,current_time) 
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time = self.state.get()
        current_time += sigma
        self.msgs.pop()
        if len(self.msgs) == 0:
            sigma = INFINITY
        else:
            sigma = self.period
        return SplitterGeneratorState(sigma,current_time) 
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        # sigma, current_time = self.state.get()
        return self.msgs[-1]

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, current_time = self.state.get()
        return sigma

class SplitterState:
    """
    Encapsulates the system's state
    """

    def __init__(self, sigmaval=0.1, tval=0.0, uval=[]):
        """
        Constructor (parameterizable).
        """
        self.set(sigmaval, tval, uval)

    def set(self, sigmavalue, tvalue, uval):
        self._sigma  = sigmavalue
        self._tvalue = tvalue
        self._uval = uval

    def get(self):
        return self._sigma, self._tvalue, self._uval

class Splitter(AtomicDEVS):
    """
    Split input message in as many outputs as elements the message has 
    """
  
    def __init__(self, name=None, numoutputs=1, debug=False):
        """
        Constructor (parameterizable).
        """
        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)
    
        # PARAMETERS
        self.N  = numoutputs # number of output ports
        self.debug = debug

        # STATE:
        #  Define 'state' attribute (initial sate):
        _time0 = 0.0
        _sigma0 = INFINITY
        _data0 = []
        self.state = SplitterState(_sigma0,_time0,_data0)

        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0
    
        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.in_splitter_msgs = self.addInPort(name="IN")
        self.out_splitter_msgs = []
        for i in range(self.N):
            self.out_splitter_msgs.append(self.addOutPort(name="out_splitter_port_{}".format(i)))
        # self.out_splitter_port0 = self.addOutPort(name="out_splitter_port0")
        # self.out_splitter_port1 = self.addOutPort(name="out_splitter_port1")

    def __lt__(self, other):
        return self.name < other.name

    def intTransition(self):
        """
        Internal Transition Function.
        """
        _, current_time, data = self.state.get()
        data.pop()
        if len(data) == 0:
            sigma = INFINITY
        else:
            sigma = 0.0

        return SplitterState(sigma,current_time,data)

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        _, current_time, _ = self.state.get()
        current_time += self.elapsed

        # Received a new event, so start processing it
        in0   = inputs[self.in_splitter_msgs]
        i=0
        data = []
        for msg in in0:
            data.append({self.out_splitter_msgs[i]: [msg]})
            i += 1
        # data = [
        #        {self.out_splitter_port0: 0},
        #        {self.out_splitter_port1: 1}
        # ]
        sigma = 0

        return SplitterState(sigma,current_time,data) 

    def outputFnc(self):
        """
        Output Funtion.
        """
        sigma, current_time, data = self.state.get()
        if (self.debug):
            print("I'm the splitter, and I'm sending: {}".format(data[-1]))
        return data[-1]
    
    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _, _ = self.state.get()
        return sigma

