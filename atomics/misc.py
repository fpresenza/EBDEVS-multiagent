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
from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY

from atomics.qsstools import advance_time

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
        Output Function.
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
class SplitterState:
    """
    Encapsulates the system's state
    """

    def __init__(self, sigma, tvalue, data):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue, data)

    def set(self, sigma, tvalue, data):
        self._sigma  = sigma
        self._tvalue = tvalue
        self._data = data

    def get(self):
        return self._sigma, self._tvalue, self._data


class Splitter(AtomicDEVS):
    """
    Split input message in as many outputs as elements the message has 
    """
  
    def __init__(self, num_outputs, name='Splitter', debug=False):
        """
        Constructor (parameterizable).
        """
        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)
    
        # PARAMETERS
        self.num_outputs = num_outputs # number of output ports
        self.debug = debug

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.state = SplitterState(sigma=INFINITY, tvalue=0.0, data=[])

        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0
    
        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.inPort = self.addInPort(name="in")
        self.outPorts = {}
        for i in range(self.num_outputs):
            self.outPorts[i] = self.addOutPort(name="out_{}".format(i))

        self.outputs_queue = []

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))


    def __lt__(self, other):
        return self.name < other.name

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        _, current_time, _ = self.state.get()
        current_time += self.elapsed

        # Received a new event, so start processing it
        data = np.ravel(inputs[self.inPort])    # serialize data

        sigma = 0.0
        if (self.debug):
            print("t: {} s, Atomic name: {}, External Transition Function".format(current_time, self.name))

        return SplitterState(sigma, current_time, data) 

    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time, data = self.state.get()
        current_time += sigma

        if len(self.outputs_queue) == 0:
            sigma = INFINITY
        else:
            sigma = 0.0
        
        if (self.debug):
            print("t: {} s, Atomic name: {}, Internal Transition Function".format(current_time,self.name))

        return SplitterState(sigma, current_time, data)

    def outputFnc(self):
        """
        Output Function.
        """
        if len(self.outputs_queue) == 0:
            _, _, data = self.state.get()
            for i, splitted_data in enumerate(data):
                self.outputs_queue.append({self.outPorts[i]: [splitted_data]})

        return self.outputs_queue.pop()
    
    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _, _ = self.state.get()
        return sigma

######################## 
# Merger atomic model
########################
class MergerState:
    """
    Encapsulates the system's state
    """

    def __init__(self, sigma, tvalue, data):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue, data)

    def set(self, sigma, tvalue, data):
        self._sigma  = sigma
        self._tvalue = tvalue
        self._data = data

    def get(self):
        return self._sigma, self._tvalue, self._data


class Merger(AtomicDEVS):
    """
    Merger several input messages in one output 
    """
  
    def __init__(self, num_inputs, name='Merger', debug=False):
        """
        Constructor (parameterizable).
        """
        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)
    
        # PARAMETERS
        self.num_inputs = num_inputs # number of input ports
        self.debug = debug

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.state = MergerState(sigma=INFINITY, tvalue=0.0, data=[None] * num_inputs)

        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0
    
        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.inPorts = {}
        for i in range(self.num_inputs):
            self.inPorts[i] = self.addInPort(name="in_{}".format(i))

        self.outPort = self.addOutPort(name="out")

        self.outputs_queue = []

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))


    def __lt__(self, other):
        return self.name < other.name

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        _, current_time, data = self.state.get()
        current_time += self.elapsed

        # Received a new event, so start processing it
        for i in range(self.num_inputs):
            if self.inPorts[i] in inputs:
                data[i] = inputs[self.inPorts[i]]
            else:
                if data[i] != None:
                    data[i] = advance_time(data[i], self.elapsed, order=-1)

        sigma = 0.0

        if (self.debug):
            print("t: {} s, Atomic name: {}, External Transition Function".format(current_time, self.name))

        return MergerState(sigma, current_time, data) 

    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time, data = self.state.get()
        current_time += sigma
        sigma = INFINITY
        
        if (self.debug):
            print("t: {} s, Atomic name: {}, Internal Transition Function".format(current_time,self.name))

        return MergerState(sigma,current_time,data)

    def outputFnc(self):
        """
        Output Function.
        """
        _, _, data = self.state.get()

        return {self.outPort: data}
    
    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _, _ = self.state.get()
        return sigma
