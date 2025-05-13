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
   
class TargetState:
    """
    Encapsulates the system's state
    """

    def __init__(self, sigma=0.1, tvalue=0.0, status=None):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue, status)

    def set(self, sigma, tvalue, status):
        self._sigma  = sigma
        self._tvalue = tvalue
        self._status = status

    def get(self):
        return self._sigma, self._tvalue, self._status

class Target(AtomicDEVS):
    """
    A target atomic model 
    """
  
    def __init__(
            self,
            config, # position, comm_range
            name = 'Target', 
            debug=False
        ):
        """
        Constructor (parameterizable).
        """
        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)
    
        # PARAMETERS
        self.position = config['position']
        self.period = config['period']
        self.comm_range = config['comm_range']
        self.collect_range = config['collect_range']
        self.debug = debug

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.state = TargetState(
            sigma=self.period,
            tvalue=0.0,
            status='active'
        ) 

        # initialize y_up
        self.y_up = [
            self.name, 
            {
                'time': 0.0, 
                'pose': [coord + [0.0] * 9 for coord in self.position], # 10-tuple
                'comm_range': self.comm_range,
                'status': 'active',
            }
        ]

        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0
    
        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.inPorts = {'radio': self.addInPort(name="in_radio")}
        self.outPorts = {'radio': self.addOutPort(name="out_radio")}

    def __lt__(self, other):
        return self.name < other.name


    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, status = self.state.get()
        current_time += self.elapsed
        sigma -= self.elapsed    # holds last status

        self.y_up[1]['time'] = current_time
        transmitter, (_, distance_measurement) = inputs[self.inPorts['radio']]
        if transmitter.startswith('Robot'):
            if (status == 'active') and  (distance_measurement < self.collect_range * 0.9):
                status = 'passive'
                sigma = INFINITY
                self.y_up[1]['status'] = 'passive'

        if (self.debug):
            print(
                "t: {:.2f} s, Atomic name: {}, External Transition Function, Status: {}"
                .format(current_time, self.name, status)
            )

        return TargetState(sigma, current_time, status)


    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time, status = self.state.get()
        current_time += sigma
        sigma = self.period

        self.y_up[1]['time'] = current_time

        if (self.debug):
            print(
                "t: {:.2f} s, Parent name: {}, Atomic name: {}, Internal Transition Function, Status: {}"
                .format(current_time, self.parent.parent.name, self.name,status)
            )
            
        return TargetState(sigma, current_time, status) 
    

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
 
        sigma, current_time, status = self.state.get()

        if (self.debug):
            print(
                "t: {:.2f} s, Parent name: {}, Atomic name: {}, Output Function, Status: {}"
                .format(current_time,self.parent.parent.name, self.name, status)
            )

        return {self.outPorts['radio']: (self.name, self.position)}
    
    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _, _ = self.state.get()
        return sigma