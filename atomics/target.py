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
        self.set(sigma, tval, dataval)

    def set(self, sigma, tvalue, datavalue):
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
        self.position  = config['position']
        self.comm_range = config['comm_range']
        self.period = config['period']
        self.debug = debug

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.state = TargetState(
            sigma=self.period,
            tvalue=0.0,
            status='Active'
        ) 

        # initialize y_up
        self.y_up = [
            self.name, 
            {
                'time': 0.0, 
                'pose': [coord + [0.0] * 9 for coord in self.position], # 10-tuple
                'comm_range': self.comm_range,
                'status': 'Active',
            }
        ]

        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0
    
        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.OUT_router_target = self.addOutPort(name="OUT_router_target")
        self.IN_router_target  = self.addInPort(name="IN_router_target")

    def __lt__(self, other):
        return self.name < other.name

    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time, status = self.state.get()
        current_time += sigma

        if status == 'Passive':
            sigma = INFINITY
            self.y_up[1]['status'] = 'Passive'
        else:
            sigma = self.period

        if (self.debug):
            print(
                "t: {:.2f} s, Parent name: {}, Atomic name: {}, Internal Transition Function, Status: {}"
                .format(current_time, self.parent.parent.name, self.name,status)
            )
            
        return TargetState(sigma, current_time, status) 
    
    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, _ = self.state.get()
        current_time += self.elapsed

        if self.IN_router_target in inputs: # external events turn off the target
            status = 'Passive' # target passivated
            sigma  = 0.0

        if (self.debug):
            print(
                "t: {:.2f} s, Atomic name: {}, External Transition Function, Status: {}"
                .format(current_time, self.name, status)
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
                .format(current_time,self.parent.parent.name,self.name,status)
            )

        return {self.OUT_router_target: status}
    
    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _, _ = self.state.get()
        return sigma