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
from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY

from atomics.integrators.qss1tools import advance_time, pad_zeros

#################################
#  Dynamics Function atomic model
#################################


class DynamicsFunctionState:
    """
    Encapsulates the system's state
    """

    def __init__(self, sigma, tvalue, data):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue, data)

    def set(self, sigma, tvalue, data):
        self._sigma = sigma
        self._tvalue = tvalue
        self._data = data

    def get(self):
        return self._sigma, self._tvalue, self._data


class DynamicsFunction(AtomicDEVS):
    """
    Evaluates the dynamic function of the system: xdot = f(x,u)
    """

    def __init__(self, num_outputs, name='DynamicsFunction', debug=False):
        """
        Constructor (parameterizable).
        """
        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # PARAMETERS
        self.num_outputs = num_outputs  # number of output ports
        self.debug = debug

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.state = DynamicsFunctionState(
            sigma=INFINITY,
            tvalue=0.0,
            data=[None, None]
        )

        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.inPorts = {
            'control_action': self.addInPort(name="u"),  # dimension m
            'state': self.addInPort(name="x")  # dimension n
        }

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
        _, current_time, data = self.state.get()
        current_time += self.elapsed

        # Received a new event, so start processing it
        if self.inPorts['control_action'] in inputs:
            # receives an np.array() as many rows as states
            # and as many columns as polinomial coeffs.
            data[0] = [
                pad_zeros(ui.tolist())
                for ui in inputs[self.inPorts['control_action']]
            ]
            if data[1] is not None:
                data[1] = [advance_time(pol, self.elapsed) for pol in data[1]]
        if self.inPorts['state'] in inputs:
            # receives an np.array() as many rows as states
            # and as many columns as polinomial coeffs.
            data[1] = inputs[self.inPorts['state']]
            if data[0] is not None:
                data[0] = [advance_time(pol, self.elapsed) for pol in data[0]]

        if any(d is None for d in data):
            sigma = INFINITY
        else:
            sigma = 0.0

        if (self.debug):
            print(
                "t: {} s, Atomic name: {}, External Transition Function"
                .format(current_time, self.name)
            )

        return DynamicsFunctionState(sigma, current_time, data)

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
            print(
                "t: {} s, Atomic name: {}, Internal Transition Function"
                .format(current_time, self.name)
            )

        return DynamicsFunctionState(sigma, current_time, data)

    def outputFnc(self):
        """
        Output Function.
        """
        if len(self.outputs_queue) == 0:
            _, _, data = self.state.get()
            u = data[0]  # noqa
            x = data[1]  # noqa

            xdot = self.vector_field(x, u)

            if len(self.outPorts) == 1:
                xdot = [xdot]

            # outputs one polynomial per output port
            self.outputs_queue = [
                {port: var} for var, port in zip(xdot, self.outPorts.values())
                ]

        return self.outputs_queue.pop()

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _, _ = self.state.get()
        return sigma

    def vector_field(self, x, u):
        #
        #  compute f(x, u) here
        #
        return None
