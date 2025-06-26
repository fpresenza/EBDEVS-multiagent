#!/usr/bin/env python
# -*- coding: utf-8 -*-

from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY

from multirobot_ebdevs.atomics.integrators.qss1tools import (
    advance_time_q, pad_zeros_q
)

#################################
#  Dynamics Function atomic model
#################################


class DynamicsFunctionState:
    """
    Encapsulates the system's state
    """

    def __init__(self, sigma, tvalue, x, u):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue, x, u)

    def set(self, sigma, tvalue, x, u):
        self._sigma = sigma
        self._tvalue = tvalue
        self._x = x
        self._u = u

    def get(self):
        return self._sigma, self._tvalue, self._x, self._u


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
            x=None,
            u=None
        )

        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.inPorts = {
            'input': self.addInPort(name="input"),  # dimension m
            'state': self.addInPort(name="state")  # dimension n
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
        _, current_time, x, u = self.state.get()
        current_time += self.elapsed
        sigma = INFINITY

        # Received a new event, so start processing it
        if self.inPorts['input'] in inputs:
            # receives an np.array() as many rows as states
            # and as many columns as polinomial coeffs.
            u = [
                pad_zeros_q(ui.tolist())
                for ui in inputs[self.inPorts['input']]
            ]
            if x is not None:
                x = [advance_time_q(poly, self.elapsed) for poly in x]
                sigma = 0.0

        if self.inPorts['state'] in inputs:
            # receives an np.array() as many rows as states
            # and as many columns as polinomial coeffs.
            x = inputs[self.inPorts['state']]
            if u is not None:
                u = [advance_time_q(poly, self.elapsed) for poly in u]
                sigma = 0.0

        if (self.debug):
            print(
                "t: {} s, Atomic name: {}, External Transition Function"
                .format(current_time, self.name)
            )

        return DynamicsFunctionState(sigma, current_time, x, u)

    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time, x, u = self.state.get()
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

        return DynamicsFunctionState(sigma, current_time, x, u)

    def outputFnc(self):
        """
        Output Function.
        """
        if len(self.outputs_queue) == 0:
            _, _, x, u = self.state.get()

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
        sigma, _, _, _ = self.state.get()
        return sigma

    def vector_field(self, x, u):
        #
        #  compute f(x, u) here
        #
        return None
