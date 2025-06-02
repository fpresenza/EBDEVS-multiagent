#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

from pypdevs.DEVS import AtomicDEVS


class InteroceptiveSensorState:
    """
    Encapsulates the system's state
    """

    def __init__(
            self,
            sigma,
            tvalue,
            internal_state
            ):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue, internal_state)

    def set(self, sigma, tvalue, internal_state):
        self._sigma = sigma
        self._tvalue = tvalue
        self._internal_state = internal_state

    def get(self):
        return self._sigma, self._tvalue, self._internal_state


class InteroceptiveSensor(AtomicDEVS):
    def __init__(self, config, name='InteroceptiveSensor', debug=False):
        """Atomic model for an interoceptive sensor"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        self.noise_mean = np.array(config['bias'])
        self.noise_covariance = np.array(config['covariance'])
        self.period = config['period']

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.state = InteroceptiveSensorState(
            sigma=self.period,   # waits till first token
            tvalue=0.0,
            internal_state=None
        )
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        self.debug = debug

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.inPorts = {
            'internal_state':
            self.addInPort(name="in_internal_state")
        }
        self.outPorts = {
            'measurement':
            self.addOutPort(name="out_measurement")
        }

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, _ = self.state.get()
        current_time += self.elapsed

        internal_state = inputs[self.inPorts['internal_state']]
        sigma = sigma - self.elapsed  # holds last status

        if (self.debug):
            print(
                "t: {:.2f} s, Atomic name: {}, External Transition Function"
                .format(current_time, self.name)
            )

        return InteroceptiveSensorState(sigma, current_time, internal_state)

    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time, internal_state = self.state.get()
        current_time += sigma

        sigma = self.period

        if (self.debug):
            print(
                "t: {:.2f} s, Atomic name: {}, Internal Transition Function"
                .format(current_time, self.name)
            )

        return InteroceptiveSensorState(sigma, current_time, internal_state)

    def outputFnc(self):
        """
        Output Funtion.
        """
        sigma, current_time, internal_state = self.state.get()

        measurement = None
        if internal_state is not None:
            measurement = self.compute_measurement(
                current_time, internal_state
            )

        return {self.outPorts['measurement']: measurement}

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _, _ = self.state.get()
        return max(sigma, 0.0)

    def __lt__(self, other):
        return self.name < other.name

    def compute_measurement(self, current_time, internal_state):
        #
        #  compute measurement here
        #
        return None
