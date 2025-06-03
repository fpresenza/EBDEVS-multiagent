#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

from pypdevs.DEVS import AtomicDEVS


class ExteroceptiveSensorState:
    """
    Encapsulates the system's state
    """

    def __init__(
            self,
            sigma,
            tvalue,
            external_state
            ):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue, external_state)

    def set(self, sigma, tvalue, external_state):
        self._sigma = sigma
        self._tvalue = tvalue
        self._external_state = external_state

    def get(self):
        return self._sigma, self._tvalue, self._external_state


class ExteroceptiveSensor(AtomicDEVS):
    def __init__(self, config, name='ExteroceptiveSensor', debug=False):
        """Atomic model for an exteroceptive sensor"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        self.noise_mean = np.array(config['bias'])
        self.noise_covariance = np.array(config['covariance'])
        self.period = config['period']

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.state = ExteroceptiveSensorState(
            sigma=self.period,   # waits till first token
            tvalue=0.0,
            external_state=None
        )
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        self.debug = debug

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.outPorts = {
            'measurement':
            self.addOutPort(name="out_measurement")
        }

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    # def extTransition(self, inputs):
    #     """
    #     External Transition Function.
    #     """
    #     sigma, current_time, _ = self.state.get()
    #     current_time += self.elapsed    # NOTE: self.elapsed is always zero

    #     external_state = inputs[self.inPorts['external_state']]
    #     sigma = sigma - self.elapsed  # holds last status

    #     if (self.debug):
    #         print(
    #             "t: {:.2f} s, Atomic name: {}, External Transition Function"
    #             .format(current_time, self.name)
    #         )

    #     return ExteroceptiveSensorState(sigma, current_time, external_state)

    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time, external_state = self.state.get()
        current_time += sigma
        sigma = self.period

        if (self.debug):
            print(
                "t: {:.2f} s, Atomic name: {}, Internal Transition Function"
                .format(current_time, self.name)
            )

        return ExteroceptiveSensorState(sigma, current_time, external_state)

    def outputFnc(self):
        """
        Output Funtion.
        """
        sigma, current_time, external_state = self.state.get()

        measurement = self.compute_measurement(current_time)

        return {self.outPorts['measurement']: measurement}

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _, _ = self.state.get()
        return sigma

    def __lt__(self, other):
        return self.name < other.name
