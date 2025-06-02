#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import pi, sin
import random

# Import code for DEVS model representation:
from pypdevs.DEVS import AtomicDEVS, DEVSException


class PeriodicGeneratorState:
    """
    Encapsulates the system's state
    """

    def __init__(self, val="LO"):
        """
        Constructor (parameterizable).
        """
        self.set(val)

    def set(self, value="LO"):
        self._value = value

    def get(self):
        return self._value


class PeriodicGenerator(AtomicDEVS):
    """
    A pulse generator
    """

    def __init__(self, name=None, peri=1):
        """
        Constructor (parameterizable).
        """
        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.peri = peri
        self.state = PeriodicGeneratorState("LO")

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

    def __lt__(self, other):
        return self.name < other.name

    def intTransition(self):
        """
        Internal Transition Function.
        The policeman works forever, so only one mode.
        """

        state = self.state.get()

        if state == "LO":
            return PeriodicGeneratorState("HI")
            # return self.state.set("HI")
        elif state == "HI":
            return PeriodicGeneratorState("LO")
            # return self.state.set("LO")
        else:
            raise DEVSException(
               "unknown state <%f> in internal transition function"
               % state)

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

        state = self.state.get()

        if state == "LO":
            return {self.OUT: 0}
        elif state == "HI":
            return {self.OUT: 1}
        else:
            raise DEVSException(
                "unknown state <%f> in external transition function"
                % state)

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        return self.peri


class SinusoidalGeneratorState:
    """
    Encapsulates the system's state
    """

    def __init__(self, freq, amp, phi, deltat):
        """
        Constructor (parameterizable).
        """
        self._time = 0
        self.set(freq, amp, phi, deltat)

    def set(self, freq, amp, phi, deltat):
        self._time += deltat
        self._value = amp * sin(2 * pi * freq * self._time + phi)

    def get(self):
        return self._value, self._time


class SinusoidalGenerator(AtomicDEVS):
    """
    A sinusoidal generator
    """

    def __init__(self, name=None, freq=1, amp=1, phi=0, samp=0.1):
        """
        Constructor (parameterizable).
        """
        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.frequency = freq
        self.amplitude = amp
        self.sampling = samp
        self.phase = phi
        self.state = SinusoidalGeneratorState(freq, amp, phi, 0)

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

    def __lt__(self, other):
        return self.name < other.name

    def intTransition(self):
        """
        Internal Transition Function.
        The policeman works forever, so only one mode.
        """

        _, current_time = self.state.get()
        current_time += self.timeAdvance()
        return SinusoidalGeneratorState(
            self.frequency, self.amplitude, self.phase, current_time
        )
        # return self.state.set(
        #     self.frequency, self.amplitude, self.phase, current_time
        # )

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

        val, _ = self.state.get()
        return {self.OUT: [val, 0.0]}

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        return self.sampling


class SinusoidalGenerator10Hz(SinusoidalGenerator):
    """
    A sinusoidal generator with f = 10Hz
    """

    def __init__(self, name=None, samptime=0.01):
        """
        Constructor (parameterizable).
        """
        # Always call parent class' constructor FIRST:
        SinusoidalGenerator.__init__(
            self, name, freq=10, amp=1, phi=0, samp=samptime
        )

        # STATE:
        #  Define 'state' attribute (initial sate):

        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):


class PulseGeneratorState:
    """
    Encapsulates the system's state
    """

    def __init__(self, val=0, sig=1):
        """
        Constructor (parameterizable).
        """
        self.set(val, sig)

    def set(self, value=0, sigma=10):
        self._value = value
        self._sigma = sigma

    def get(self):
        return self._value, self._sigma


class PulseGenerator(AtomicDEVS):
    """
    A pulse generator
    """

    def __init__(self, name=None, a=1, b=2):
        """
        Constructor (parameterizable).
        """
        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.a = a
        self.b = b
        value = 1
        sigma = random.uniform(a, b)
        self.state = PulseGeneratorState(value, sigma)
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

    def __lt__(self, other):
        return self.name < other.name

    def intTransition(self):
        """
        Internal Transition Function.
        The policeman works forever, so only one mode.
        """

        value, _ = self.state.get()
        sigma = random.uniform(self.a, self.b)
        return PulseGeneratorState(value, sigma)
        # return self.state.set(value, sigma)

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

        value, _ = self.state.get()
        return {self.OUT: value}

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        _, sigma = self.state.get()
        return sigma
