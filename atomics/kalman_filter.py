import numpy as np
from dataclasses import dataclass
from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY



class KalmanFilterState:
    """
    Encapsulates the system's state
    """

    def __init__(self, sigmaval=0.1, tval=0.0, dataval=[]):
        """
        Constructor (parameterizable).
        """
        self.set(sigmaval, tval, dataval)

    def set(self, sigmavalue, tvalue, datavalue):
        self._sigma  = sigmavalue
        self._tvalue = tvalue
        self._data   = datavalue

    def get(self):
        return self._sigma, self._tvalue, self._data


class KalmanFilter(AtomicDEVS):
    def __init__(self,robot_id,name=None):
        """Atomic model for the kalman filter"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        self.robot_id = robot_id    # Robot identifier
        # self.status = []          # TODO

        # STATE:
        #  Define 'state' attribute (initial sate):
        _time0  = 0.0
        _sigma0 = INFINITY # waits till firts token
        _data0  = []
        self.state = KalmanFilterState(_sigma0,_time0,_data0) 
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.OUT_control = self.addOutPort(name="control_out")
        self.OUT_handler   = self.addOutPort(name="handler_out")
        #
        self.IN_control  = self.addInPort(name="control_in")
        self.IN_handler    = self.addInPort(name="handler_in")

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, data = self.state.get()
        current_time += self.elapsed

        if self.IN_control in inputs: # if data arrives through port IN_control
            control_action = inputs[self.IN_control]
            new_position = self.prediction_step(control_action) # events list
            data = [(self.OUT_control, new_position), (self.OUT_handler, new_position)]
            sigma = 0 # holds last status
        elif self.IN_handler in inputs: # if token arrives through port IN_control
            neighbor_data = inputs[self.IN_handler]
            new_position = self.update_step(neighbor_data) # events list
            data = [(self.OUT_control, new_position)]
            sigma = 0 # holds last status

        return KalmanFilterState(sigma, current_time, data) 
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        _, current_time, data = self.state.get()
        data.pop()
        if len(data) == 0:
            sigma = INFINITY
        else:
            sigma = 0
        return KalmanFilterState(sigma,current_time,data) 
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        sigma, current_time, data = self.state.get()
        return data[-1]

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _, _ = self.state.get()
        return sigma

    def prediction_step(self, token):
        """Prediction step based on control actions"""
        # the list of outputs to be returned
        return new_position

    def update_step(self, token):
        """Update step based on distance measurements with neighbors"""
        # the list of outputs to be returned
        return new_position


