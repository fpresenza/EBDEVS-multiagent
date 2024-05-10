import numpy as np
from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY


class ControllerState:
    """
    Encapsulates the system's state
    """

    def __init__(
            self, 
            sigmaval=0.1, 
            tval=0.0, 
            last_position=np.array([0.0, 0.0]), 
            ext_action=np.array([0.0, 0.0])
        ):
        """
        Constructor (parameterizable).
        """
        self.set(sigmaval, tval, dataval)

    def set(self, sigmavalue, tvalue, last_position, control_action):
        self._sigma  = sigmavalue
        self._tvalue = tvalue
        self._last_position = last_position
        self._ext_action = ext_action

    def get(self):
        return self._sigma, self._tvalue, self._last_position, self._ext_action


class Controller(AtomicDEVS):
    def __init__(self,robot_id,name=None):
        """Atomic model for the rigidity maintenance controller"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        self.robot_id = robot_id    # Robot identifier
        self.period = 0.1

        # STATE:
        #  Define 'state' attribute (initial sate):
        _time0  = 0.0
        _sigma0 = self.period # waits till firts token
        _last_position0 = None
        _ext_action0 = np.array([0.0, 0.0])
        self.state = ControllerState(
            _sigma0,
            _time0, 
            _last_position0,
            _ext_action0
        ) 
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.OUT_kalman   = self.addOutPort(name="kalman_out")
        self.OUT_handler  = self.addOutPort(name="handler_out")
        self.OUT_dynamics = self.addOutPort(name="dynamics_out")
        
        self.IN_kalman   = self.addInPort(name="kalman_in")
        self.in_handler_extpos  = self.addInPort(name="in_handler_extpos")
        self.in_handler_extact  = self.addInPort(name="in_handler_extact")
        self.IN_dynamics = self.addInPort(name="dynamics_in")
        
        self.first_control_call = True

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, last_position, ext_action = self.state.get()
        current_time += self.elapsed

        if self.IN_kalman in inputs: # if data arrives through port IN_kalman
            last_position = inputs[self.IN_kalman]
            sigma = sigma - self.elapsed # holds last status
        elif self.in_handler_extpos in inputs: # if ext pos arrives through port IN_handler
            # TODO: agregar position a subframework
        elif self.in_handler_extact in inputs: # if ext action arrives through port IN_handler
            ext_action += inputs[self.in_handler_extact]
            sigma = sigma - self.elapsed # holds last status

        return ControllerState(sigma, current_time, last_position, ext_action) 
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        _, current_time, last_position, _ = self.state.get()
        self.action.pop()
        if len(self.action) == 0:
            sigma = self.period
            self.first_control_call = True
            ext_action = 0
        else:
            sigma = 0
        return ControllerState(sigma,current_time,last_position, ext_action) 
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        sigma, current_time, last_position, ext_action = self.state.get()
        if self.first_control_call:
            new_action = self.control_action(last_position, ext_action)
            self.action = [
                {self.OUT_kalman: new_action}, 
                {self.OUT_handler: new_action},
                {self.OUT_dynamics: new_action}
            ]
            self.first_control_call = False

        return self.action[-1]

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _, _ = self.state.get()
        return sigma

    def control_action(self, position, ext_action):
        """Compute control action"""
        # the list of outputs to be returned
        if position is None:
            control_action = np.array([0.0, 0.0])
        else:
            control_action = np.array([0.0, 0.0])
        return control_action

