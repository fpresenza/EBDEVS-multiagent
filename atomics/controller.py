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
        self.out_kalman_intact   = self.addOutPort(name="out_kalman_intact")
        self.out_handler_intact  = self.addOutPort(name="out_handler_intact")
        self.out_physics_intact = self.addOutPort(name="out_physics_intact")
        
        self.in_kalman_intpos   = self.addInPort(name="in_kalman_intpos")
        self.in_handler_extpos  = self.addInPort(name="in_handler_extpos")
        self.in_handler_extact  = self.addInPort(name="in_handler_extact")
        
        self.first_control_call = True

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, last_position, ext_action = self.state.get()
        current_time += self.elapsed

        if self.in_kalman_intpos in inputs: # if data arrives through port in_kalman_intpos
            last_position = inputs[self.in_kalman_intpos]
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
            int_action, ext_action = self.control_action(last_position, ext_action)
            self.action = [
                {self.out_kalman_intact: int_action}, 
                {self.out_handler_intact: ext_action},
                {self.out_physics_intact: int_action}
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
            return np.array([0.0, 0.0]), {}
        else:
            return np.array([0.0, 0.0]), {}

