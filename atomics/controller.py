import numpy as np
from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY

# do not reset random seed
np.random.seed(0)


class ControllerState:
    """
    Encapsulates the system's state
    """

    def __init__(
            self, 
            sigma=0.1, 
            tvalue=0.0, 
            last_position=np.array([0.0, 0.0]), 
            external_action=np.array([0.0, 0.0])
        ):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue, last_position, external_action)

    def set(self, sigma, tvalue, last_position, external_action):
        self._sigma  = sigma
        self._tvalue = tvalue
        self._last_position = last_position
        self._external_action = external_action

    def get(self):
        return self._sigma, self._tvalue, self._last_position, self._external_action


class Controller(AtomicDEVS):
    def __init__(self, robot_id, config, name='Controller', debug=False):
        """Atomic model for the rigidity maintenance controller"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        self.robot_id = robot_id    # Robot identifier
        self.period = config['period']

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.state = ControllerState(
            sigma=self.period,   # waits till first token
            tvalue=0.0, 
            last_position=None, 
            external_action=np.zeros(2, dtype=float)
        )
        self.subframework = {}

        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        self.debug = debug

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.out_kalman_intact   = self.addOutPort(name="out_kalman_intact")
        self.out_handler_intact  = self.addOutPort(name="out_handler_intact")
        self.out_dynamics_intact = self.addOutPort(name="out_dynamics_intact")
        
        self.in_kalman_intpos   = self.addInPort(name="in_kalman_intpos")
        self.in_handler_extpos  = self.addInPort(name="in_handler_extpos")
        self.in_handler_extact  = self.addInPort(name="in_handler_extact")
        
        self.first_control_call = True

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, last_position, external_action = self.state.get()
        current_time += self.elapsed

        if self.in_kalman_intpos in inputs: # if data arrives through port in_kalman_intpos
            last_position = inputs[self.in_kalman_intpos]
            sigma = sigma - self.elapsed # holds last status
        elif self.in_handler_extpos in inputs: # if ext pos arrives through port IN_handler
            node_id, external_position = inputs[self.in_handler_extpos]
            self.subframework[node_id] = external_position
        elif self.in_handler_extact in inputs: # if ext action arrives through port IN_handler
            external_action += inputs[self.in_handler_extact]
            sigma = sigma - self.elapsed # holds last status

        if (self.debug):
            print(
                "t: {:.2f} s, Atomic name: {}, External Transition Function"
                .format(current_time, self.name)
            )

        return ControllerState(sigma, current_time, last_position, external_action) 
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time, last_position, external_action = self.state.get()
        current_time += sigma
        self.action.pop()
        if len(self.action) == 0:
            sigma = self.period
            self.first_control_call = True
            external_action = 0
        else:
            sigma = 0

        if (self.debug):
            print("t: {:.2f} s, Atomic name: {}, Internal Transition Function".format(current_time,self.name))

        return ControllerState(sigma,current_time,last_position, external_action) 
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        sigma, current_time, last_position, external_action = self.state.get()
        if self.first_control_call:
            internal_action, external_action = self.control_action(last_position, external_action)
            self.action = [
                {self.out_kalman_intact: internal_action}, 
                {self.out_handler_intact: external_action},
                {self.out_dynamics_intact: internal_action}
            ]
            self.first_control_call = False

        return self.action[-1]

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _, _, _ = self.state.get()
        return sigma

    def __lt__(self, other):
        return self.name < other.name

    def control_action(self, position, external_action):
        """Compute control action"""
        _, current_time, _, _ = self.state.get()
        x = 0.0
        y = 0.0
        # the list of outputs to be returned
        if position is None: # if Kalman filter has not yet sent an estimation of position
            # return np.array([0.0, 0.0]), {'2': np.array([-4.0, 4.0])}
            return np.array([0.0, 0.0]), {}
        else:
            # return np.array([0.0, 0.0]), {}
            return np.array([x, y]), {}

