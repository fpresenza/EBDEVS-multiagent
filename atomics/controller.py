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
            sigma, 
            tvalue, 
            subframework_state,
            externally_commanded_action
            ):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue, subframework_state, externally_commanded_action)

    def set(self, sigma, tvalue, subframework_state, externally_commanded_action):
        self._sigma  = sigma
        self._tvalue = tvalue
        self._subframework_state = subframework_state
        self._externally_commanded_action = externally_commanded_action

    def get(self):
        return self._sigma, self._tvalue, self._subframework_state, self._externally_commanded_action


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
            subframework_state={self.robot_id: None},
            externally_commanded_action=np.zeros(2, dtype=float)
        )
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        self.debug = debug

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.out_handler_intact  = self.addOutPort(name="out_handler_intact")
        self.out_dynamics_intact = self.addOutPort(name="out_dynamics_intact")
        
        self.in_kalman_intpos   = self.addInPort(name="in_kalman_intpos")
        self.in_handler_extpos  = self.addInPort(name="in_handler_extpos")
        self.in_handler_extact  = self.addInPort(name="in_handler_extact")
        
        self.outputs_queue = []

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, subframework_state, externally_commanded_action = self.state.get()
        current_time += self.elapsed

        if self.in_kalman_intpos in inputs: # if data arrives through port in_kalman_intpos
            subframework_state[self.robot_id] = inputs[self.in_kalman_intpos]
            sigma = sigma - self.elapsed # holds last status
        elif self.in_handler_extpos in inputs: # if ext pos arrives through port IN_handler
            node_id, external_position = inputs[self.in_handler_extpos]
            subframework_state[node_id] = external_position
        elif self.in_handler_extact in inputs: # if ext action arrives through port IN_handler
            externally_commanded_action += inputs[self.in_handler_extact]
            sigma = sigma - self.elapsed # holds last status

        if (self.debug):
            print(
                "t: {:.2f} s, Atomic name: {}, External Transition Function"
                .format(current_time, self.name)
            )

        return ControllerState(sigma, current_time, subframework_state, externally_commanded_action)
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time, subframework_state, externally_commanded_action = self.state.get()
        current_time += sigma

        if len(self.outputs_queue) == 0:
            sigma = self.period
            externally_commanded_action = np.zeros(2, dtype=float)
        else:
            sigma = 0.0

        if (self.debug):
            print("t: {:.2f} s, Atomic name: {}, Internal Transition Function".format(current_time,self.name))

        return ControllerState(sigma, current_time, subframework_state, externally_commanded_action) 
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        sigma, current_time, subframework_state, externally_commanded_action = self.state.get()
        if len(self.outputs_queue) == 0:
            own_action, others_action = self.control_action(subframework_state, externally_commanded_action)
            self.outputs_queue = [
                {self.out_handler_intact: others_action},
                {self.out_dynamics_intact: own_action}
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

    def __lt__(self, other):
        return self.name < other.name

    def control_action(self, subframework_state, externally_commanded_action):
        """Compute control action"""
        own_action = np.zeros(2, dtype=float) + externally_commanded_action
        others_action = {
            node_id: np.zeros(2, dtype=float)
            for node_id in subframework_state if node_id != self.robot_id
        }
        return own_action, others_action
