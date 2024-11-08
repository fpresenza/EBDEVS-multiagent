import numpy as np

from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY

from uvnpy.distances.control import (
    RigidityMaintenance,
    CollisionAvoidance
)

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
            subframework,
            external_action
            ):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue, subframework, external_action)

    def set(self, sigma, tvalue, subframework, external_action):
        self._sigma  = sigma
        self._tvalue = tvalue
        self._subframework = subframework
        self._externally_commanded_action = external_action

    def get(self):
        return self._sigma, self._tvalue, self._subframework, self._externally_commanded_action


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
            subframework={},
            external_action=np.zeros((2, 1), dtype=float)
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

        self.collision = CollisionAvoidance(power=2.0)

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, subframework, external_action = self.state.get()
        current_time += self.elapsed    # NOTE: self.elapsed is always zero
        sigma = sigma - self.elapsed    # holds last status

        if self.in_kalman_intpos in inputs: # if data arrives through port in_kalman_intpos
            subframework[self.robot_id] = (inputs[self.in_kalman_intpos], 0)
        elif self.in_handler_extpos in inputs: # if ext pos arrives through port IN_handler
            node_id, external_position, hops, _ = inputs[self.in_handler_extpos]
            subframework[node_id] = (external_position, hops)
        elif self.in_handler_extact in inputs: # if ext action arrives through port IN_handler
            _, external_action = inputs[self.in_handler_extact]
            external_action += external_action

        if (self.debug):
            print(
                "t: {:.2f} s, Atomic name: {}, External Transition Function"
                .format(current_time, self.name)
            )

        return ControllerState(sigma, current_time, subframework, external_action)
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time, subframework, external_action = self.state.get()
        current_time += sigma

        if len(self.outputs_queue) == 0:
            sigma = self.period
            subframework = {}
            external_action[:] = 0.0
        else:
            sigma = 0.0

        if (self.debug):
            print("t: {:.2f} s, Atomic name: {}, Internal Transition Function".format(current_time,self.name))

        return ControllerState(sigma, current_time, subframework, external_action) 
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        if len(self.outputs_queue) == 0:
            sigma, current_time, subframework, external_action = self.state.get()
            own_action, others_action = self.control_action(subframework, external_action)
            self.outputs_queue.append({self.out_handler_intact: others_action})
            self.outputs_queue.append({self.out_dynamics_intact: own_action})

        return self.outputs_queue.pop()

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _, _, _ = self.state.get()
        return max(sigma, 0.0)

    def __lt__(self, other):
        return self.name < other.name

    def control_action(self, subframework, external_action):
        """Compute control action"""
        if self.robot_id in subframework:
            position = subframework[self.robot_id][0]

            u_target = np.zeros((2, 1), dtype=float)
            obstacles = np.array([
                p.ravel() for p, hops in subframework.values() if hops == 1
            ])
            if len(obstacles) > 0:
                u_collision = 20000.0 * self.collision.update(
                    position.ravel(), obstacles
                ).reshape(-1, 1)
            else:
                u_collision = np.zeros((2, 1), dtype=float)

            subframeworks_ids, subframework_positions = list(zip(*subframework.items()))
            rigidity_actions = np.zeros((len(subframework), 2), dtype=float)
            rigidity_actions = {
                node_id: action.reshape(-1, 1)
                for node_id, action in zip(subframeworks_ids, rigidity_actions)
            }

            u_rigidity = rigidity_actions.pop(self.robot_id)
            u_rigidity += external_action 
            
            own_action = u_target + u_collision + u_rigidity
            others_action = rigidity_actions
        else:
            own_action = np.zeros((2, 1), dtype=float)
            others_action = {
                node_id: np.zeros((2, 1), dtype=float)
                for node_id in subframework if node_id != self.robot_id
            }

        return own_action, others_action
