import numpy as np

from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY

from uvnpy.distances.control import (
    RigidityMaintenance,
    CollisionAvoidance
)

from utils.files import append_csv_file


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
            external_action,
            obstacles
            ):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue, subframework, external_action, obstacles)

    def set(self, sigma, tvalue, subframework, external_action, obstacles):
        self._sigma  = sigma
        self._tvalue = tvalue
        self._subframework = subframework
        self._external_action = external_action
        self._obstacles = obstacles

    def get(self):
        return self._sigma, self._tvalue, self._subframework, self._external_action, self._obstacles


class Controller(AtomicDEVS):
    def __init__(
            self,
            robot_id,
            config,
            name='Controller',
            logpath='./',
            debug=False):
        """Atomic model for the rigidity maintenance controller"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        self.robot_id = robot_id    # Robot identifier
        self.logpath = logpath
        self.period = config['period']
        self.debug = debug

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.state = ControllerState(
            sigma=self.period,   # waits till first token
            tvalue=0.0, 
            subframework={},
            external_action=np.zeros((2, 1), dtype=float),
            obstacles=[]
        )
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

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
        self.rigidity = RigidityMaintenance(
            dim=2,
            dmax=config['dmax'],
            steepness=config['steepness'],
            eigenvalues='all',
            functional='log'
        )

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, subframework, external_action, obstacles = self.state.get()
        current_time += self.elapsed    # NOTE: self.elapsed is always zero
        sigma -= self.elapsed    # holds last status

        if self.in_kalman_intpos in inputs: # if data arrives through port in_kalman_intpos
            position = inputs[self.in_kalman_intpos]
            subframework[self.robot_id] = position.ravel()
        elif self.in_handler_extpos in inputs: # if ext pos arrives through port IN_handler
            node_id, external_position, hops = inputs[self.in_handler_extpos]
            subframework[node_id] = external_position.ravel()
            if hops == 1:
                obstacles.append(external_position.ravel())
        elif self.in_handler_extact in inputs: # if ext action arrives through port IN_handler
            _, external_action_term = inputs[self.in_handler_extact]
            external_action += external_action_term

        if (self.debug):
            print(
                "t: {:.2f} s, Atomic name: {}, External Transition Function"
                .format(current_time, self.name)
            )

        return ControllerState(sigma, current_time, subframework, external_action, obstacles)
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time, subframework, external_action, obstacles = self.state.get()
        current_time += sigma

        if len(self.outputs_queue) == 0:
            sigma = self.period
            subframework.clear()
            external_action[:] = 0.0
            obstacles.clear()
        else:
            sigma = 0.0

        if (self.debug):
            print("t: {:.2f} s, Atomic name: {}, Internal Transition Function".format(current_time,self.name))

        return ControllerState(sigma, current_time, subframework, external_action, obstacles) 
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        if len(self.outputs_queue) == 0:
            _, current_time, subframework, external_action, obstacles = self.state.get()
            own_action, others_actions = self.control_action(subframework, external_action, obstacles)
            self.outputs_queue.append({self.out_handler_intact: others_actions})
            self.outputs_queue.append({self.out_dynamics_intact: own_action})

            log = [current_time, own_action[0][0], own_action[1][0]]
            append_csv_file(self.logpath + 'controller_{}.csv'.format(self.robot_id), log)

        return self.outputs_queue.pop()

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _, _, _, _ = self.state.get()
        return max(sigma, 0.0)

    def __lt__(self, other):
        return self.name < other.name

    def control_action(self, subframework, external_action, obstacles):
        """Compute control action"""
        if self.robot_id in subframework:
            position = subframework[self.robot_id]

            # target collection
            own_target = np.zeros((2, 1), dtype=float)

            # obstacle avoidance
            if len(obstacles) > 0:
                own_collision = 20000.0 * self.collision.update(
                    position, obstacles
                ).reshape(-1, 1)
            else:
                own_collision = np.zeros((2, 1), dtype=float)

            # rigidity maintenance
            own_rigidity = external_action
            others_rigidity = {}
            if len(subframework) > 1:
                subframework_ids, subframework_positions = list(zip(*subframework.items()))
                subframework_actions = 5.0 * self.rigidity.update(np.array(subframework_positions))
                others_rigidity = {
                    node_id: action.reshape(-1, 1)
                    for node_id, action in zip(subframework_ids, subframework_actions)
                }
                own_rigidity += others_rigidity.pop(self.robot_id)
            
            own_action = own_target + own_collision + own_rigidity
            others_actions = others_rigidity
        else:
            own_action = np.zeros((2, 1), dtype=float)
            others_actions = {
                node_id: np.zeros((2, 1), dtype=float)
                for node_id in subframework if node_id != self.robot_id
            }

        return own_action, others_actions
