import numpy as np

from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY

from uvnpy.distances.control import RigidityMaintenance
from uvnpy.control.core import CollisionAvoidanceVanishing

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
            obstacles,
            target_position
            ):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue, subframework, external_action, obstacles, target_position)

    def set(self, sigma, tvalue, subframework, external_action, obstacles, target_position):
        self._sigma  = sigma
        self._tvalue = tvalue
        self._subframework = subframework
        self._external_action = external_action
        self._obstacles = obstacles
        self._target_position = target_position

    def get(self):
        return self._sigma, self._tvalue, self._subframework, self._external_action, self._obstacles, self._target_position


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
            obstacles=[],
            target_position=None
        )
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.inPorts = {
            'position': self.addInPort(name="in_position"),
            'other_position': self.addInPort(name="in_other_position"),
            'external_action': self.addInPort(name="in_external_action"),
            'target_position': self.addInPort(name="in_target_position")
        }        
        self.outPorts = {
            'own_action': self.addOutPort(name="out_own_action"),
            'others_actions': self.addOutPort(name="out_others_actions")
        }
            
        self.outputs_queue = []

        self.rigidity = RigidityMaintenance(
            dim=2,
            dmax=config['dmax'][0],
            steepness=config['steepness'],
            threshold=1e-4,
            eigenvalues='all',
            functional='log'
        )
        self.collision = CollisionAvoidanceVanishing(
            power=2.0,
            dmin=1.0,
            dmax=config['dmax'][1]
        )

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, subframework, external_action, obstacles, target_position = self.state.get()
        current_time += self.elapsed    # NOTE: self.elapsed is always zero
        sigma -= self.elapsed    # holds last status

        if self.inPorts['position'] in inputs: # if data arrives through port inPorts['position']
            position = inputs[self.inPorts['position']]
            subframework[self.robot_id] = position.ravel()
       
        elif self.inPorts['other_position'] in inputs: # if ext pos arrives through port IN_handler
            node_id, other_position, hops = inputs[self.inPorts['other_position']]
            subframework[node_id] = other_position.ravel()
            if hops == 1:
                obstacles.append(other_position.ravel())

        elif self.inPorts['external_action'] in inputs: # if ext action arrives through port IN_handler
            _, external_action_term = inputs[self.inPorts['external_action']]
            external_action += external_action_term

        elif self.inPorts['target_position'] in inputs:
            target_position = inputs[self.inPorts['target_position']]

        if (self.debug):
            print(
                "t: {:.2f} s, Atomic name: {}, External Transition Function"
                .format(current_time, self.name)
            )

        return ControllerState(sigma, current_time, subframework, external_action, obstacles, target_position)
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time, subframework, external_action, obstacles, target_position = self.state.get()
        current_time += sigma

        if len(self.outputs_queue) == 0:
            sigma = self.period
            subframework.clear()
            external_action[:] = 0.0
            obstacles.clear()
            # target_position = None
        else:
            sigma = 0.0

        if (self.debug):
            print("t: {:.2f} s, Atomic name: {}, Internal Transition Function".format(current_time,self.name))

        return ControllerState(sigma, current_time, subframework, external_action, obstacles, target_position) 
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        if len(self.outputs_queue) == 0:
            sigma, current_time, subframework, external_action, obstacles, target_position = self.state.get()
            own_action, others_actions = self.control_action(subframework, external_action, obstacles, target_position)
            self.outputs_queue.append({self.outPorts['own_action']: own_action})
            self.outputs_queue.append({self.outPorts['others_actions']: others_actions})

            log = [current_time + sigma, own_action[0][0], own_action[1][0]]
            append_csv_file(self.logpath + 'controller_{}.csv'.format(self.robot_id), log)

        return self.outputs_queue.pop()

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _, _, _, _, _ = self.state.get()
        return max(sigma, 0.0)

    def __lt__(self, other):
        return self.name < other.name

    def control_action(self, subframework, external_action, obstacles, target_position):
        """Compute control action"""
        if self.robot_id in subframework:
            position = subframework[self.robot_id]

            # target collection
            if target_position is not None:
                r = position.reshape(-1, 1) - target_position
                d = np.sqrt(np.square(r).sum())
                tracking_radius = 20.0    # radius
                forget_radius = 100.0     # radius
                v_collect_max = 2.0
                if d < tracking_radius:
                    v_collect = v_collect_max
                elif d < forget_radius:
                    factor = (forget_radius - d)/(forget_radius - tracking_radius)
                    v_collect = v_collect_max * factor
                else:
                    v_collect = 0.0
                target_action = - v_collect * r / d
            else:
                target_action = np.zeros((2, 1), dtype=float)

            # obstacle avoidance
            if len(obstacles) > 0:
                collision_action = 0.5 * self.collision.update(
                    position, obstacles
                ).reshape(-1, 1)
            else:
                collision_action = np.zeros((2, 1), dtype=float)

            # rigidity maintenance
            rigidity_action = external_action
            others_rigidity = {}

            if len(subframework) > 1:
                subframework_ids, subframework_positions = list(zip(*subframework.items()))
                subframework_actions = 0.75 * self.rigidity.update(np.array(subframework_positions))
                others_rigidity = {
                    node_id: action.reshape(-1, 1)
                    for node_id, action in zip(subframework_ids, subframework_actions)
                }
                rigidity_action += others_rigidity.pop(self.robot_id)

            own_action = (target_action + collision_action + rigidity_action) * 0.5
            others_actions = others_rigidity
        else:
            own_action = np.zeros((2, 1), dtype=float)
            others_actions = {
                node_id: np.zeros((2, 1), dtype=float)
                for node_id in subframework if node_id != self.robot_id
            }

        return own_action, others_actions
