import numpy as np
from dataclasses import dataclass
import copy
from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY


@dataclass
class Token(object):
    creator: str                # The robot that created it
    kind: str                   # action or state
    order: int                  # A counter to differentiate tokens
    data: object                # The data it carries
    hops_to_target: int         # The number of hops it must travel
    hops_travelled: int = 0     # The number of hops it has travelled


class TokenHandlerState:
    """
    Encapsulates the system's state
    """
    def __init__(self, sigma, tvalue, history, position):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue, history, position)

    def set(self, sigma, tvalue, history, position):
        self._sigma = sigma
        self._tvalue = tvalue
        self._history = history
        self._state = position

    def get(self):
        return self._sigma, self._tvalue, self._history, self._state


class TokenHandler(AtomicDEVS):
    def __init__(
            self, 
            robot_id,
            config,
            name='TokenHandler', 
            debug=False
            ):
        """Atomic model for the toking handling protocol"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        self.robot_id = robot_id    # Robot identifier
        self.action_extent = config['action']      # The robot's action extent
        self.state_extent = config['state']        # The robot's state extent
        # self.status = []          # TODO
        self.debug = debug

        # Dictionaries as records of tokens received

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.state = TokenHandlerState(
            sigma=INFINITY, 
            tvalue=0.0, 
            history={
                'out': {'action': 0, 'state': 0}, 
                'in': {'action': {}, 'state': {}}
            },
            position=None,
        ) 
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.out_router_token      = self.addOutPort(name="out_router_token")
        self.out_controller_extpos = self.addOutPort(name="out_controller_extpos")
        self.out_controller_extact = self.addOutPort(name="out_controller_extact")
        self.out_kalman_extpos     = self.addOutPort(name="out_kalman_extpos")
        #
        self.in_router_token       = self.addInPort(name="in_router_token")
        self.in_controller_intact  = self.addInPort(name="in_controller_intact")
        self.in_kalman_intpos      = self.addInPort(name="in_kalman_intpos")

        self.outputs_queue = []

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, history, position = self.state.get()
        current_time += self.elapsed

        if self.in_router_token in inputs:    # if token arrives through port in_router_token
            token, distance_measurement = inputs[self.in_router_token]
            history, msgs = self.handle_received_token(
                history,
                token,
                distance_measurement
            )

            if len(msgs) > 0:    # else pass, nothing to send
                self.outputs_queue += msgs
                sigma = 0.0
    
            if (self.debug):
                print(
                    "t: {} s, Atomic name: {}, External Transition Function, token: {} from Router"
                    .format(current_time, self.name, token)
                )

        elif self.in_controller_intact in inputs: # if data arrives through port in_controller_intact
            action_token = Token(
                creator=self.parent.name,
                kind='action',
                order=history['out']['action'],
                data=inputs[self.in_controller_intact],
                hops_to_target=self.action_extent,
                hops_travelled=0
            )
            history['out']['action'] += 1
            self.outputs_queue.append({self.out_router_token: action_token})

            if position is not None:
                state_token = Token(
                    creator=self.parent.name,
                    kind='state',
                    order=history['out']['state'],
                    data=position,
                    hops_to_target=self.state_extent,
                    hops_travelled=0
                )
                history['out']['state'] += 1
                self.outputs_queue.append({self.out_router_token: state_token})

            sigma = 0.0

            if (self.debug):
                print(
                    "t: {} s, Atomic name: {}@{}, External Transition Function, token: {} from Controller"
                    .format(current_time, self.name, self.parent.name, action_token)
                )

        elif self.in_kalman_intpos in inputs:   # if data arrives through port in_kalman_intpos
            position = inputs[self.in_kalman_intpos]

            if (self.debug):
                print(
                    "t: {} s, Atomic name: {}@{}, External Transition Function, token: {} from Kalman"
                    .format(current_time, self.name, self.parent.name, token)
                )

        return TokenHandlerState(sigma, current_time, history, position) 
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        _, current_time, history, position = self.state.get()
        
        if len(self.outputs_queue) == 0:
            sigma = INFINITY
        else:
            sigma = 0.0

        if (self.debug):
            print(
                "t: {} s, Atomic name: {}@{}, Internal Transition Function"
                .format(current_time, self.name, self.parent.name)
            )

        return TokenHandlerState(sigma, current_time, history, position)
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        _, current_time, _, _ = self.state.get()

        if (self.debug):
            print(
                "t: {} s, Atomic name: {}@{}, Output Function, data: {}"
                .format(current_time, self.name, self.parent.name, self.outputs_queue[0])
            )
        
        return self.outputs_queue.pop(0)
    

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

    def handle_received_token(self, history, token, distance):
        """Decide what to do with the received token"""
        msgs = []

        if token.creator == self.robot_id:
            # do nothing if this robot is the creator
            pass
        else:
            # update the number of traversed hops
            token = copy.deepcopy(token)
            token.hops_travelled += 1

            # check if retransmission is needed
            if token.hops_travelled < token.hops_to_target:
                msgs.append({self.out_router_token: token})

            try:
                # gets order from received dictionary
                last_order = history['in'][token.kind][token.creator]
            except KeyError:
                # first time received
                last_order = -1

            # check if token is newer than last received
            if token.order > last_order:
                history['in'][token.kind][token.creator] = token.order
                # check if token is of kind action
                if token.kind == 'action':
                    try:
                        # check if there is data for this robot
                        data = (token.creator, token.data[self.robot_id])
                        # send data to controller
                        msgs.append({self.out_controller_extact: data})
                    except KeyError:
                        pass
                # check if token is of kind state
                elif token.kind == 'state':
                    # check if token creator is within action extent
                    if token.hops_travelled <= self.action_extent:
                        # send data to controller
                        data = (token.creator, token.data)
                        msgs.append({self.out_controller_extpos: data})
                        if token.hops_travelled == 1:
                            # send data to positioning system
                            data += (distance,)
                            msgs.append({self.out_kalman_extpos: data})

        return history, msgs


