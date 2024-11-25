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
    def __init__(self, sigma, tvalue, history, token_queue, position):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue, history, token_queue, position)

    def set(self, sigma, tvalue, history, token_queue, position):
        self._sigma = sigma
        self._tvalue = tvalue
        self._history = history
        self._tokens = token_queue
        self._state = position

    def get(self):
        return self._sigma, self._tvalue, self._history, self._tokens, self._state


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
            token_queue=[],
            position=None,
        ) 
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        #
        self.inPorts = {
            'token': self.addInPort(name="in_token"),
            'others_actions': self.addInPort(name="in_others_actions"),
            'position': self.addInPort(name="in_position")
        }

        self.outPorts = {
            'token': self.addOutPort(name="out_token"),
            'other_position': self.addOutPort(name="out_other_position"),
            'neighbors_positions': self.addOutPort(name="out_neighbors_positions"),
            'external_action': self.addOutPort(name="out_external_action"),
        }

        self.outputs_queue = []

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, history, token_queue, position = self.state.get()
        current_time += self.elapsed

        if self.inPorts['token'] in inputs:    # if token arrives through port self.inPorts['token']
            token_list, distance_measurement = inputs[self.inPorts['token']]

            for token in token_list:
                history, broadcast, response = self.handle_received_token(
                    history,
                    token,
                    distance_measurement
                )
                token_queue += broadcast

                if len(response) > 0:    # else pass, nothing to send
                    self.outputs_queue += response
                    sigma = 0.0
    
            if (self.debug):
                print(
                    "t: {} s, Atomic name: {}, External Transition Function, token: {} from Router"
                    .format(current_time, self.name, token)
                )

        elif self.inPorts['others_actions'] in inputs: # if data arrives through port inPorts['others_actions']
            action_token = Token(
                creator=self.parent.name,
                kind='action',
                order=history['out']['action'],
                data=inputs[self.inPorts['others_actions']],
                hops_to_target=self.action_extent,
                hops_travelled=0
            )
            history['out']['action'] += 1
            token_queue += [action_token]

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
                position = None
                token_queue += [state_token]

            self.outputs_queue.append({self.outPorts['token']: token_queue})
            token_queue = []
            sigma = 0.0

            if (self.debug):
                print(
                    "t: {} s, Atomic name: {}@{}, External Transition Function, token: {} from Controller"
                    .format(current_time, self.name, self.parent.name, action_token)
                )

        elif self.inPorts['position'] in inputs:   # if data arrives through port inPorts['position']
            position = inputs[self.inPorts['position']]

            if (self.debug):
                print(
                    "t: {} s, Atomic name: {}@{}, External Transition Function, token: {} from Kalman"
                    .format(current_time, self.name, self.parent.name, token)
                )

        return TokenHandlerState(sigma, current_time, history, token_queue, position) 
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        _, current_time, history, token_queue, position = self.state.get()
        
        if len(self.outputs_queue) == 0:
            sigma = INFINITY
        else:
            sigma = 0.0

        if (self.debug):
            print(
                "t: {} s, Atomic name: {}@{}, Internal Transition Function"
                .format(current_time, self.name, self.parent.name)
            )

        return TokenHandlerState(sigma, current_time, history, token_queue, position)
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        _, current_time, _, _, _ = self.state.get()

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
        sigma, _, _, _, _ = self.state.get()
        return max(sigma, 0.0)
    
    def __lt__(self, other):
        return self.name < other.name

    def handle_received_token(self, history, token, distance):
        """Decide what to do with the received token"""
        response = []
        broadcast = []
        
        if token.creator == self.robot_id:
            # do nothing if this robot is the creator
            pass
        else:
            # update the number of traversed hops
            token = copy.deepcopy(token)
            token.hops_travelled += 1

            try:
                # gets order from received dictionary
                last_order = history['in'][token.kind][token.creator]
            except KeyError:
                # first time received
                last_order = -1

            # check if token is newer than last received
            if token.order > last_order:
                history['in'][token.kind][token.creator] = token.order

                # check if retransmission is needed
                if token.hops_travelled < token.hops_to_target:
                    broadcast += [token]

                # check if token is of kind action
                if token.kind == 'action':
                    try:
                        # check if there is data for this robot
                        data = (token.creator, token.data[self.robot_id])
                        # send data to controller
                        response.append({self.outPorts['external_action']: data})
                    except KeyError:
                        pass

                # check if token is of kind state
                elif token.kind == 'state':
                    # check if token creator is within action extent
                    if token.hops_travelled <= self.action_extent:
                        # send data to controller
                        data = (token.creator, token.data)
                        response.append({self.outPorts['other_position']: data + (token.hops_travelled, )})
                        if token.hops_travelled == 1:
                            # send data to positioning system
                            response.append({self.outPorts['neighbors_positions']: data + (distance, )})

        return history, broadcast, response


