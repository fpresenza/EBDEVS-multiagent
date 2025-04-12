import numpy as np
from dataclasses import dataclass
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


class TargetCoordinationState:
    """
    Encapsulates the system's state
    """
    def __init__(self, sigma, tvalue, status, record, position):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue, status, record, position)

    def set(self, sigma, tvalue, status, record, position):
        self._sigma = sigma
        self._tvalue = tvalue
        self._status = status
        self._record = record
        self._state = position

    def get(self):
        return self._sigma, self._tvalue, self._status, self._record, self._state


class TargetCoordination(AtomicDEVS):
    def __init__(
            self, 
            robot_id,
            config,
            name='TargetCoordination', 
            debug=False
        ):
        """Atomic model for the toking handling protocol"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        self.robot_id = robot_id    # Robot identifier
        self.collect_range = config['collect_range']
        self.debug = debug

        # Dictionaries as records of tokens received

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.state = TargetCoordinationState(
            sigma=INFINITY, 
            tvalue=0.0,
            status='active',
            record=0,
            position=np.array(config['position']),
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
            'beacon': self.addInPort(name="in_beacon"), 
            'token': self.addInPort(name="in_token")
        }
        self.outPorts = {'token': self.addOutPort(name="out_token")}

        self.output = None

        self.y_up = [
            self.name, 
            {
                'time': 0.0, 
                'status': 'active', 
            }
        ]

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, status, record, position = self.state.get()
        current_time += self.elapsed

        if self.inPorts['token'] in inputs:    # if token arrives through port self.inPorts['token']
            transmitter, token, distance_meas = inputs[self.inPorts['token']]

            if transmitter.startswith('Robot'):
                if (status == 'active') and  (distance_meas < self.collect_range):
                    status = 'passive'
                    self.y_up[1]['time'] = current_time
                    self.y_up[1]['status'] = 'passive'
    
            if (self.debug):
                print(
                    "t: {} s, Atomic name: {}, External Transition Function, token: {} from Router"
                    .format(current_time, self.name, token)
                )

        elif self.inPorts['beacon'] in inputs: # if data arrives through port inPorts['beacon']
            token = Token(
                creator=self.robot_id,
                kind=status,
                order=record,
                data=position,
                hops_to_target=1,
                hops_travelled=0
            )
            record += 1
            self.output = token
            sigma = 0.0

            if (self.debug):
                print(
                    "t: {} s, Atomic name: {}@{}, External Transition Function, token: {} from Controller"
                    .format(current_time, self.name, self.robot_id, token)
                )

        return TargetCoordinationState(sigma, current_time, status, record, position) 
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        _, current_time, status, record, position = self.state.get()
    
        self.output = None        
        sigma = INFINITY

        if (self.debug):
            print(
                "t: {} s, Atomic name: {}@{}, Internal Transition Function"
                .format(current_time, self.name, self.robot_id)
            )

        return TargetCoordinationState(sigma, current_time, status, record, position)
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        _, current_time, _, _, _ = self.state.get()

        if (self.debug):
            print(
                "t: {} s, Atomic name: {}@{}, Output Function, data: {}"
                .format(current_time, self.name, self.robot_id, self.output)
            )
        
        return {self.outPorts['token']: self.output}
    

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


