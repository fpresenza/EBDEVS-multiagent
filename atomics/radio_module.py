import numpy as np
import copy
from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY


class RadioModuleState:
    """
    Encapsulates the system's state
    """
    def __init__(self, sigma, tvalue, record):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue, record)

    def set(self, sigma, tvalue, record):
        self._sigma = sigma
        self._tvalue = tvalue
        self._record = record

    def get(self):
        return self._sigma, self._tvalue, self._record


class RadioModule(AtomicDEVS):
    def __init__(
            self, 
            robot_id,
            name='RadioModule', 
            debug=False
            ):
        """Atomic model for the toking handling protocol"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        self.robot_id = robot_id    # Robot identifier
        # self.status = []          # TODO
        self.debug = debug

        # Dictionaries as records of tokens received

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.state = RadioModuleState(
            sigma=INFINITY,
            tvalue=0.0,
            record={'action': {}, 'state': {}},
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
            'radio': self.addInPort(name="in_radio"),
            'token': self.addInPort(name="in_token")
        }

        self.outPorts = {
            'radio': self.addOutPort(name="out_radio"),
            'token': self.addOutPort(name="out_token"),
            'target_position': self.addOutPort(name="out_target_position"),
        }

        self.outputs_queue = []

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, record = self.state.get()
        current_time += self.elapsed

        if self.inPorts['radio'] in inputs:
            transmitter, token, distance_meas = inputs[self.inPorts['radio']]

            # gets last order from record dictionary
            
            if token.creator == self.robot_id:
                # do nothing if this robot is the creator
               pass
            else:
                # update the number of traversed hops
                token = copy.deepcopy(token)
                token.hops_travelled += 1

                try:
                    last_order = record[token.kind][token.creator]
                except KeyError:
                    # first time received
                    last_order = -1

                # check if token is newer than last received
                if token.order > last_order:
                    record[token.kind][token.creator] = token.order

                    # check if retransmission is needed
                    if token.hops_travelled < token.hops_to_target:
                        self.outputs_queue.append({self.outPorts['radio']: (self.robot_id, token)})
                        sigma = 0.0

                    if transmitter.startswith('Robot'):
                        self.outputs_queue.append({self.outPorts['token']: (token, distance_meas)})
                        sigma = 0.0
                    elif transmitter.startswith('Target'):
                        self.outputs_queue.append({self.outPorts['target_position']: (token, distance_meas)})
                        sigma = 0.0

        elif self.inPorts['token'] in inputs:
            token = inputs[self.inPorts['token']]
            self.outputs_queue.append({self.outPorts['radio']: (self.robot_id, token)})
            sigma = 0.0

        if (self.debug):
            print(
                "t: {} s, Atomic name: {}, External Transition Function, token: {} from Router"
                .format(current_time, self.name, token)
            )

        return RadioModuleState(sigma, current_time, record) 
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        _, current_time, record = self.state.get()
        
        if len(self.outputs_queue) == 0:
            sigma = INFINITY
        else:
            sigma = 0.0

        if (self.debug):
            print(
                "t: {} s, Atomic name: {}@{}, Internal Transition Function"
                .format(current_time, self.name, self.parent.name)
            )

        return RadioModuleState(sigma, current_time, record)
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        _, current_time, _ = self.state.get()

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
        sigma, _ , _ = self.state.get()
        return max(sigma, 0.0)
    
    def __lt__(self, other):
        return self.name < other.name


