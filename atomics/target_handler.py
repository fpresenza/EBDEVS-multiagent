import numpy as np
from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY


class TargetHandlerState:
    """
    Encapsulates the system's state
    """
    def __init__(self, sigma, tvalue, targets, position):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue, targets, position)

    def set(self, sigma, tvalue, targets, position):
        self._sigma = sigma
        self._tvalue = tvalue
        self._targets = targets
        self._position = position

    def get(self):
        return self._sigma, self._tvalue, self._targets, self._position


class TargetHandler(AtomicDEVS):
    def __init__(
            self, 
            config,
            name='TargetHandler', 
            debug=False
            ):
        """Atomic model for the toking handling protocol"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        self.period = config["period"]
        # self.status = []          # TODO
        self.debug = debug

        # Dictionaries as records of targets received

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.state = TargetHandlerState(
            sigma=self.period, 
            tvalue=0.0, 
            targets={},
            position=None
        ) 
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.inPorts = {
            'target_position': self.addInPort(name="in_target_position"),
            'position': self.addInPort(name="in_position")
        }
        self.outPorts = {
            'collect': self.addOutPort(name="out_collect"),
            'target_position': self.addOutPort(name="out_target_position")
        }

        self.outputs_queue = []

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, targets, position = self.state.get()

        current_time += self.elapsed
        sigma -= self.elapsed    # holds last status

        if self.inPorts['target_position'] in inputs:
            target_id, target_position = inputs[self.inPorts['target_position']]
            targets[target_id] = target_position

        elif self.inPorts['position'] in inputs:   # if data arrives through port in_position
            position = inputs[self.inPorts['position']]
    
        if (self.debug):
            print(
                "t: {} s, Atomic name: {}, External Transition Function, target: {} from Router"
                .format(current_time, self.name)
            )

        return TargetHandlerState(sigma, current_time, targets, position) 
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time, targets, position = self.state.get()
        current_time += sigma
        
        if len(self.outputs_queue) == 0:
            sigma = self.period
            targets.clear()
            # position = None
        else:
            sigma = 0.0

        if (self.debug):
            print(
                "t: {} s, Atomic name: {}@{}, Internal Transition Function"
                .format(current_time, self.name, self.parent.name)
            )

        return TargetHandlerState(sigma, current_time, targets, position)
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        if len(self.outputs_queue) == 0:
            _, current_time, targets, position = self.state.get()
            target_position = self.allocation(targets, position)
            self.outputs_queue.append({self.outPorts['target_position']: target_position})
            self.outputs_queue.append({self.outPorts['collect']: None})
        
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

    def allocation(self, targets, position):
        if len(targets) > 0:
            # distances = position - list(targets.values())
            _, target_position = targets.popitem()
        else:
            target_position = None
        return target_position