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
            sigma=INFINITY, 
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
            'router': self.addInPort(name="in_router"),
            'kalman': self.addInPort(name="in_kalman")
        }
        self.outPorts = {
            'router': self.addOutPort(name="out_router"),
            'controller': self.addOutPort(name="out_controller")
        
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
        print(self.parent.name)
        
        if self.inPorts['router'] in inputs:
            target_id, target_position = inputs[self.inPorts['router']]
            targets[target_id] = target_position
        elif self.inPorts['kalman'] in inputs:   # if data arrives through port in_kalman
            position = inputs[self.inPorts['kalman']]
    
        if (self.debug):
            print(
                "t: {} s, Atomic name: {}, External Transition Function, target: {} from Router"
                .format(current_time, self.name, target_position)
            )

        return TargetHandlerState(sigma, current_time, targets, position) 
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        _, current_time, targets, position = self.state.get()
        
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
            goal_position = self.allocation(targets, position)
            self.outputs_queue.append({self.outPorts['controller']: goal_position})
            self.outputs_queue.append({self.outPorts['router']: None})
        
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
