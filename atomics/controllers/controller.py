import numpy as np

from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY

from utils.files import append_csv_file


class ControllerState:
    """
    Encapsulates the system's state
    """
    def __init__(
            self, 
            sigma, 
            tvalue
            ):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue)

    def set(self, sigma, tvalue):
        self._sigma  = sigma
        self._tvalue = tvalue

    def get(self):
        return self._sigma, self._tvalue


class Controller(AtomicDEVS):
    def __init__(
            self,
            robot_id,
            config,
            name='Controller',
            logpath='./',
            debug=False
        ):
        """Atomic model for the controller"""

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
        )
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.inPorts = {
            # 
            #    add inports here
            # 
        }        
        self.outPorts = {
            # 
            #    add outports here
            # 
        }
            
        self.outputs_queue = []

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time = self.state.get()
        current_time += self.elapsed    # NOTE: self.elapsed is always zero
        sigma -= self.elapsed    # holds last status

        #
        #    implement external transition here
        #

        if (self.debug):
            print(
                "t: {:.2f} s, Atomic name: {}, External Transition Function"
                .format(current_time, self.name)
            )

        return ControllerState(sigma, current_time)
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time = self.state.get()
        current_time += sigma

        if len(self.outputs_queue) == 0:
            sigma = self.period
            #
            #    nothing else to output here
            #
        else:
            sigma = 0.0

        if (self.debug):
            print(
                "t: {:.2f} s, Atomic name: {}, Internal Transition Function"
                .format(current_time, self.name)
            )

        return ControllerState(sigma, current_time) 
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        if len(self.outputs_queue) == 0:
            sigma, current_time = self.state.get()
            #
            #    compute control action here   
            #

            #
            #    append control action to self.outputs_queue here
            #

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