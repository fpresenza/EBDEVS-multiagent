import numpy as np

from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY

from uvnpy.distances.control import RigidityMaintenance
from uvnpy.control.core import CollisionAvoidanceVanishing

from utils.files import append_csv_file


# do not reset random seed
np.random.seed(0)


class TargetControlState:
    """
    Encapsulates the system's state
    """

    def __init__(
            self, 
            sigma, 
            tvalue, 
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

class TargetControl(AtomicDEVS):
    def __init__(
            self,
            robot_id,
            config,
            name='TargetControl',
            debug=False
        ):
        """Atomic model for the rigidity maintenance controller"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        self.robot_id = robot_id    # Robot identifier
        self.period = config['period']
        self.debug = debug

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.state = TargetControlState(
            sigma=self.period,   # waits till first token
            tvalue=0.0
        )
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.outPorts = {'beacon': self.addOutPort(name="out_beacon")}
            
        self.outputs_queue = []

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time = self.state.get()
        
        current_time += sigma
        sigma = self.period

        if (self.debug):
            print("t: {:.2f} s, Atomic name: {}, Internal Transition Function".format(current_time,self.name))

        return TargetControlState(sigma, current_time) 
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        return {self.outPorts['beacon']: None}

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _ = self.state.get()
        return max(sigma, 0.0)

    def __lt__(self, other):
        return self.name < other.name