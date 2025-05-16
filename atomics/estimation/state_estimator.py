import numpy as np

from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY

from utils.files import append_jsonl_file


class StateEstimatorState:
    """
    Encapsulates the system's state
    """
    def __init__(self, sigma, tvalue, estimation):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue, estimation)

    def set(self, sigma, tvalue, tvalue_prev, estimation):
        self._sigma = sigma
        self._tvalue = tvalue
        self._estimation = estimation

    def get(self):
        return self._sigma, self._tvalue, self._estimation


class StateEstimator(AtomicDEVS):
    def __init__(
            self,
            robot_id,
            config,
            name='StateEstimator',
            logpath='./',
            debug=False):
        """Atomic model for the state estimator"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        self.robot_id = robot_id    # Robot identifier
        self.logpath = logpath
        self.debug = debug

        # self.status = []          # TODO

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.state = StateEstimatorState(
            sigma=INFINITY,
            tvalue=0.0,
            estimation=np.array(config['estimation']),
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
            #
            #    add inports here
            #
        }
        self.outPorts = {
            #
            #    add inports here
            #
        }

        self.outputs_queue = []

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    def __lt__(self, other):
        return self.name < other.name

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, estimation = self.state.get()
        current_time += self.elapsed

        #
        #    implement external transition here
        #

        if (self.debug):
            print(
                "t: {:.2f} s, Atomic name: {}, External Transition Function"
                .format(current_time, self.name)
            )

        return StateEstimatorState(sigma, current_time, estimation)

    def intTransition(self):
        """
        Internal Transition Function.
        """
        _, current_time, estimation = self.state.get()

        if len(self.outputs_queue) == 0:
            sigma = INFINITY
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

        return StateEstimatorState(sigma, current_time, estimation)

    def outputFnc(self):
        """
        Output Funtion.
        """
        _, current_time, estimation = self.state.get()

        #
        #    append control action to self.outputs_queue here
        #

        append_jsonl_file(
            self.logpath + 'state_estimator_{}.jsonl'.format(self.robot_id),
            {'t': current_time, 'estimation': estimation, 'metadata': None}
        )

        return self.outputs_queue.pop()

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _, _ = self.state.get()
        return sigma
