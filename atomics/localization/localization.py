from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY

from utils.files import append_jsonl_file


class LocalizationState:
    """
    Encapsulates the system's state
    """
    def __init__(self, sigma, tvalue, loc_filter):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue, loc_filter)

    def set(self, sigma, tvalue, loc_filter):
        self._sigma = sigma
        self._tvalue = tvalue
        self._loc_filter = loc_filter

    def get(self):
        return self._sigma, self._tvalue, self._loc_filter


class Localization(AtomicDEVS):
    def __init__(
            self,
            robot_id,
            config,
            name='Localization',
            logpath='./',
            debug=False):
        """Atomic model for the kalman filter"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        self.robot_id = robot_id    # Robot identifier
        self.logpath = logpath
        self.debug = debug

        # self.status = []          # TODO

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.state = LocalizationState(
            sigma=INFINITY,
            tvalue=0.0,
            loc_filter=self.set_loc_filter(config)
        )
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        #
        self.inPorts = self.set_in_ports()
        self.outPorts = {'estimation': self.addOutPort(name="out_estimation")}

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    def __lt__(self, other):
        return self.name < other.name

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, loc_filter = self.state.get()
        current_time += self.elapsed

        sigma, loc_filter = self.process_inputs(
            sigma, current_time, loc_filter, inputs
        )

        if (self.debug):
            print(
                "t: {:.2f} s, Atomic name: {}, External Transition Function"
                .format(current_time, self.name)
            )

        return LocalizationState(sigma, current_time, loc_filter)

    def intTransition(self):
        """
        Internal Transition Function.
        """
        _, current_time, loc_filter = self.state.get()

        sigma = INFINITY

        if (self.debug):
            print(
                "t: {:.2f} s, Atomic name: {}, Internal Transition Function"
                .format(current_time, self.name)
            )

        return LocalizationState(sigma, current_time, loc_filter)

    def outputFnc(self):
        """
        Output Funtion.
        """
        _, current_time, loc_filter = self.state.get()

        loc_estimation, loc_metadata = self.loc_filter_results(loc_filter)

        append_jsonl_file(
            self.logpath + 'kalman_{}.jsonl'.format(self.robot_id),
            {
                't': current_time,
                'estimation': loc_estimation.tolist(),
                'metadata': loc_metadata
            }
        )

        return {self.outPorts['estimation']: loc_estimation}

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _, _ = self.state.get()
        return sigma

    def set_loc_filter(self, config):
        #
        #    define localization filter here
        #
        return None

    def set_in_ports(self):
        #
        #    define input ports here
        #
        return {}

    def process_inputs(self, sigma, current_time, loc_filter, inputs):
        #
        #    process inputs here
        #
        return sigma, loc_filter

    def loc_filter_results(self, loc_filter):
        #
        #    get estimation here
        #
        return None, None
