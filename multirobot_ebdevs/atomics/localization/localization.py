from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY

from multirobot_ebdevs.utils.files import append_jsonl_file


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
            log_path='./',
            debug=False):
        """Atomic model for the kalman filter"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        self.robot_id = robot_id    # Robot identifier
        self.log_path = log_path
        self.debug = debug

        # self.status = []          # TODO

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.state = LocalizationState(
            sigma=INFINITY,
            tvalue=0.0,
            loc_filter=self.set_loc_filter(robot_id, config)
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
            name: self.addInPort(name="in_" + name)
            for name in self.set_in_port_names()
        }
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

        port_id, data = inputs.popitem()
        port_name = next(k for k, v in self.inPorts.items() if v == port_id)

        sigma, loc_filter = self.process_inputs(
            sigma, current_time, loc_filter, port_name, data
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
            self.log_path + 'kalman_{}.jsonl'.format(self.robot_id),
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
