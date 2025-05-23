from pypdevs.DEVS import AtomicDEVS

from utils.files import append_jsonl_file


class ControllerState:
    """
    Encapsulates the system's state
    """
    def __init__(self, sigma, tvalue, control):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue, control)

    def set(self, sigma, tvalue, control):
        self._sigma = sigma
        self._tvalue = tvalue
        self._control = control

    def get(self):
        return self._sigma, self._tvalue, self._control


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
            control=self.set_control(config)
        )
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.inPorts = self.set_in_ports()
        self.outPorts = {
            'action': self.addOutPort(name="out_action"),
            'coordination_data': self.addOutPort("out_coodination_data")
        }

        self.outputs_queue = []

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, control = self.state.get()
        current_time += self.elapsed    # NOTE: self.elapsed is always zero
        sigma -= self.elapsed    # holds last status

        control = self.process_inputs(
            sigma, current_time, control, inputs
        )

        if (self.debug):
            print(
                "t: {:.2f} s, Atomic name: {}, External Transition Function"
                .format(current_time, self.name)
            )

        return ControllerState(sigma, current_time, control)

    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time, control = self.state.get()
        current_time += sigma

        if len(self.outputs_queue) == 0:
            sigma = self.period
            control.clear()
        else:
            sigma = 0.0

        if (self.debug):
            print(
                "t: {:.2f} s, Atomic name: {}, Internal Transition Function"
                .format(current_time, self.name)
            )

        return ControllerState(sigma, current_time, control)

    def outputFnc(self):
        """
        Output Funtion.
        """
        sigma, current_time, control = self.state.get()

        if len(self.outputs_queue) == 0:
            control_action, coordination_data, control_metadata = \
                self.compute_action(control)
            self.outputs_queue.append(
                {self.outPorts['action']: control_action}
            )
            self.outputs_queue.append(
                {self.outPorts['coordination_data']: coordination_data}
            )

            append_jsonl_file(
                self.logpath + 'control_{}.jsonl'.format(self.robot_id),
                {
                    't': current_time,
                    'action': control_action.tolist(),
                    'metadata': control_metadata
                }
            )

        return self.outputs_queue.pop()

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _, _ = self.state.get()
        return max(sigma, 0.0)

    def __lt__(self, other):
        return self.name < other.name

    def set_control(self, config):
        #
        #    define controller here
        #
        return None

    def set_in_ports(self):
        #
        #    define input ports here
        #
        return {}

    def process_inputs(self, sigma, current_time, control, inputs):
        #
        #    process inputs here
        #
        return control

    def compute_action(self, control):
        #
        #    compute control action here
        #
        return None, None, None
