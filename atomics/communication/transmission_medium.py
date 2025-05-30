from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY


class TransmissionMediumState:
    """
    Encapsulates the system's state
    """

    def __init__(self, sigma, tvalue):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue)

    def set(self, sigma, tvalue):
        self._sigma = sigma
        self._tvalue = tvalue

    def get(self):
        return self._sigma, self._tvalue


class TransmissionMedium(AtomicDEVS):
    def __init__(self, robots_ids, targets_ids, name=None, debug=False):
        """Atomic model for the TransmissionMedium """

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        # self.robots = range(number_of_robots)
        self.robots = robots_ids
        self.targets = targets_ids
        # self.status = []          # TODO
        self.debug = debug

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.state = TransmissionMediumState(sigma=INFINITY, tvalue=0.0)
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.inPorts = {}
        self.outPorts = {}

        for robot_id in self.robots:
            self.inPorts[robot_id] = self.addInPort(
                                                name="in_{}".format(robot_id)
                                                )
            self.outPorts[robot_id] = self.addOutPort(
                                                name="out_{}".format(robot_id)
                                                )

        for target_id in self.targets:
            self.inPorts[target_id] = self.addInPort(
                                                name="in_{}".format(target_id)
                                                )
            self.outPorts[target_id] = self.addOutPort(
                                                name="out_{}".format(target_id)
                                                )

        self.outputs_queue = []

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time = self.state.get()
        current_time += self.elapsed

        _, (transmitter, token_batch) = inputs.popitem()

        receivers = self.parent.getNeighbors(transmitter, current_time)
        if len(receivers) > 0:
            self.outputs_queue += [
                {self.outPorts[receiver_id]: (transmitter, token_batch)}
                for receiver_id in receivers
            ]
            sigma = 0.0    # holds last status

        if (self.debug):
            print(
                "t: {} s, Atomic name: {}, External Transition Function,"
                "transmitter: {} -> receivers: {}"
                .format(current_time, self.name, transmitter, receivers)
            )

        return TransmissionMediumState(sigma, current_time)

    def intTransition(self):
        """
        Internal Transition Function.
        """
        _, current_time = self.state.get()

        if len(self.outputs_queue) == 0:
            sigma = INFINITY
        else:
            sigma = 0.0

        if (self.debug):
            print("t: {} s, Atomic name: {}, Internal Transition Function"
                  .format(current_time, self.name)
                  )

        return TransmissionMediumState(sigma, current_time)

    def outputFnc(self):
        """
        Output Funtion.
        """
        _, current_time = self.state.get()

        if (self.debug):
            print(
                "t: {} s, Atomic name: {}, Output Function, data: {}"
                .format(current_time, self.name, self.outputs_queue[0])
                  )

        return self.outputs_queue.pop(0)

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _ = self.state.get()
        return sigma

    def __lt__(self, other):
        return self.name < other.name
