import numpy as np
from pypdevs.DEVS import AtomicDEVS


class SpeedSensorState:
    """
    Encapsulates the system's state
    """

    def __init__(
            self,
            sigma,
            tvalue,
            position
            ):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue, position)

    def set(self, sigma, tvalue, position):
        self._sigma = sigma
        self._tvalue = tvalue
        self._position = position

    def get(self):
        return self._sigma, self._tvalue, self._position


class SpeedSensor(AtomicDEVS):
    def __init__(self, config, name='SpeedSensor', debug=False):
        """Atomic model for the positioning system"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        self.noise_mean = np.array(config['bias'])
        self.noise_covariance = np.array(config['covariance'])
        self.period = config['period']

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.state = SpeedSensorState(
            sigma=self.period,   # waits till first token
            tvalue=0.0,
            position=None
        )
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        self.debug = debug

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.inPorts = {
            'position_polynomial':
            self.addInPort(name="in_position_polynomial")
        }
        self.outPorts = {
            'velocity_measurement':
            self.addOutPort(name="out_velocity_measurement")
        }

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, _ = self.state.get()
        current_time += self.elapsed

        position = inputs[self.inPorts['position_polynomial']]
        sigma = sigma - self.elapsed  # holds last status

        if (self.debug):
            print(
                "t: {:.2f} s, Atomic name: {}, External Transition Function"
                .format(current_time, self.name)
            )

        return SpeedSensorState(sigma, current_time, position)

    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time, position = self.state.get()
        current_time += sigma

        sigma = self.period

        if (self.debug):
            print(
                "t: {:.2f} s, Atomic name: {}, Internal Transition Function"
                .format(current_time, self.name)
            )

        return SpeedSensorState(sigma, current_time, position)

    def outputFnc(self):
        """
        Output Funtion.
        """
        sigma, current_time, position = self.state.get()

        try:
            vx = position[0][1]
            vy = position[1][1]
        except TypeError:
            raise "Error: SpeedSensor has not received position data yet."
        noise_sample = np.random.multivariate_normal(
            mean=self.noise_mean.ravel(),
            cov=self.noise_covariance
        )
        output = np.array([vx, vy]) + noise_sample

        return {self.outPorts['velocity_measurement']: output}

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
