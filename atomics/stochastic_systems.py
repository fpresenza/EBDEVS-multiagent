import numpy as np

from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY


class StochasticSystemState:
    """
    Encapsulates the system's state
    """
    def __init__(self, sigma, tvalue, state):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue, state)

    def set(self, sigma, tvalue, state):
        self._sigma = sigma
        self._tvalue = tvalue
        self._state = state

    def get(self):
        return self._sigma, self._tvalue, self._state


class StochasticSystem(AtomicDEVS):
    def __init__(
            self,
            initial_condition,
            process_noise,
            output_noise,
            name='StochasticSystem',
            debug=False):
        """
        Atomic model for the system between the input (u)
        and the output (y):
            x(n) = f(x(n-1), u(n)) + process_noise
            y(n) = h(x(n), u(n)) + output_noise
        where x is the system's state. 
        """

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        self.process_noise = process_noise
        self.output_noise = output_noise
        self.debug = debug

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.state = StochasticSystemState(sigma=INFINITY, tvalue=0.0, state=initial_condition) 

        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.output = self.addOutPort(name="output")
        self.input = self.addInPort(name="input")

    def __lt__(self, other):
        return self.name < other.name

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _, _ = self.state.get()
        return sigma


class ZeroOrderLinearSystem(StochasticSystem):
    def __init__(
            self,
            input_matrix,
            noise_mean,
            noise_covariance,
            name='ZeroOrderLinearSystem',
            debug=False):
        """
        Atomic model for a zero-order system between the input (u)
        and the output (y):
            v = input_matrix * u + noise
        """

        # Always call parent class' constructor FIRST:
        StochasticSystem.__init__(self, None, None, (noise_mean, noise_covariance), name)
        self.input_matrix = input_matrix

    
    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, _ = self.state.get()
        current_time += self.elapsed

        # data arrive through port in_control_intact
        input = inputs[self.input].reshape(-1, 1)
        noise_sample = np.random.multivariate_normal(
            mean=self.output_noise[0].ravel(),
            cov=self.output_noise[1]
        ).reshape(-1, 1)
        output = self.input_matrix.dot(input) + noise_sample
        sigma = 0.0    # holds last status
        
        if (self.debug):
            print(
                "t: {:.2f} s, Parent name: {}, Atomic name: {}, External Transition Function, input: {}, output: {}"
                .format(current_time, self.parent.parent.name, self.name, input, output)
            )

        return StochasticSystemState(sigma, current_time, output) 
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time, output = self.state.get()
        current_time += sigma
        sigma = INFINITY

        if (self.debug):
            print(
                "t: {:.2f} s, Parent name: {}, Atomic name: {}, Internal Transition Function"
                .format(current_time, self.parent.parent.name, self.name)
            )
            
        return StochasticSystemState(sigma, current_time, output) 
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        _, _, output = self.state.get()
        return {self.output: output}