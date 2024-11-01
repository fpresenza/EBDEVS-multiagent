import numpy as np
from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY
from atomics.qsstools import *


# do not reset random seed
np.random.seed(0)


class GPSSensorState:
    """
    Encapsulates the system's state
    """

    def __init__(self, sigmaval=0.1, tval=0.0, dataval=[]):
        """
        Constructor (parameterizable).
        """
        self.set(sigmaval, tval, dataval)

    def set(self, sigmavalue, tvalue, datavalue):
        self._sigma  = sigmavalue
        self._tvalue = tvalue
        self._data   = datavalue

    def get(self):
        return self._sigma, self._tvalue, self._data


class GPSSensor(AtomicDEVS):
    def __init__(self,name='GPSSensor',
                 noisecov=np.zeros((2,2)),
                 bias=np.zeros((2,1)),
                 period=1,
                 debug=False
                ):
        """Atomic model for the speed sensor"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters

        # STATE:
        #  Define 'state' attribute (initial sate):
        _time0  = 0.0
        _sigma0 = period # waits till firts token
        _data0  = {'x': [0.0]*10, 'y': [0.0]*10}
        self.state = GPSSensorState(_sigma0,_time0,_data0) 

        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        self.noisecov = noisecov
        self.bias     = bias
        self.period   = period
        self.debug    = debug

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.out_meas_pos = self.addOutPort(name="out_meas_pos")
        #
        self.in_x_pos = self.addInPort(name="in_x_pos")
        self.in_y_pos = self.addInPort(name="in_y_pos")

    def __lt__(self, other):
        return self.name < other.name
    
    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, data = self.state.get()
        current_time += self.elapsed

        if self.in_x_pos in inputs: # if data arrives through port in_x_pos
            x = inputs[self.in_x_pos]
            data['x'] = x
            data['y'] = advance_time(data['y'], self.elapsed, order=-1)
            sigma = sigma - self.elapsed # holds last status

        if self.in_y_pos in inputs: # if data arrives through port in_y_pos
            y = inputs[self.in_y_pos]
            data['x'] = advance_time(data['x'], self.elapsed, order=-1)
            data['y'] = y
            sigma = sigma - self.elapsed # holds last status
        
        if (self.debug):
            print("t: {:.2f} s, Parent name: {}, Atomic name: {}, External Transition Function".format(current_time,self.parent.parent.name,self.name))

        return GPSSensorState(sigma, current_time, data)
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time, data = self.state.get()
        data['x'] = advance_time(data['x'], sigma, order=-1)
        data['y'] = advance_time(data['y'], sigma, order=-1)
        current_time += sigma
        sigma = self.period

        if (self.debug):
            print("t: {:.2f} s, Parent name: {}, Atomic name: {}, Internal Transition Function".format(current_time,self.parent.parent.name,self.name))
            
        return GPSSensorState(sigma,current_time,data) 
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        sigma, current_time, data = self.state.get()
        noise = np.random.multivariate_normal(
                            [float(self.bias[0]), float(self.bias[1])],
                            self.noisecov
                            )
        x = evaluate_poly(data['x'], sigma, order=1) + noise[0]
        y = evaluate_poly(data['y'], sigma, order=1) + noise[1]

        return {self.out_meas_pos: [x, y]}

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _, _ = self.state.get()
        return sigma
    
    def evalpoly(self,p):
        return p[0]+p[1]*0 # eval poly in zero time


class PositioningSystemState:
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
        self._sigma  = sigma
        self._tvalue = tvalue
        self._position = position

    def get(self):
        return self._sigma, self._tvalue, self._position


class PositioningSystem(AtomicDEVS):
    def __init__(self, robot_id, config, name='PositioningSystem', debug=False):
        """Atomic model for the rigidity maintenance controller"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        self.robot_id = robot_id    # Robot identifier
        self.period = config['period']
        self.noise_mean = config['noise_mean']
        self.noise_covariance = config['noise_covariance']

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.state = PositioningSystemState(
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
        self.output = self.addOutPort(name="output")
        self.input = self.addInPort(name="input")
        
        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, _ = self.state.get()
        current_time += self.elapsed    # NOTE: self.elapsed is always zero

        position = inputs[self.input]
        sigma = sigma - self.elapsed # holds last status

        if (self.debug):
            print(
                "t: {:.2f} s, Atomic name: {}, External Transition Function"
                .format(current_time, self.name)
            )

        return PositioningSystemState(sigma, current_time, position)
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time, position = self.state.get()
        current_time += sigma

        sigma = self.period

        if (self.debug):
            print("t: {:.2f} s, Atomic name: {}, Internal Transition Function".format(current_time,self.name))

        return PositioningSystemState(sigma, current_time, position) 
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        sigma, current_time, position = self.state.get()

        try:
            px = position[0][0]
            py = position[1][0]
        except TypeError:
            raise "Error: PositioningSystem has not received position data yet."
        noise_sample = np.random.multivariate_normal(
            mean=self.noise_mean.ravel(),
            cov=self.noise_covariance
        )
        output = np.array([px, py]) + noise_sample

        return {self.output: output}

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _, _ = self.state.get()
        return sigma

    def __lt__(self, other):
        return self.name < other.name