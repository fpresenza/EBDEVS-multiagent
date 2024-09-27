import numpy as np
from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY


class KalmanGeneratorState:
    """
    Encapsulates the system's state
    """

    def __init__(self, sigmaval=0.1, tval=0.0):
        """
        Constructor (parameterizable).
        """
        self.set(sigmaval, tval)

    def set(self, sigmavalue, tvalue):
        self._sigma  = sigmavalue
        self._tvalue = tvalue

    def get(self):
        return self._sigma, self._tvalue


class KalmanGenerator(AtomicDEVS):
    def __init__(self,name=None,period=1):
        """
        Atomic model for the kalman filter inputs
        """

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # STATE:
        #  Define 'state' attribute (initial sate):
        _time0  = 0.0
        _sigma0 = 0
        self.state = KalmanGeneratorState(_sigma0,_time0) 
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.out_kalman_extpos = self.addOutPort(name="out_kalman_extpos")
        self.out_kalman_intact = self.addOutPort(name="out_kalman_intact")
 
        # Parameters
        self.msgs = [
            {self.out_kalman_intact: np.array([[0.0], [1.0]])},
            {self.out_kalman_extpos: (np.array([[0.0], [5.0]]), 6.0)},
            {self.out_kalman_extpos: (np.array([[7.0], [0.0]]), 6.0)}
        ]
        self.period = period

    def __lt__(self, other):
        return self.name < other.name

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        # it should never be executed
        sigma, current_time = self.state.get()
        current_time += self.elapsed
        return KalmanGeneratorState(sigma,current_time) 
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time = self.state.get()
        current_time += sigma
        self.msgs.pop()
        if len(self.msgs) == 0:
            sigma = INFINITY
        else:
            sigma = self.period
        return KalmanGeneratorState(sigma,current_time) 
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        # sigma, current_time = self.state.get()
        return self.msgs[-1]

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, current_time = self.state.get()
        return sigma


class KalmanFilterState:
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


class KalmanFilter(AtomicDEVS):
    def __init__(self, robot_id, x0, y0, name=None, debug=False):
        """Atomic model for the kalman filter"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        self.robot_id = robot_id    # Robot identifier
        # self.status = []          # TODO

        # STATE:
        #  Define 'state' attribute (initial sate):
        _time0  = 0.0
        _sigma0 = INFINITY # waits till firts token
        _data0  = []
        self.state = KalmanFilterState(_sigma0,_time0,_data0) 
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        self.debug = debug

        # kalman filter parameters (hardcoded)
        self.position = np.array([[x0], [y0]])
        self.covariance = np.array([[1.0, 0.0], [0.0, 1.0]])
        self.dynamic_process_covariance = np.array([[0.5, 0.0], [0.0, 0.5]])
        self.distance_measurement_covariance = np.array([[1.5]])
        self.position_measurement_covariance = np.array([[2.0, 0.0], [0.0, 2.0]])

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.out_control_intpos = self.addOutPort(name="out_control_intpos")
        self.out_handler_intpos = self.addOutPort(name="out_handler_intpos")
        #
        self.in_control_intact = self.addInPort(name="in_control_intact")
        self.in_handler_extpos = self.addInPort(name="in_handler_extpos")

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    def __lt__(self, other):
        return self.name < other.name
    
    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, data = self.state.get()
        current_time += self.elapsed

        if self.in_control_intact in inputs: # if data arrives through port in_control_intact
            control_action = inputs[self.in_control_intact].reshape(2,1)
            new_position = self.dynamic_step(control_action) # events list
            data = [
                {self.out_control_intpos: new_position},
                {self.out_handler_intpos: new_position}
            ]
            sigma = 0 # holds last status
        elif self.in_handler_extpos in inputs: # if token arrives through port in_handler_extpos
            robot_id, ext_position, distance = inputs[self.in_handler_extpos]
            new_position = self.range_step(ext_position, distance) # events list
            data = [{self.out_control_intpos: new_position}]
            sigma = 0 # holds last status
        
        if (self.debug):
            print("t: {:.2f} s, Atomic name: {}, External Transition Function".format(current_time,self.name))

        return KalmanFilterState(sigma, current_time, data) 
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        _, current_time, data = self.state.get()
        data.pop()
        if len(data) == 0:
            sigma = INFINITY
        else:
            sigma = 0

        if (self.debug):
            print("t: {:.2f} s, Atomic name: {}, Internal Transition Function".format(current_time,self.name))
            
        return KalmanFilterState(sigma,current_time,data) 
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        sigma, current_time, data = self.state.get()
        return data[-1]

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _, _ = self.state.get()
        return sigma

    def dynamic_step(self, control_action):
        """Prediction step based on control actions"""
        # the list of outputs to be returned
        self.position += control_action * self.elapsed      # debe ser el tiempo entre acciones de control (chequear)
        self.covariance += self.dynamic_process_covariance * self.elapsed**2
        return self.position.copy()

    def range_step(self, ext_position, dist):
        """Update step based on distance measurements with neighbors

        args:
        -----
            ext_position : neighbor position
            dist : distance measurements
        """
        r = self.position - ext_position
        d = np.sqrt(np.square(r).sum())
        H = r.T / d
        PHt = self.covariance.dot(H.T)
        Pz = H.dot(PHt) + self.distance_measurement_covariance
        K = PHt / Pz
        self.position += K.dot(dist - d)
        self.covariance -= K.dot(H).dot(self.covariance)
        return self.position.copy()


