import numpy as np
from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY
from utils.files import append_csv_file


class KalmanFilterState:
    """
    Encapsulates the system's state
    """

    def __init__(self, sigma, tvalue, position, covariance):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue, position, covariance)

    def set(self, sigma, tvalue, position, covariance):
        self._sigma = sigma
        self._tvalue = tvalue
        self._position = position
        self._covariance = covariance

    def get(self):
        return self._sigma, self._tvalue, self._position, self._covariance


class KalmanFilter(AtomicDEVS):
    def __init__(
            self, 
            robot_id, 
            config, 
            name='KalmanFilter', 
            logpath='./', 
            debug=False):
        """Atomic model for the kalman filter"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        self.robot_id = robot_id    # Robot identifier
        
        # self.status = []          # TODO

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.state = KalmanFilterState(
            sigma=INFINITY,
            tvalue=0.0,
            position=np.array(config['position']),
            covariance=np.array(config['covariance'])
        )
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        self.logpath = logpath
        self.debug = debug

        # kalman filter parameters (hardcoded)
        self.velocity_measurement_covariance = np.array([[0.0225, 0.0], [0.0, 0.0225]])
        self.distance_measurement_covariance = np.array([[25.0]])
        self.position_measurement_covariance = np.array([[25.0, 0.0], [0.0, 25.0]])

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.out_control_intpos = self.addOutPort(name="out_control_intpos")
        self.out_handler_intpos = self.addOutPort(name="out_handler_intpos")
        #
        self.in_dynamics_velmeas = self.addInPort(name="in_dynamics_velmeas")
        self.in_handler_extpos = self.addInPort(name="in_handler_extpos")

        self.outputs_queue = []

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    def __lt__(self, other):
        return self.name < other.name
    
    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, position, covariance = self.state.get()
        current_time += self.elapsed

        if self.in_dynamics_velmeas in inputs: # if data arrives through port in_dynamics_velmeas
            speed_measurement = inputs[self.in_dynamics_velmeas].reshape(2, 1)
            position, covariance = self.dynamic_step(position, covariance, speed_measurement) # events list
            sigma = 0.0 # holds last status
        elif self.in_handler_extpos in inputs: # if token arrives through port in_handler_extpos
            robot_id, neighbor_position, distance = inputs[self.in_handler_extpos]
            position, covariance = self.range_step(position, covariance, neighbor_position, distance) # events list

        log = [current_time, position[0][0], position[1][0]]
        append_csv_file(self.logpath + 'kalman_{}.csv'.format(self.robot_id), log)
        
        if (self.debug):
            print("t: {:.2f} s, Atomic name: {}, External Transition Function".format(current_time, self.name))

        return KalmanFilterState(sigma, current_time, position, covariance)
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        _, current_time, position, covariance = self.state.get()

        if len(self.outputs_queue) == 0:
            sigma = INFINITY
        else:
            sigma = 0.0

        if (self.debug):
            print("t: {:.2f} s, Atomic name: {}, Internal Transition Function".format(current_time,self.name))
            
        return KalmanFilterState(sigma, current_time, position, covariance) 
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        if len(self.outputs_queue) == 0:
            _, _, position, _ = self.state.get()
            self.outputs_queue.append({self.out_control_intpos: position})
            self.outputs_queue.append({self.out_handler_intpos: position})

        return self.outputs_queue.pop()

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _, _, _ = self.state.get()
        return sigma

    def dynamic_step(self, position, covariance, speed_measurement):
        """Prediction step based on control actions"""
        # the list of outputs to be returned
        position += speed_measurement * self.elapsed      # debe ser el tiempo entre acciones de control (chequear)
        covariance += self.velocity_measurement_covariance * self.elapsed**2
        return position, covariance

    def range_step(self, position, covariance, neighbor_position, dist):
        """Update step based on distance measurements with neighbors

        args:
        -----
            neighbor_position : neighbor position
            dist : distance measurements
        """
        r = position - neighbor_position
        d = np.sqrt(np.square(r).sum())
        H = r.T / d
        PHt = covariance.dot(H.T)
        Pz = H.dot(PHt) + self.distance_measurement_covariance
        # TODO: include neighbor covariance
        # Pz = H.dot(PHt) + H.dot(Pj.dot(H.T)) + self.distance_measurement_covariance
        K = PHt / Pz
        position += K.dot(dist - d)
        covariance -= K.dot(H).dot(covariance)
        return position, covariance