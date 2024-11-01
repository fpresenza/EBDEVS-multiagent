import numpy as np

from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY

from uvnpy.distances.localization import StatelessKalmanFilter
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
        self.logpath = logpath
        self.debug = debug
        
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

        # kalman filter parameters (hardcoded)
        self.ekf = StatelessKalmanFilter(
            input_covariance=np.array([[0.0225, 0.0], [0.0, 0.0225]]),
            distance_measurement_covariance=np.array([[100.0]]),
            position_measurement_covariance=np.array([[25.0, 0.0], [0.0, 25.0]])
        )

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.out_control_intpos = self.addOutPort(name="out_control_intpos")
        self.out_handler_intpos = self.addOutPort(name="out_handler_intpos")
        #
        self.in_dynamics_velmeas = self.addInPort(name="in_dynamics_velmeas")
        self.in_gps_posmeas = self.addInPort(name="in_gps_posmeas")
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
            velocity_measurement = inputs[self.in_dynamics_velmeas].reshape(2, 1)
            position, covariance = self.ekf.first_order_dynamic_step(
                position,
                covariance,
                self.elapsed,
                velocity_measurement
            )
            sigma = 0.0 # holds last status
        elif self.in_gps_posmeas in inputs: # if data arrives through port in_gps_posmeas
            position_measurement = inputs[self.in_gps_posmeas].reshape(2, 1)
            position, covariance = self.ekf.position_measurement_step(
                position,
                covariance,
                position_measurement
            )
        elif self.in_handler_extpos in inputs: # if token arrives through port in_handler_extpos
            robot_id, neighbor_position, distance_measurement = inputs[self.in_handler_extpos]
            position, covariance = self.ekf.asynchronous_distance_measurement_step(
                position, 
                covariance, 
                distance_measurement, 
                neighbor_position, 
                np.zeros((2, 2), dtype=float)
            )

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