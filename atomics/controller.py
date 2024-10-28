import numpy as np
from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY

# do not reset random seed
np.random.seed(0)

class ControllerGeneratorState:
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


class ControllerGenerator(AtomicDEVS):
    def __init__(self,name=None,period=1):
        """
        Atomic model for the controller inputs
        """

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # STATE:
        #  Define 'state' attribute (initial sate):
        _time0  = 0.0
        _sigma0 = 0
        self.state = ControllerGeneratorState(_sigma0,_time0) 
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.out_controller_extact = self.addOutPort(name="out_controller_extact")
        self.out_controller_extpos = self.addOutPort(name="out_controller_extpos")
        self.out_controller_intpos = self.addOutPort(name="out_controller_intpos")
 
        # Parameters
        self.msgs = [
            {self.out_controller_extact: np.array([-30.0, -30.0])},
            {self.out_controller_extpos: np.array([30.0, 30.0])},
            {self.out_controller_intpos: np.array([40.0, 50.0])}
        ]
        self.period = period

    def __lt__(self, other):
        return self.name < other.name

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        # it should never be executed
        sigma, current_time, i = self.state.get()
        current_time += self.elapsed
        return ControllerGeneratorState(sigma,current_time) 
    
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
        return ControllerGeneratorState(sigma,current_time) 
    
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


class ControllerState:
    """
    Encapsulates the system's state
    """

    def __init__(
            self, 
            sigmaval=0.1, 
            tval=0.0, 
            last_position=np.array([0.0, 0.0]), 
            ext_action=np.array([0.0, 0.0])
        ):
        """
        Constructor (parameterizable).
        """
        self.set(sigmaval, tval, last_position, ext_action)

    def set(self, sigmavalue, tvalue, last_position, control_action):
        self._sigma  = sigmavalue
        self._tvalue = tvalue
        self._last_position = last_position
        self._ext_action = control_action

    def get(self):
        return self._sigma, self._tvalue, self._last_position, self._ext_action


class Controller(AtomicDEVS):
    def __init__(self, robot_id, config, name='Controller', debug=False):
        """Atomic model for the rigidity maintenance controller"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        self.robot_id = robot_id    # Robot identifier
        self.period = config['period']

        # STATE:
        #  Define 'state' attribute (initial sate):
        _time0  = 0.0
        _sigma0 = self.period # waits till firts token
        _last_position0 = None
        _ext_action0 = np.array([0.0, 0.0])
        self.state = ControllerState(
            _sigma0,
            _time0, 
            _last_position0,
            _ext_action0
        )
        self.subframework = {}

        self.freq   = 0.05
        self.radius = np.random.uniform(1.0, 10.0)
        self.omega  = np.random.choice([-1, 1]) * 2*np.pi*self.freq

        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        self.debug = debug

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.out_kalman_intact   = self.addOutPort(name="out_kalman_intact")
        self.out_handler_intact  = self.addOutPort(name="out_handler_intact")
        self.out_dynamics_intact = self.addOutPort(name="out_dynamics_intact")
        
        self.in_kalman_intpos   = self.addInPort(name="in_kalman_intpos")
        self.in_handler_extpos  = self.addInPort(name="in_handler_extpos")
        self.in_handler_extact  = self.addInPort(name="in_handler_extact")
        
        self.first_control_call = True

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, last_position, ext_action = self.state.get()
        current_time += self.elapsed

        if self.in_kalman_intpos in inputs: # if data arrives through port in_kalman_intpos
            last_position = inputs[self.in_kalman_intpos]
            sigma = sigma - self.elapsed # holds last status
        elif self.in_handler_extpos in inputs: # if ext pos arrives through port IN_handler
            node_id, ext_position = inputs[self.in_handler_extpos]
            self.subframework[node_id] = ext_position
        elif self.in_handler_extact in inputs: # if ext action arrives through port IN_handler
            ext_action += inputs[self.in_handler_extact]
            sigma = sigma - self.elapsed # holds last status

        if (self.debug):
            print("t: {:.2f} s, Atomic name: {}, External Transition Function".format(current_time,self.name))

        return ControllerState(sigma, current_time, last_position, ext_action) 
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time, last_position, ext_action = self.state.get()
        current_time += sigma
        self.action.pop()
        if len(self.action) == 0:
            sigma = self.period
            self.first_control_call = True
            ext_action = 0
        else:
            sigma = 0

        if (self.debug):
            print("t: {:.2f} s, Atomic name: {}, Internal Transition Function".format(current_time,self.name))

        return ControllerState(sigma,current_time,last_position, ext_action) 
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        sigma, current_time, last_position, ext_action = self.state.get()
        if self.first_control_call:
            int_action, ext_action = self.control_action(last_position, ext_action)
            self.action = [
                {self.out_kalman_intact: int_action}, 
                {self.out_handler_intact: ext_action},
                {self.out_dynamics_intact: int_action}
            ]
            self.first_control_call = False

        return self.action[-1]

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _, _, _ = self.state.get()
        return sigma

    def __lt__(self, other):
        return self.name < other.name

    def control_action(self, position, ext_action):
        """Compute control action"""
        _, current_time, _, _ = self.state.get()
        x = -self.radius*self.omega*np.sin(self.omega*current_time)
        y =  self.radius*self.omega*np.cos(self.omega*current_time)
        # the list of outputs to be returned
        if position is None: # if Kalman filter has not yet sent an estimation of position
            # return np.array([0.0, 0.0]), {'2': np.array([-4.0, 4.0])}
            return np.array([0.0, 0.0]), {}
        else:
            # return np.array([0.0, 0.0]), {}
            return np.array([x, y]), {}

