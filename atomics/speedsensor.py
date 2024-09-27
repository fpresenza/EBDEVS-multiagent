import numpy as np
from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY

class SpeedSensorState:
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


class SpeedSensor(AtomicDEVS):
    def __init__(self,name=None,noisestd=0.0,bias=np.zeros((2,1)),transf=np.eye(2),debug=False):
        """Atomic model for the speed sensor"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        # self.robot_id = robot_id    # Robot identifier
        # self.status = []          # TODO

        # STATE:
        #  Define 'state' attribute (initial sate):
        _time0  = 0.0
        _sigma0 = INFINITY # waits till firts token
        _data0  = []
        self.state = SpeedSensorState(_sigma0,_time0,_data0) 
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        self.noisestd = noisestd
        self.bias     = bias
        self.transf   = transf

        self.debug = debug

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.out_measured_speed = self.addOutPort(name="out_measured_speed")
        #
        self.in_commanded_speed = self.addInPort(name="in_commanded_speed")

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    def __lt__(self, other):
        return self.name < other.name
    
    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, _ = self.state.get()
        current_time += self.elapsed

        #if self.in_commanded_speed in inputs: # if data arrives through port in_control_intact
        v = inputs[self.in_commanded_speed].reshape(2,1)
        noise = np.random.normal(loc=self.bias,scale=self.noisestd,size=(2,1))
        vmeasured = self.transf.dot(v) + noise
        data = [{self.out_measured_speed: vmeasured}]
        sigma = 0 # holds last status
        
        if (self.debug):
            print("t: {:.2f} s, Parent name: {}, Atomic name: {}, External Transition Function, v: {}, vmeasured: {}".format(current_time,self.parent.parent.name,self.name,v,vmeasured))

        return SpeedSensorState(sigma, current_time, data) 
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        _, current_time, data = self.state.get()
        data.pop()
        sigma = INFINITY

        if (self.debug):
            print("t: {:.2f} s, Atomic name: {}, Internal Transition Function".format(current_time,self.name))
            
        return SpeedSensorState(sigma,current_time,data) 
    
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