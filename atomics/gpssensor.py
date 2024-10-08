import numpy as np
from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY

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
    def __init__(self,name=None,noisestd=0.0,bias=np.zeros((2,1)),period=1,debug=False):
        """Atomic model for the speed sensor"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters

        # STATE:
        #  Define 'state' attribute (initial sate):
        _time0  = 0.0
        _sigma0 = period # waits till firts token
        _data0  = [0.0,0.0]
        self.state = GPSSensorState(_sigma0,_time0,_data0) 

        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        self.noisestd = noisestd
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

        if self.in_x_pos in inputs: # if data arrives through port in_x_pos
            x = inputs[self.in_x_pos]
            data[0] = self.evalpoly(x) + np.random.normal(loc=float(self.bias[0]),scale=self.noisestd)
            sigma = sigma - self.elapsed # holds last status

        if self.in_y_pos in inputs: # if data arrives through port in_y_pos
            y = inputs[self.in_y_pos]
            data[1] = self.evalpoly(y) + np.random.normal(loc=float(self.bias[1]),scale=self.noisestd)
            sigma = sigma - self.elapsed # holds last status
        
        if (self.debug):
            print("t: {:.2f} s, Parent name: {}, Atomic name: {}, External Transition Function".format(current_time,self.parent.parent.name,self.name))

        return GPSSensorState(sigma, current_time, data)
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time, data = self.state.get()
        current_time += sigma
        sigma = self.period

        if (self.debug):
            print("t: {:.2f} s, Atomic name: {}, data: {}, Internal Transition Function".format(current_time,self.name,data))

        if (self.debug):
            print("t: {:.2f} s, Atomic name: {}, Internal Transition Function".format(current_time,self.name))
            
        return GPSSensorState(sigma,current_time,data) 
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        sigma, current_time, data = self.state.get()
        outval = {self.out_meas_pos: data}
        return outval

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