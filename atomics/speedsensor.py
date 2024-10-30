import numpy as np
from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY
from atomics.qsstools import *


class SpeedSensorDiffState:
    """
    Encapsulates the system's state
    """
    def __init__(self, sigma, tvalue, data):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue, data)

    def set(self, sigma, tvalue, data):
        self._sigma = sigma
        self._tvalue = tvalue
        self._data = data

    def get(self):
        return self._sigma, self._tvalue, self.data


class SpeedSensorDiff(AtomicDEVS):
    def __init__(self,name=None,period=0.1,noisestd=0.0,bias=np.zeros((2,1)),transf=np.eye(2),debug=False):
        """Atomic model for the speed sensor"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        # self.robot_id = robot_id    # Robot identifier
        # self.status = []          # TODO

        # STATE:
        #  Define 'state' attribute (initial sate):
        _time0  = 0.0
        _sigma0 = period # waits till firts token
        _x0 = np.array([0.0, 0.0],dtype=float)
        _y0 = np.array([0.0, 0.0],dtype=float)
        _xprev = np.array([0.0],dtype=float)
        _yprev = np.array([0.0],dtype=float)
        _data0  = {'xprev': _xprev, 'yprev': _yprev, 'x': _x0, 'y': _y0}
        self.state = SpeedSensorDiffState(_sigma0,_time0,_data0) 
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        self.period   = period
        self.noisestd = noisestd
        self.bias     = bias
        self.transf   = transf

        self.debug = debug

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.out_measured_speed = self.addOutPort(name="out_measured_speed")
        #
        self.in_position_x = self.addInPort(name="in_position_x")
        self.in_position_y = self.addInPort(name="in_position_y")

        if (self.debug):
            print("t: 0 s, Parent name: {}, Atomic name: {}, Init Function".format(self.parent.parent.name,self.name))

    def __lt__(self, other):
        return self.name < other.name
    
    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, data = self.state.get()
        current_time += self.elapsed

        # update polynomial coefficients for x
        if self.in_position_x in inputs:
            poly_x = inputs[self.in_position_x]
            data['x'] = poly_x
            data['y'] = advance_time(data['y'], self.elapsed, 1)

        # update polynomial coefficients for y
        elif self.in_position_y in inputs:
            poly_y = inputs[self.in_position_y]
            data['x'] = advance_time(data['x'], self.elapsed, 1)
            data['y'] = poly_y
        
        sigma = sigma - self.elapsed # holds last status
        
        if (self.debug):
            print("t: {:.2f} s, Parent name: {}, Atomic name: {}, External Transition Function, v: {}, vmeasured: {}".format(current_time,self.parent.parent.name,self.name,v,vmeasured))

        return SpeedSensorDiffState(sigma, current_time, data) 
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time, data = self.state.get()
        current_time += sigma

        # update the xprev and yprev items
        xact  = evaluate_poly(data['x'], sigma, 1, debug=True)
        yact  = evaluate_poly(data['y'], sigma, 1, debug=True) 
        data['xprev'] = xact
        data['yprev'] = yact

        sigma = self.period

        if (self.debug):
            print("t: {:.2f} s, Parent name: {}, Atomic name: {}, Internal Transition Function".format(current_time,self.parent.parent.name,self.name))
            
        return SpeedSensorDiffState(sigma,current_time,data) 
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        sigma, current_time, data = self.state.get()

        # robot speeds (vx and vy) are represented by scalars, thus
        # the speed calculation is done by evaluating the polynomials.
        xact  = evaluate_poly(data['x'], sigma, 1, debug=True)
        yact  = evaluate_poly(data['y'], sigma, 1, debug=True) 
        xprev = data['xprev']
        yprev = data['yprev']
        vx = (xact - xprev) / self.period
        vy = (yact - yprev) / self.period
        v = np.array([[float(vx)], [float(vy)]])
        noise = np.random.normal(loc=self.bias,scale=self.noisestd,size=(2,1))
        vmeasured = self.transf.dot(v) + noise

        print("t: {:.2f} s, Parent name: {}, Atomic name: {}, Output Function, vmeasured: {}".format(current_time,self.parent.parent.name,self.name,vmeasured))
        
        return {self.out_measured_speed: vmeasured}

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _, _ = self.state.get()
        return sigma