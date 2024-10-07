# Copyright 2014 Modelling, Simulation and Design Lab (MSDL) at 
# McGill University and the University of Antwerp (http://msdl.cs.mcgill.ca/)
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys

# Import code for DEVS model representation:
from pypdevs.DEVS import *
from pypdevs.infinity import INFINITY
from atomics.qsstools import *

class QSSState:
    """
    Encapsulates the system's state
    """

    def __init__(self, qval=0, xval=[0,0], sigmaval=0.0, tval=0.0):
        """
        Constructor (parameterizable).
        """
        self.set(qval,xval,sigmaval, tval)

    def set(self, qvalue, xvalue, sigmavalue, tvalue):
        self._qvalue = qvalue
        self._xvalue = xvalue
        self._svalue = sigmavalue
        self._tvalue = tvalue

    def get(self):
        return [self._qvalue, self._xvalue, self._svalue, self._tvalue]

class QSSIntegrator(AtomicDEVS):
    """
    QSS1 integrator atomic model
    """
  
    def __init__(self, name=None, dQMin=1e-6, dQRel=1e-3, gain=1, x0=0, debug=False):
        """
        Constructor (parameterizable).
        """
        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)
    
        # PARAMETERS
        self.dQMin = dQMin
        self.dQRel = dQRel
        self.dQ    = max(abs(x0) * self.dQRel, self.dQMin)
        self.gain  = gain
        self.debug = debug

        # STATE:
        #  Define 'state' attribute (initial sate):
        _q0     =  x0
        _x0     = [x0, 0.0]
        _sigma0 = 0.0
        _t0     = 0.0
        self.state = QSSState(_q0, _x0, _sigma0, _t0) # q, x, sigma, t

        if self.debug:
            _q0, _x0, _sigma0, _t0 = self.state.get()
            print("Init Function @ {} - t0: {}, q0: {}, x0: {}, sigma0: {}".format(self.name,_t0,_q0,_x0,_sigma0))

        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0
    
        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.OUT_q = self.addOutPort(name="q_out")
        self.IN_dx = self.addInPort(name="dx_in")

    def __lt__(self, other):
        return self.name < other.name

    def intTransition(self):
        """
        Internal Transition Function.
        """
  
        q, xprev, sigma, current_time = self.state.get()

        current_time += sigma
        # TODO: replace by x = advance_time(xprev,sigma,1) # p: x, dt: sigma, order: 1
        # x = [xprev[0] + sigma * xprev[1], xprev[1]]
        x = advance_time(xprev, sigma, 1) # p: x, dt: sigma, order: 1
        q = x[0]

        self.dQ = max(self.dQRel * abs(x[0]), self.dQMin)

        if (x[1]==0):
            sigma = INFINITY
        else:
            sigma = abs(self.dQ/x[1])

        if (sigma<0):
            raise DEVSException(\
                  "invalid state sigma <%f> in internal transition function"\
                  % sigma)

        if self.debug:
            print("Internal Transition Function @ {} - t: {}, xprev: {}, x: {}, q: {}, sigma: {}".format(self.name,current_time,xprev,x,q,sigma))

        return QSSState(q,x,sigma,current_time)


    def extTransition(self, inputs):
        """
        External Transition Function.
        """

        # Received a new event, so start processing it
        derx     = inputs[self.IN_dx][0]
        derx_val = derx * self.gain # [dx[i] * self.gain for i in range(len(dx))]

        # if (x.port==0) {
        q, x, sigma, current_time = self.state.get()
        current_time += self.elapsed

        if self.IN_dx in inputs:
            # update polynomial x
            x[0] =  x[0] + x[1] * self.elapsed
            x[1] = derx_val # dx[0]

            diffxq = [0 for i in range(len(x))]

            if (sigma>0):
                # inferior delta crossing
                # diffxq = q - x - dQ = {q[0] - x[0] - dQ, -x[1]} 
                diffxq[1] = -x[1]
                diffxq[0] =  q - x[0] - self.dQ
                sigma     = minposroot(diffxq, 1) # coeff: diffxq, order: 1
                sigma_lo  = sigma

                # superior delta difference
                # diffxq = q - x + dQ = {q[0] - x[0] + dQ, -x[1]} 
                diffxq[0] =  q - x[0] + self.dQ
                sigma_up  = minposroot(diffxq, 1) # coeff: diffxq, order: 1

                # keep the smallest one
                if (sigma_up < sigma):
                    sigma = sigma_up

                if (abs(x[0] - q) > self.dQ):
                    sigma = 0

                if self.debug:
                    print("External Transition Function @ {} - t: {}, dx: {}, x: {}, sigma: {}, sigma_lo: {}, sigma_up: {}"\
                            .format(self.name,current_time,derx_val,x,sigma,sigma_lo,sigma_up))

        else:
            x[0] = derx_val
            sigma = 0

        return QSSState(q,x,sigma,current_time)

    def outputFnc(self):
        """
        Output Function.
        """

        y = []
        # BEWARE: ouput is based on the OLD state
        # and is produced BEFORE making the transition.
        # We'll encode an "observation" of the state the
        # system will transition to !
 
        q, xprev, sigma, current_time = self.state.get()


        if (sigma<0):
            raise DEVSException(\
                 "invalid state sigma <%f> in output function"\
                 % sigma)

        # y[0] = x[0]
        # y[1] = x[1]
        # y = x
        current_time += sigma
        # y = xprev

        # make time advance to get next q
        # this change in q will be performed right after in the internal transition function
        # y = advance_time(y,sigma,1) # p: y, dt: sigma, order: 1
        y = [xprev[0] + sigma * xprev[1], 0.0]
        # y[1] = 0.0

        if self.debug:
            print("Output Function @ {} - t: {} xprev: {}, y: {}".format(self.name, current_time, xprev, y))

        # Send messages (events) to a subset of the atomic-DEVS' 
        # output ports by means of the 'poke' method, i.e.:
        # The content of the messages is based (typically) on current State.
        return {self.OUT_q: y} #[y[0],y[1]]}

    
    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        q, x, sigma, current_time = self.state.get()

        if self.debug:
            print("Time Advance Function @ {} - t: {}, sigma: {}".format(self.name, current_time, sigma))

        return sigma

