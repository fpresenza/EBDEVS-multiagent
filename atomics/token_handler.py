import numpy as np
from dataclasses import dataclass
from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY

#__all__ = [
#    'Token',
#    'TokenHandler'
#]
TOKEN_KINDS = {
    'action': 0,
    'state' : 1, 
}


@dataclass
class Token(object):
    creator: str                # The robot that created it
    kind: str                   # Whether it is action or state
    order: int                  # A counter to differentiate tokens
    data: object                # The data it carries
    hops_to_target: int         # The number of hops it must travel
    hops_travelled: int = 0     # The number of hops it has travelled

class TokenGeneratorState:
    """
    Encapsulates the system's state
    """

    def __init__(self, sigmaval=0.1, tval=0.0, ival=0):
        """
        Constructor (parameterizable).
        """
        self.set(sigmaval, tval, ival)

    def set(self, sigmavalue, tvalue, ivalue):
        self._sigma  = sigmavalue
        self._tvalue = tvalue
        self._ivalue = ivalue

    def get(self):
        return self._sigma, self._tvalue, self._ivalue

class TokenGenerator(AtomicDEVS):
    def __init__(self,name=None,period=1):
        """
        Atomic model for the toking handling protocol
        """

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        token_0 = Token(creator='2',kind='action',order=127,data={'3': np.array([1.0, 2.0])},hops_to_target=4,hops_travelled=1)
        token_1 = Token(creator='1',kind='action',order=127,data={'2': np.array([1.0, 2.0])},hops_to_target=4,hops_travelled=2)
        token_2 = Token(creator='1',kind='action',order=128,data={'2': np.array([1.0, 2.0])},hops_to_target=4,hops_travelled=2)
        token_3 = Token(creator='3',kind='state' ,order=132,data={'1': np.array([3.0, 4.0])},hops_to_target=4,hops_travelled=2)
        self.tokens = [token_0, token_1, token_2, token_3]
        self.N = len(self.tokens)
        self.period = period
        # STATE:
        #  Define 'state' attribute (initial sate):
        _time0  = 0.0
        _sigma0 = 0
        _i0 = 0
        self.state = TokenGeneratorState(_sigma0,_time0,_i0) 
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.OUT_token    = self.addOutPort(name="token_out")

    def __lt__(self, other):
        return self.name < other.name

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        # it should never be executed
        sigma, current_time, i = self.state.get()
        current_time += self.elapsed
        return TokenGeneratorState(sigma,current_time,i) 
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time, i = self.state.get()
        current_time += sigma
        if i<self.N-1:
            i += 1
            sigma = self.period
        else:
            i += 1
            sigma = INFINITY
        return TokenGeneratorState(sigma,current_time,i) 
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        sigma, current_time, i = self.state.get()
        y = self.tokens[i]
        return {self.OUT_token: y}

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, current_time, i = self.state.get()
        return sigma


class TokenHandlerState:
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


class TokenHandler(AtomicDEVS):
    def __init__(self,robot_id,name=None):
        """Atomic model for the toking handling protocol"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        self.robot_id = robot_id    # Robot identifier
        self.extent = INFINITY      # The robot's subgraph extent
        # self.status = []          # TODO

        # Dictionaries as records of tokens received
        # {'action': {creator: order}, 'state': {creator: order}}
        self.received = {'action': {}, 'state': {}}

        # STATE:
        #  Define 'state' attribute (initial sate):
        _time0  = 0.0
        _sigma0 = INFINITY # waits till firts token
        _data0  = []
        self.state = TokenHandlerState(_sigma0,_time0,_data0) 
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.OUT_token   = self.addOutPort(name="token_out")
        self.OUT_control = self.addOutPort(name="control_out")
        self.OUT_state   = self.addOutPort(name="state_out")
        #
        self.IN_token    = self.addInPort(name="token_in")
        self.IN_control  = self.addInPort(name="control_in")
        self.IN_state    = self.addInPort(name="state_in")

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, data = self.state.get()
        current_time += self.elapsed

        if self.IN_token in inputs: # if token arrives through port IN_token
            token = inputs[self.IN_token]
            ret = self.handle_received_token(token) # events list
            if (ret == []): # discard, the token received was sent by this same robot
                sigma = sigma - self.elapsed # holds last status
            else:
                data  = ret # events list
                sigma = 0
        elif self.IN_control in inputs: # if token arrives through port IN_control
            # pass # do nothing
            sigma = sigma - self.elapsed
        elif self.IN_state in inputs:   # if token arrives through port IN_state
            # pass # do nothing
            sigma = sigma - self.elapsed

        return TokenHandlerState(sigma,current_time,data) 
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time, data = self.state.get()
        sigma = INFINITY
        data = []
        return TokenHandlerState(sigma,current_time,data) 
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        sigma, current_time, data = self.state.get()
        port = data[0]
        y    = data[1]

        if port==1:
            return {self.OUT_control: y}
        elif port==2:
            return {self.OUT_state: y}
        else:
            return {self.OUT_token: y}

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _, _ = self.state.get()
        return sigma

    def handle_received_token(self, token):
        """Decide what to do with the received token"""
        # the list of outputs to be returned
        outputs = []

        if token.creator == self.robot_id:
            # do nothing if this robot is the creator
            pass
        else:
            # update the number of traversed hops
            token.hops_travelled += 1

            # check if retransmission is needed
            if token.hops_travelled < token.hops_to_target:
                # outputs.append((self.OUT_token, token))
                outputs.append((0, token))

            try:
                # check if already received from creator
                order = self.received[token.kind][token.creator]
            except KeyError:
                # append to list of tokens received
                order = -1

            # check if token is newer than last received
            if token.order > order:
                self.received[token.kind][token.creator] = token.order
                # check if token is of kind action
                if TOKEN_KINDS[token.kind] == 0:
                    try:
                        # check if there is data for this robot
                        data = token.data[self.robot_id]
                        # send data to controller
                        # outputs.append((self.OUT_control, data))
                        outputs.append((1, data))
                    except KeyError:
                        pass
                # check if token is of kind state
                elif TOKEN_KINDS[token.kind] == 1:
                    # check if token creator is within extent
                    if token.hops_travelled <= self.extent:
                        # send data to controller
                        data = (token.creator, token.data)
                        # outputs.append((self.OUT_control, data))
                        outputs.append((1, data))
                        if token.hops_travelled == 1:
                            # send data to positioning system
                            # outputs.append((self.OUT_state, data))
                            outputs.append((2, data))

        return outputs


