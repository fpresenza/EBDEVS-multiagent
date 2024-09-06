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
        self.out_handler_token    = self.addOutPort(name="token_out")

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
        return {self.out_handler_token: y}

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
    def __init__(self,robot_id,name=None,debug=False):
        """Atomic model for the toking handling protocol"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        self.robot_id = robot_id    # Robot identifier
        self.extent = INFINITY      # The robot's subgraph extent
        # self.status = []          # TODO
        self.debug = debug

        # Dictionaries as records of tokens received
        # {'action': {creator: order}, 'state': {creator: order}}
        self.received = {'action': {}, 'state': {}}
        self.action_token_order = 0
        self.state_token_order = 0

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
        self.out_router_token      = self.addOutPort(name="out_router_token")
        self.out_controller_extpos = self.addOutPort(name="out_controller_extpos")
        self.out_controller_extact = self.addOutPort(name="out_controller_extact")
        self.out_kalman_extpos     = self.addOutPort(name="out_kalman_extpos")
        #
        self.in_router_token       = self.addInPort(name="in_router_token")
        self.in_controller_intact  = self.addInPort(name="in_controller_intact")
        self.in_kalman_intpos      = self.addInPort(name="in_kalman_intpos")

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, data = self.state.get()
        current_time += self.elapsed

        if self.in_router_token in inputs: # if token arrives through port in_router_token
            token = inputs[self.in_router_token]
            ret = self.handle_received_token(token) # events list
            if (ret == []): # discard, the token received was sent by this same robot
                sigma = sigma - self.elapsed # holds last status
            else:
                data += ret # events list
                sigma = 0
            if (self.debug):
                print("t: {} s, Atomic name: {}, External Transition Function, token: {} from Router".format(current_time,self.name,token))

        elif self.in_controller_intact in inputs: # if token arrives through port in_controller_intact
            # pass # do nothing
            token = Token(
                creator=self.parent.name,
                kind='action',
                order=self.action_token_order,
                data=inputs[self.in_controller_intact],
                hops_to_target=1,
                hops_travelled=0
            )
            data.append({self.out_router_token: token})
            self.action_token_order+=1
            # sigma = sigma - self.elapsed
            sigma = 0
            if (self.debug):
                print("t: {} s, Atomic name: {}@{}, External Transition Function, token: {} from Controller".format(current_time,self.name,self.parent.name,token))

        elif self.in_kalman_intpos in inputs:   # if token arrives through port in_kalman_intpos
            # pass # do nothing
            # token = Token(
            #     creator=self.parent.name,
            #     kind='state',
            #     order=self.state_token_order,
            #     data=inputs[self.in_kalman_intpos],
            #     hops_to_target=1,
            #     hops_travelled=0
            # )
            self.state_token_order+=1
            sigma = sigma - self.elapsed
            if (self.debug):
                print("t: {} s, Atomic name: {}@{}, External Transition Function, token: {} from Kalman".format(current_time,self.name,self.parent.name,token))

        return TokenHandlerState(sigma,current_time,data) 
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        _, current_time, data = self.state.get()
        data.pop(0)
        if len(data) == 0:
            sigma = INFINITY
        else:
            sigma = 0
        if (self.debug):
            print("t: {} s, Atomic name: {}@{}, Internal Transition Function".format(current_time,self.name,self.parent.name))
        return TokenHandlerState(sigma,current_time,data)
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        sigma, current_time, data = self.state.get()
        if (self.debug):
            print("t: {} s, Atomic name: {}@{}, Output Function, data: {}".format(current_time,self.name,self.parent.name, data[0]))
        return data[0]
    

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
                outputs.append({self.out_router_token: token})

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
                        data = (token.creator, token.data[self.robot_id])
                        # send data to controller
                        outputs.append({self.out_controller_extact: data})
                    except KeyError:
                        pass
                # check if token is of kind state
                elif TOKEN_KINDS[token.kind] == 1:
                    # check if token creator is within extent
                    if token.hops_travelled <= self.extent:
                        # send data to controller
                        data = (token.creator, token.data)
                        outputs.append({self.out_controller_extpos: data})
                        if token.hops_travelled == 1:
                            # send data to positioning system
                            outputs.append({self.out_kalman_extpos: data})

        return outputs


