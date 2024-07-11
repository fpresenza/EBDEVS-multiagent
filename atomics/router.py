import numpy as np
from dataclasses import dataclass
from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY


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
    def __init__(self, agents_ids, name=None, period=1):
        """
        Atomic model for the toking handling protocol
        """

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        # self.agents = range(number_of_robots)
        self.agents = agents_ids

        self.tokens = {
            0: Token(creator='0',kind='action',order=127,data={'3': np.array([1.0, 2.0])},hops_to_target=4,hops_travelled=1),
            1: Token(creator='1',kind='action',order=127,data={'2': np.array([2.0, 4.0])},hops_to_target=4,hops_travelled=2),
            2: Token(creator='2',kind='action',order=128,data={'2': np.array([4.0, 6.0])},hops_to_target=4,hops_travelled=2),
            3: Token(creator='3',kind='action',order=128,data={'4': np.array([5.0, -3.0])},hops_to_target=4,hops_travelled=2),
        }
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
        self.out_router_token = {
            agent: self.addOutPort(name="out_router_token_{}".format(i)) for i, agent in enumerate(self.agents)
        }

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
        return {self.out_router_token[self.agents[i]]: self.tokens[i]}

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, current_time, i = self.state.get()
        return sigma


class RouterState:
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


class Router(AtomicDEVS):
    def __init__(self, agents_ids, name=None):
        """Atomic model for the Router """

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        # self.agents = range(number_of_robots)
        self.agents = agents_ids
        num_agents = len(agents_ids)
        # self.status = []          # TODO

        # STATE:
        #  Define 'state' attribute (initial sate):
        _time0  = 0.0
        _sigma0 = INFINITY # waits till firts token
        _data0  = []
        self.state = RouterState(_sigma0,_time0,_data0) 
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.out_agent_token = {
            agent: self.addOutPort(name="out_agent_token_{}".format(i)) for i, agent in enumerate(self.agents)
        }
        self.in_agent_token = {
            agent: self.addInPort(name="in_agent_token_{}".format(i)) for i,agent in enumerate(self.agents)
        }

        self.in_port_mapping = {"in_agent_token_{}".format(i): agent_id for i, agent_id in enumerate(self.agents)}

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, data = self.state.get()
        current_time += self.elapsed

        port, token = list(inputs.items())[0]
        transmitter = self.in_port_mapping[port.name]
        receivers = self.parent.getContextInformation(transmitter)
        data += [{self.out_agent_token[agent]: token} for agent, _ in receivers]
        # (no me acuerdo para que era lo de abajo pero lo copie de otro lado)
        sigma = 0 # holds last status

        return RouterState(sigma, current_time, data) 
    
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
        return RouterState(sigma,current_time,data) 
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        sigma, current_time, data = self.state.get()
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