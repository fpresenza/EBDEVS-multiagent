import numpy as np
from dataclasses import dataclass
from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY

#__all__ = [
#    'Token',
#    'TokenHandler'
#]

@dataclass
class Token(object):
    creator: str                # The robot that created it
    kind: str                   # Whether it is action or state
    order: int                  # A counter to differentiate tokens
    data: object                # The data it carries
    hops_to_target: int         # The number of hops it must travel
    hops_travelled: int = 0     # The number of hops it has travelled

class TokenGenerator(AtomicDEVS):
    def __init__(self, name=None):
        """Atomic model for the toking handling protocol"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters

        # TODO: definir estados

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.OUT_token    = self.addOutPort(name="token_out")
        self.OUT_control  = self.addOutPort(name="control_out")
        self.OUT_state    = self.addOutPort(name="state_out")

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        # TODO
        return self.sigma 
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        self.sigma = INFINITY
        return self.sigma
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        # TODO
        return {self.OUT: y}

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        return self.sigma

class TokenHandler(AtomicDEVS):
    def __init__(self, robot_id, name=None):
        """Atomic model for the toking handling protocol"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        self.robot_id = robot_id    # Robot identifier
        self.extent = np.inf        # The robot's subgraph extent

        # Dictionaries as records of tokens received
        # {'action': {creator: order}, 'state': {creator: order}}
        self.received = {'action': {}, 'state': {}}

        # TODO: definir estados

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.OUT_token   = self.addOutPort(name="token_out")
        self.OUT_control = self.addOutPort(name="control_out")
        self.OUT_state   = self.addInPort(name="state_out")
        self.IN_token    = self.addInPort(name="token_in")
        self.IN_control  = self.addInPort(name="control_in")
        self.IN_state    = self.addInPort(name="state_in")

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        handle_received_token() # TODO
        return self.sigma 
    
    def intTransition(self):
        """
        Internal Transition Function.
        """
        self.sigma = INFINITY
        return self.sigma
    
    def outputFnc(self):
        """
        Output Funtion.
        """
        return {self.OUT: y}

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        return self.sigma


    def handle_received_token(self, token):
        if token.creator == self.robot_id:
            # do nothing if this robot is the creator
            pass
        else:
            # update the number of traversed hops
            token.hops_travelled += 1

            try:
                # check if already received from creator
                order = self.received[token.kind][token.creator]
            except KeyError:
                # append to list of tokens received
                order = -1

            # check if token is newer than last received
            if token.order > order:
                self.received[token.kind][token.creator] = token.order
                if token.kind == 'action':
                    self.handle_action_token_in(token)
                elif token.kind == 'state':
                    self.handle_state_token_in(token)
                # check if retransmission is needed
                if token.hops_travelled < token.hops_to_target:
                    self.broadcast_token(token)

    def broadcast_token(self, token):
        # TODO : enviar token al atómico 'router'
        pass

    def handle_action_token_in(self, token):
        # TODO : Enviar la accion de control al atómico 'controller'
        try:
            # check if there is data for this robot
            data = token.data[self.robot_id]
            # send data to controller
            pass    # NOT IMPLEMENTED
        except KeyError:
            pass

    def handle_state_token_in(self, token):
        # TODO : Enviar la posición recibida al atómico 'positioning_system'
        # check if token creator is within extent
        if token.hops_travelled <= self.extent:
            # send data to controller
            pass    # NOT IMPLEMENTED
            if token.hops_travelled == 1:
                # send data to positioning system
                pass    # NOT IMPLEMENTED