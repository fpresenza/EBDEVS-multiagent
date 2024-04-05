import numpy as np
from dataclasses import dataclass
from pypdevs.DEVS import AtomicDEVS


@dataclass
class Token:
    creator: str                # The robot that created it
    order: int                  # A counter to differentiate tokens
    data: object                # The data it carries
    hops_to_target: int         # The number of hops it must travel
    hops_travelled: int = 0     # The number of hops it has travelled


class TokenHandler(AtomicDEVS):
    def __init__(self, robot_id, name=None):
        """Atomic model for the toking handling protocol"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Robot identifier
        self.robot_id = robot_id

        # Dictionary used to remember the last tokens retransmitted
        # {cretor: order}
        self.retransmitted = {}

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        self.OUT_token   = self.addOutPort(name="token_out")
        self.OUT_control = self.addOutPort(name="control_out")
        self.IN_token    = self.addInPort(name="token_in")
        self.IN_control  = self.addInPort(name="control_in")
        self.IN_state    = self.addInPort(name="state_in")