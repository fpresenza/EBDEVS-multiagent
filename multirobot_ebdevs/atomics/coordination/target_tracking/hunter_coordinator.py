#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY

from multirobot_ebdevs.atomics.coordination.target_tracking.token import Token


class HunterCoordinatorState:
    """
    Encapsulates the system's state
    """
    def __init__(self, sigma, tvalue, record, position, nearest_target):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue, record, position, nearest_target)

    def set(self, sigma, tvalue, record, position, nearest_target):
        self._sigma = sigma
        self._tvalue = tvalue
        self._record = record
        self._state = position
        self._nearest_target = nearest_target

    def get(self):
        return (
            self._sigma,
            self._tvalue,
            self._record,
            self._state,
            self._nearest_target
        )


class HunterCoordinator(AtomicDEVS):
    def __init__(
            self,
            robot_id,
            config,
            name='HunterCoordinator',
            debug=False
            ):
        """Atomic model for the toking handling protocol"""

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        self.robot_id = robot_id    # Robot identifier
        self.action_extent = config['action']      # The robot's action extent
        self.state_extent = config['state']        # The robot's state extent
        # self.status = []          # TODO
        self.debug = debug

        # Dictionaries as records of tokens received

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.state = HunterCoordinatorState(
            sigma=INFINITY,
            tvalue=0.0,
            record={'action': 0, 'state': 0},
            position=None,
            nearest_target={'id': None, 'sqdist': np.inf},
        )
        # ELAPSED TIME:
        #  Initialize 'elapsed time' attribute if required
        #  (by default, value is 0.0):
        self.elapsed = 0.0

        # PORTS:
        #  Declare as many input and output ports as desired
        #  (usually store returned references in local variables):
        #
        self.inPorts = {
            'token': self.addInPort(name="in_token"),
            'others_actions': self.addInPort(name="in_others_actions"),
            'position': self.addInPort(name="in_position")
        }

        self.outPorts = {
            'token': self.addOutPort(name="out_token"),
            'other_position': self.addOutPort(name="out_other_position"),
            'neighbors_positions': self.addOutPort(
                name="out_neighbors_positions"
            ),
            'external_action': self.addOutPort(name="out_external_action"),
            'target_position': self.addOutPort(name="out_target_position")
        }

        self.outputs_queue = []

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    def extTransition(self, inputs):
        """
        External Transition Function.
        """
        sigma, current_time, record, position, nearest_target = \
            self.state.get()
        current_time += self.elapsed

        if self.inPorts['token'] in inputs:
            #  if token arrives through port self.inPorts['token']
            transmitter, token, distance_meas = inputs[self.inPorts['token']]

            response, nearest_target = self.handle_received_token(
                token, distance_meas, position, nearest_target
            )

            if len(response) > 0:    # else pass, nothing to send
                self.outputs_queue += response
                sigma = 0.0

        elif self.inPorts['others_actions'] in inputs:
            #  if data arrives through port inPorts['others_actions']
            action_token = Token(
                creator=self.parent.name,
                kind='action',
                order=record['action'],
                data=inputs[self.inPorts['others_actions']],
                hops_to_target=self.action_extent,
                hops_travelled=1
            )
            record['action'] += 1
            self.outputs_queue.append({self.outPorts['token']: action_token})

            if position is not None:
                state_token = Token(
                    creator=self.parent.name,
                    kind='state',
                    order=record['state'],
                    data=position,
                    hops_to_target=self.state_extent,
                    hops_travelled=1
                )
                record['state'] += 1
                position = None
                self.outputs_queue.append(
                    {self.outPorts['token']: state_token}
                )

            sigma = 0.0

        elif self.inPorts['position'] in inputs:
            #  if data arrives through port inPorts['position']
            position = inputs[self.inPorts['position']]

        if (self.debug):
            print(
                "t: {} s, Atomic name: {}, Ext. Transition Func."
                .format(current_time, self.name)
            )

        return HunterCoordinatorState(
            sigma,
            current_time,
            record,
            position,
            nearest_target
        )

    def intTransition(self):
        """
        Internal Transition Function.
        """
        _, current_time, record, position, nearest_target = self.state.get()

        if len(self.outputs_queue) == 0:
            sigma = INFINITY
        else:
            sigma = 0.0

        if (self.debug):
            print(
                "t: {} s, Atomic name: {}@{}, Internal Transition Function"
                .format(current_time, self.name, self.parent.name)
            )

        return HunterCoordinatorState(
            sigma,
            current_time,
            record,
            position,
            nearest_target
        )

    def outputFnc(self):
        """
        Output Funtion.
        """
        _, current_time, _, _, _ = self.state.get()

        if (self.debug):
            print(
                "t: {} s, Atomic name: {}@{}, Output Function, data: {}"
                .format(
                    current_time,
                    self.name,
                    self.parent.name,
                    self.outputs_queue[0]
                )
            )

        return self.outputs_queue.pop(0)

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition
        # based (typically) on current State.
        sigma, _, _, _, _ = self.state.get()
        return max(sigma, 0.0)

    def __lt__(self, other):
        return self.name < other.name

    def handle_received_token(
            self,
            token,
            distance,
            position,
            nearest_target
            ):
        """Decide what to do with the received token"""
        response = []

        # check if token is of kind action
        if token.kind == 'action':
            try:
                # check if there is data for this robot
                data = (token.creator, token.data[self.robot_id])
                # send data to controller
                response.append({self.outPorts['external_action']: data})
            except KeyError:
                pass

        # check if token is of kind state
        elif token.kind == 'state':
            # check if token creator is within action extent
            if token.hops_travelled <= self.action_extent:
                # send data to controller
                data = (token.creator, token.data)
                response.append(
                    {
                        self.outPorts['other_position']:
                        data + (token.hops_travelled, )
                    }
                )
                if token.hops_travelled == 1:
                    # send data to positioning system
                    response.append(
                        {
                            self.outPorts['neighbors_positions']:
                            data + (distance, )
                        }
                    )

        # check if it is nearest target
        elif token.kind == 'active':
            target_position = token.data
            square_dist = np.sum(np.square(position - target_position))
            if token.creator == nearest_target['id']:
                nearest_target['sqdist'] = square_dist
            elif square_dist < nearest_target['sqdist']:
                nearest_target['id'] = token.creator
                nearest_target['sqdist'] = square_dist
                response.append(
                    {self.outPorts['target_position']: target_position}
                )

        elif token.kind == 'passive':
            if token.creator == nearest_target['id']:
                nearest_target['id'] = None
                nearest_target['sqdist'] = np.inf

        return response, nearest_target
