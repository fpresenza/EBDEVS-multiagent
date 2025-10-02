#!/usr/bin/env python
# -*- coding: utf-8 -*-

from pypdevs.DEVS import AtomicDEVS

from multirobot_ebdevs.utils.files import append_csv_file, append_jsonl_file


class LoggerState:
    """
    Encapsulates the system's state
    """

    def __init__(self, sigma, tvalue):
        """
        Constructor (parameterizable).
        """
        self.set(sigma, tvalue)

    def set(self, sigma, tvalue):
        self._sigma = sigma
        self._tvalue = tvalue

    def get(self):
        return self._sigma, self._tvalue


class Logger(AtomicDEVS):
    def __init__(self, period, name='Logger', log_path='./', debug=False):
        """Atomic model for the Logger """

        # Always call parent class' constructor FIRST:
        AtomicDEVS.__init__(self, name)

        # Parameters
        # self.robots = range(number_of_robots)
        self.period = period
        self.log_path = log_path
        self.debug = debug

        # STATE:
        #  Define 'state' attribute (initial sate):
        self.state = LoggerState(sigma=0.0, tvalue=0.0)

        if (self.debug):
            print("t: 0 s, Atomic name: {}, Init Function".format(self.name))

    def intTransition(self):
        """
        Internal Transition Function.
        """
        sigma, current_time = self.state.get()

        current_time += sigma
        sigma = self.period

        if (self.debug):
            print(
                "t: {} s, Atomic name: {}, Internal Transition Function"
                .format(current_time, self.name)
            )

        return LoggerState(sigma, current_time)

    def outputFnc(self):
        """
        Output Funtion.
        """
        sigma, current_time = self.state.get()

        robot_pose, target_state, adjacency_list = self.parent.getGlobalState(
            current_time + sigma
        )
        append_csv_file(
            self.log_path + 'log_time.csv', [current_time + sigma]
        )
        append_csv_file(
            self.log_path + 'robots_pose.csv', robot_pose
        )
        append_csv_file(
            self.log_path + 'targets_state.csv', target_state
        )
        append_jsonl_file(
            self.log_path + 'adjacency_list.jsonl', adjacency_list
        )

        if (self.debug):
            print(
                "t: {} s, Atomic name: {}, Output Function, data: {}"
                .format(current_time, self.name, self.outputs_queue[0])
            )

        return {}

    def timeAdvance(self):
        """
        Time-Advance Function.
        """
        # Compute 'ta', the time to the next scheduled internal transition,
        # based (typically) on current State.
        sigma, _ = self.state.get()
        return max(sigma, 0.0)

    def __lt__(self, other):
        return self.name < other.name
