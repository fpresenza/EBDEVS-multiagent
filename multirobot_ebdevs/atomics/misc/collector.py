#!/usr/bin/env python
# -*- coding: utf-8 -*-

from pypdevs.DEVS import AtomicDEVS


class CollectorState(object):
    def __init__(self):
        # Contains received events and simulation time
        self.events = []
        self.current_time = 0.0


class Collector(AtomicDEVS):
    def __init__(self, name="Collector", filename="output.csv"):
        AtomicDEVS.__init__(self, name)
        self.state = CollectorState()
        self.filename = filename
        # Has two input ports
        self.in1_event = self.addInPort("in1_event")
        self.in2_event = self.addInPort("in2_event")

    def extTransition(self, inputs):
        # Update simulation time
        self.state.current_time += self.elapsed

        # Write data to file
        with open(self.filename, 'a') as f:
            if self.in1_event in inputs:
                q1 = inputs[self.in1_event][0]
                current_time = self.state.current_time
                # self.state.events.append([current_time,q1,float('nan')])
                f.write("%f, %f,\n" % (current_time, q1))
            elif self.in2_event in inputs:
                q2 = inputs[self.in2_event][0]
                current_time = self.state.current_time
                # self.state.events.append([current_time,float('nan'),q2])
                f.write("%f, , %f\n" % (current_time, q2))
        return self.state

    # Don't define anything else, as we only store events.
    # Collector has no behaviour of its own.


class CollectorX8(AtomicDEVS):
    def __init__(self, name="Collector", filename="output.csv"):
        AtomicDEVS.__init__(self, name)
        self.state = CollectorState()
        self.filename = filename
        # Has two input ports
        self.in1_event = self.addInPort("in1_event")
        self.in2_event = self.addInPort("in2_event")
        self.in3_event = self.addInPort("in3_event")
        self.in4_event = self.addInPort("in4_event")
        self.in5_event = self.addInPort("in5_event")
        self.in6_event = self.addInPort("in6_event")
        self.in7_event = self.addInPort("in7_event")
        self.in8_event = self.addInPort("in8_event")

    def extTransition(self, inputs):
        # Update simulation time
        self.state.current_time += self.elapsed

        # Write data to file
        with open(self.filename, 'a') as f:
            if self.in1_event in inputs:
                q1 = inputs[self.in1_event][0]
                current_time = self.state.current_time
                # self.state.events.append([current_time, q1, float('nan')])
                f.write("%f, %f, , , , , ,\n" % (current_time, q1))
            elif self.in2_event in inputs:
                q2 = inputs[self.in2_event][0]
                current_time = self.state.current_time
                # self.state.events.append([current_time, float('nan'), q2])
                f.write("%f, , %f, , , , ,\n" % (current_time, q2))
            elif self.in3_event in inputs:
                q3 = inputs[self.in3_event][0]
                current_time = self.state.current_time
                # self.state.events.append([current_time, q1, float('nan')])
                f.write("%f, , , %f, , , , \n" % (current_time, q3))
            elif self.in4_event in inputs:
                q4 = inputs[self.in4_event][0]
                current_time = self.state.current_time
                # self.state.events.append([current_time,float('nan'),q2])
                f.write("%f, , , , %f, , , \n" % (current_time, q4))
            elif self.in5_event in inputs:
                q5 = inputs[self.in5_event][0]
                current_time = self.state.current_time
                # self.state.events.append([current_time,q1,float('nan')])
                f.write("%f, , , , , %f, , ,\n" % (current_time, q5))
            elif self.in6_event in inputs:
                q6 = inputs[self.in6_event][0]
                current_time = self.state.current_time
                # self.state.events.append([current_time,float('nan'),q2])
                f.write("%f, , , , , , %f, , \n" % (current_time, q6))
            elif self.in7_event in inputs:
                q7 = inputs[self.in7_event][0]
                current_time = self.state.current_time
                # self.state.events.append([current_time,q1,float('nan')])
                f.write("%f, , , , , , , %f,\n" % (current_time, q7))
            elif self.in8_event in inputs:
                q8 = inputs[self.in8_event][0]
                current_time = self.state.current_time
                # self.state.events.append([current_time,float('nan'),q2])
                f.write("%f, , , , , , , , %f\n" % (current_time, q8))
        return self.state
