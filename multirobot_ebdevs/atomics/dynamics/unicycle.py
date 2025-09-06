#!/usr/bin/env python
# -*- coding: utf-8 -*-

from multirobot_ebdevs.atomics.dynamics\
    .dynamics_function import DynamicsFunction


class Unicycle(DynamicsFunction):
    def vector_field(self, x, u):
        #
        #  compute f(x, u) here
        #
        # x = [x, y, theta]
        # u = [v, w]
        dot_x = []
        return dot_x
