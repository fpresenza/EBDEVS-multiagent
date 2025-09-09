#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np

from multirobot_ebdevs.atomics.dynamics\
    .dynamics_function import DynamicsFunction


class Unicycle(DynamicsFunction):
    def vector_field(self, x, u):
        #
        #  compute f(x, u) here
        #
        # x = [x, y, theta]
        # u = [v, w]
        dot_x = [0.0, 0.0, 0.0]
        dot_x[0] = u[0] * np.cos(x[2])
        dot_x[1] = u[0] * np.sin(x[2])
        dot_x[2] = u[1]
        return dot_x
