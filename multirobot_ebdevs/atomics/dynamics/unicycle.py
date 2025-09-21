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
        u_array = np.array(u).flatten()
        x_array = np.array(x).flatten()
        
        dot_x = np.array([
            u_array[0] * np.cos(x_array[2]),
            u_array[0] * np.sin(x_array[2]),
            u_array[1]
        ])

        result = [[val] for val in dot_x.tolist()]

        return result
