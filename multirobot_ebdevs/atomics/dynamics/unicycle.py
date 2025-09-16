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
        print("Unicycle x={}, u={}".format(x,u))

        u_array = np.array(u)
        x_array = np.array(x)
        
        dot_x = np.array([
            u_array[0] * np.cos(x_array[2]),
            u_array[0] * np.sin(x_array[2]),
            u_array[1]
        ])

        result = [[val] for val in dot_x.tolist()]
        print(dot_x)

        return dot_x
