#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np

from multirobot_ebdevs.atomics.dynamics\
    .dynamics_function import DynamicsFunction

class FeedbackLinearization(DynamicsFunction):
    def vector_field(self, x, u, d = 1.0):
        print("FeedbackLinearization x={}, u={}".format(x,u))
        u_array = np.array(u)
        x_array = np.array(x)

        th = x_array[2]
        ct = np.cos(th)
        st = np.sin(th)

        linearized_u = np.array([
            u_array[0]*ct + u_array[1]*st,
            -u_array[0]*st/d + u_array[1]*ct/d
        ])

        result = [[val] for val in linearized_u.tolist()]
        print(linearized_u)
        return linearized_u