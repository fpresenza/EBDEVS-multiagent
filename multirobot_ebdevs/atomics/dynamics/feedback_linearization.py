#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np

from multirobot_ebdevs.atomics.dynamics\
    .dynamics_function import DynamicsFunction

class FeedbackLinearization(DynamicsFunction):
    def feedback_linearization(self, x, u, d = 1.0):

        linearized_u = [0.0, 0.0]
        
        th = x[2]
        ct = np.cos(th)
        st = np.sin(th)

        linearized_u[0] = u[0]*ct + u[1]*st
        linearized_u[1] = -u[0]*st/d + u[1]*ct/d

        return linearized_u