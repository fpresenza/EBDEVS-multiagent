#!/usr/bin/env python
# -*- coding: utf-8 -*-

from multirobot_ebdevs.atomics.dynamics\
    .dynamics_function import DynamicsFunction


class SingleIntegrator(DynamicsFunction):
    def vector_field(self, x, u):
        #
        #  compute f(x, u) here
        #
        return u
