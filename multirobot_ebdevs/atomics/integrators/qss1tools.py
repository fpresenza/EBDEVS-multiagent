#!/usr/bin/env python
# -*- coding: utf-8 -*-

from pypdevs.infinity import INFINITY


def advance_time(p, dt):
    p[0] = p[0] + dt * p[1]
    return p


def evaluate_poly(p, dt):
    return p[0] + dt * p[1]


def advance_time_q(p, dt):
    return p


def evaluate_poly_q(p, dt):
    return p[0]


def minposroot(p):
    if (p[1] == 0.0):
        # constant polynomial
        mpr = INFINITY
    else:
        # x(t) = p[0] + p[1] * t => 0 = p[0] + p[1] * t0 => t0 = -p[0]/p[1]
        mpr = -p[0] / p[1]
    # sanity check: time cannot be < 0
    if (mpr < 0):
        mpr = INFINITY

    return mpr


def pad_zeros(x):
    return x + [0.0] * (2 - len(x))


def pad_zeros_q(q):
    return q + [0.0] * (1 - len(q))
