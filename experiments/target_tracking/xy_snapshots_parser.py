#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import argparse

from uvnpy.network.graphs import DiskGraph
from multirobot_ebdevs.utils.files import find_latest_timestamp

# ------------------------------------------------------------------
# Parse arguments
# ------------------------------------------------------------------
parser = argparse.ArgumentParser(description='')
parser.add_argument(
    '-r', '--comm_range',
    default=np.nan, type=float, help='common communication range'
)
arg = parser.parse_args()

if arg.comm_range is np.nan:
    raise ValueError('Communication range must be passed as argument.')

# ------------------------------------------------------------------
# Read simulated data
# ------------------------------------------------------------------
experiment_directory = find_latest_timestamp('output/')
print('Experiment located in: {}'.format(experiment_directory))

t = np.loadtxt(experiment_directory + 'logger_time.csv', delimiter=',')
robot_data = np.loadtxt(
    experiment_directory + 'logger_robot_data.csv', delimiter=','
)
target_data = np.loadtxt(
    experiment_directory + 'logger_target_data.csv', delimiter=','
)


# ------------------------------------------------------------------
# Parse and save
# ------------------------------------------------------------------
adjacency_matrix = [
    DiskGraph(
        realization=p.reshape(-1, 2),
        dmax=arg.comm_range
    ).adjacency_matrix().ravel()
    for p in robot_data
]

np.savetxt(experiment_directory + 't.csv', t, delimiter=',')
np.savetxt(experiment_directory + 'position.csv', robot_data, delimiter=',')
np.savetxt(experiment_directory + 'targets.csv', target_data, delimiter=',')
np.savetxt(
    experiment_directory + 'adjacency.csv', adjacency_matrix, delimiter=','
)
