#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" Created on mié 29 dic 2021 16:41:13 -03
@author: fran
"""
import numpy as np
import csv
import json
import matplotlib.pyplot as plt
import argparse

from files import read_json_file, find_latest_timestamp

np.set_printoptions(suppress=True, precision=4)

plt.rcParams['text.usetex'] = False
plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['ps.fonttype'] = 42
plt.rcParams['mathtext.fontset'] = 'dejavuserif'
plt.rcParams['font.family'] = 'serif'

# ------------------------------------------------------------------
# Parse args
# ------------------------------------------------------------------
parser = argparse.ArgumentParser(description='')
parser.add_argument(
    '-s', '--skip',
    default=1, type=int, help='skip frames'
)
parser.add_argument(
    '-x', '--speed',
    default=1.0, type=float, help='simulation speed multiplier'
)
arg = parser.parse_args()


# ------------------------------------------------------------------
# Read simulated data
# ------------------------------------------------------------------
experiment_directory = find_latest_timestamp('output/')
robots_config = read_json_file(experiment_directory + 'robots.json')
robot_ids = list(robots_config)
n = len(robot_ids)
print('Experiment located in: {}'.format(experiment_directory))
print('Number of robots: {}'.format(n))

# ------------------------------------------------------------------
# Read ground truth data
# ------------------------------------------------------------------
data = []
with open(experiment_directory + 'global.csv', 'r') as file:
    reader = csv.reader(file)
    for row in reader:
        data.append(row)

time = {robot: [] for robot in robot_ids}
position_x = {robot: [] for robot in robot_ids}
position_y = {robot: [] for robot in robot_ids}
for k, row in enumerate(data):
    robot = row[0]
    time[robot].append(float(row[1]))
    position_x[robot].append(float(row[2]))
    position_y[robot].append(float(row[3]))

# ------------------------------------------------------------------
# Plots vs time
# ------------------------------------------------------------------
fig, ax = plt.subplots(2, 1, figsize=(10, 8))
ax[0].set_xlabel('time [$sec$]')
ax[0].set_ylabel('$x$-position [$m$]')
ax[0].grid()
ax[1].set_xlabel('time [$sec$]')
ax[1].set_ylabel('$y$-position [$m$]')
ax[1].grid()

for i, robot in enumerate(robot_ids):
    ax[0].plot(
        time[robot], position_x[robot],
        color='C{}'.format(i),
        marker='.',
        ds='steps-post'
    )
    ax[1].plot(
        time[robot], position_y[robot],
        color='C{}'.format(i),
        marker='.',
        ds='steps-post'
    )

    kalman_data = np.loadtxt(experiment_directory + 'kalman_{}.csv'.format(robot), delimiter=',')
    t, est_px, est_py = kalman_data.T

    ax[0].plot(
        t, est_px,
        color='C{}'.format(i),
        marker='s',
        markersize=2,
        ds='steps-post'
    )
    ax[1].plot(
        t, est_py,
        color='C{}'.format(i),
        marker='s',
        markersize=2,
        ds='steps-post'
    )

fig.savefig(experiment_directory + 'position.png', format='png', dpi=360)
plt.show()