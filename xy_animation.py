#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" Created on mié 29 dic 2021 16:41:13 -03
@author: fran
"""
import numpy as np
import csv
import json
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import argparse

from uvnpy.network.plot import Animate2
from utils import read_json_file

np.set_printoptions(suppress=True, precision=4)

plt.rcParams['text.usetex'] = False
plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['ps.fonttype'] = 42
plt.rcParams['mathtext.fontset'] = 'dejavuserif'
plt.rcParams['font.family'] = 'serif'


class DevsAnimate(Animate2):
    def __init__(self, *args, **kwargs):
        super(DevsAnimate, self).__init__(*args, **kwargs)

    def _update_extra_artists(self, frame):
        positions = frame[1]
        n = int(len(positions))
        for i in range(n):
            self._extra_artists[i].center = positions[i]


# ------------------------------------------------------------------
# Parse args
# ------------------------------------------------------------------
parser = argparse.ArgumentParser(description='')
parser.add_argument(
    '-s', '--skip',
    default=10, type=int, help='skip frames'
)
parser.add_argument(
    '-x', '--speed',
    default=1, type=int, help='simulation speed multiplier'
)
arg = parser.parse_args()


# ------------------------------------------------------------------
# Read simulated data
# ------------------------------------------------------------------
summary = read_json_file('output/summary.csv')
robot_ids = summary['robot_ids']
n = len(robot_ids)
print('Experiment with {} robots'.format(n))

# ------------------------------------------------------------------
# Create Frames
# ------------------------------------------------------------------
data = []
with open('output/data.csv', 'r') as file:
    reader = csv.reader(file)
    for row in reader:
        data.append(row)

frames = []
positions = np.zeros((n, 2), dtype=float)
edges = []
for step in range(len(data[:-1])):
    robot_index = robot_ids.index(data[step][0])
    time = float(data[step][1])
    positions[robot_index] = data[step][2:4]
    neighbors = [int(r[-1]) for r in data[step][5:]]

    edges = [e for e in edges if robot_index not in e] + \
        [[robot_index, neighbor] for neighbor in neighbors]

    if time < float(data[step + 1][1]):
        teams = np.array([0, 1, 2, 3])
        frames.append([time, positions.copy(), edges, teams])

print('Total number of frames: {}'.format(len(frames)))
frames = frames[::arg.skip]
print('Reduced number of frames: {}'.format(len(frames)))
timestep = np.diff([frame[0] for frame in frames])
print('Average time between frames  Q_1={:.5f} sec, Q_2={:.5f} sec, Q_3={:.5f} sec'.format(
    np.quantile(timestep, 0.25),
    np.quantile(timestep, 0.50),
    np.quantile(timestep, 0.75),
))


timestep = np.quantile(timestep, 0.50) / arg.speed
print('Animation time between frames: {:.5f} sec'.format(timestep))
print('Animation total duration: {:.5f} sec'.format(len(frames) * timestep))

# ------------------------------------------------------------------
# Plot vs time
# ------------------------------------------------------------------
fig, ax = plt.subplots(2, 1, figsize=(10, 8))
ax[0].set_xlabel('time [$sec$]')
ax[0].set_ylabel('$x$-position [$m$]')
ax[0].grid()
ax[0].plot(
    [f[0] for f in frames], [f[1][:, 0] for f in frames],
    # marker='.',
    ds='steps-post')
ax[1].set_xlabel('time [$sec$]')
ax[1].set_ylabel('$y$-position [$m$]')
ax[1].grid()
ax[1].plot(
    [f[0] for f in frames], [f[1][:, 1] for f in frames],
    # marker='.',
    ds='steps-post')
fig.savefig('position.png', format='png', dpi=360)

# ------------------------------------------------------------------
# Animation
# ------------------------------------------------------------------
fig, ax = plt.subplots()
ax.tick_params(
    axis='both',       # changes apply to the x-axis
    which='both',      # both major and minor ticks are affected
    pad=1,
    labelsize='x-small'
)
ax.set_aspect('equal')
# ax.grid(1, lw=0.4)
ax.set_xlabel(r'$x$ [m]', fontsize='small', labelpad=0.6)
ax.set_ylabel(r'$y$ [m]', fontsize='small', labelpad=0.6)
lim = 15.0
ax.set_xlim(-lim, lim)
ax.set_ylim(-lim, lim)
ax.grid(zorder=0)

anim = DevsAnimate(fig, ax, timestep, frames, maxlen=1)
# cm.coolwarm goes from 0 (blue) to 255 (red)
anim.set_teams({
    '$0$': {
        'id': 0,
        'tail': False,
        'style': {
            'color': 'C0',
            'marker': 'o',
            'markersize': 10,
            'zorder': 20
        }
    },
    '$1$': {
        'id': 1,
        'tail': False,
        'style': {
            'color': 'C1',
            'marker': 'o',
            'markersize': 10,
            'zorder': 20
        }
    },
    '$2$': {
        'id': 2,
        'tail': False,
        'style': {
            'color': 'C2',
            'marker': 'o',
            'markersize': 10,
            'zorder': 20
        }
    },
    '$3$': {
        'id': 3,
        'tail': False,
        'style': {
            'color': 'C3',
            'marker': 'o',
            'markersize': 10,
            'zorder': 20
        }
    }
})
anim.set_edgestyle(color='k', lw=0.5, zorder=10)

circles = []
for p in frames[0][1]:
    circle = plt.Circle(p, 6.0, alpha=0.1)
    circles.append(circle)
    ax.add_artist(circle)
anim.set_extra_artists(*circles)

anim.ax.legend(
    ncol=4,
    loc='upper center',
    fontsize='small',
    handletextpad=1
)
anim.run('xy_animation.mp4')
