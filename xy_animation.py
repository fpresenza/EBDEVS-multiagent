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
# Read simulated data
# ------------------------------------------------------------------

summary = read_json_file('output/summary.csv')
robot_ids = summary['robot_ids']
n = len(robot_ids)
print('Experiment with {} robots'.format(n))

frames = []
positions = np.zeros((n, 2), dtype=float)
edges = []
timesteps = [[] for _ in robot_ids]
with open('output/data.csv', 'r') as file:
    reader = csv.reader(file)
    for row in reader:
        robot_index = robot_ids.index(row[0])
        time = float(row[1])
        timesteps[robot_index].append(time)
        positions[robot_index] = row[2:4]
        neighbors = [int(r[-1]) for r in row[5:]]

        edges = [e for e in edges if robot_index not in e] + \
            [[robot_index, neighbor] for neighbor in neighbors]
        teams = np.array([1, 2, 3, 4])
        
        frames.append([time, positions.copy(), edges, teams])


nodes = np.arange(n)

# ------------------------------------------------------------------
# Create Frames
# ------------------------------------------------------------------
lim = 15.0
timestep = 0.01
print([len(ts) for ts in timesteps])

# ------------------------------------------------------------------
# Plot vs time
# ------------------------------------------------------------------
fig, ax = plt.subplots(2, 1, figsize=(10, 8))
ax[0].set_xlabel('time [$seg$]')
ax[0].set_ylabel('$x$-position [$m$]')
ax[0].grid(1)
ax[0].plot(
    [f[0] for f in frames], [f[1][:, 0] for f in frames],
    marker='.',
    ds='steps-post')
ax[1].set_xlabel('time [$seg$]')
ax[1].set_ylabel('$y$-position [$m$]')
ax[1].grid(1)
ax[1].plot(
    [f[0] for f in frames], [f[1][:, 0] for f in frames],
    marker='.',
    ds='steps-post')
fig.savefig('/tmp/position.png', format='png', dpi=360)
plt.show()

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
ax.set_xlim(-lim, lim)
ax.set_ylim(-lim, lim)
ax.grid(1)

anim = DevsAnimate(fig, ax, timestep, frames, maxlen=1)
# cm.coolwarm goes from 0 (blue) to 255 (red)
anim.set_teams({
    '$1$': {
        'id': 1,
        'tail': False,
        'style': {
            # 'color': 'royalblue',
            'color': 'C0',
            'marker': 'o',
            'markersize': 10
        }
    },
    '$2$': {
        'id': 2,
        'tail': False,
        'style': {
            # 'color': 'royalblue',
            'color': 'C1',
            'marker': 'o',
            'markersize': 10
        }
    },
    '$3$': {
        'id': 3,
        'tail': False,
        'style': {
            # 'color': 'royalblue',
            'color': 'C3',
            'marker': 'o',
            'markersize': 10
        }
    },
    '$4$': {
        'id': 4,
        'tail': False,
        'style': {
            # 'color': 'royalblue',
            'color': 'C4',
            'marker': 'o',
            'markersize': 10
        }
    }
})
anim.set_edgestyle(color='k', lw=0.5, zorder=0)

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
# anim.run('xy_animation.mp4')
