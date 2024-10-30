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
import subprocess

from uvnpy.network.plot import Animate2
from files import read_json_file, find_latest_timestamp

np.set_printoptions(suppress=True, precision=4)

plt.rcParams['text.usetex'] = False
plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['ps.fonttype'] = 42
plt.rcParams['mathtext.fontset'] = 'dejavuserif'
plt.rcParams['font.family'] = 'serif'


class DevsAnimate(Animate2):
    def __init__(self, *args, **kwargs):
        super(DevsAnimate, self).__init__(*args, **kwargs)

    def set_xlim(self, t):
        return (0.0, 250.0 + t)

    def set_ylim(self, t):
        return (0.0, 250.0 + t)

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
experiment_directory = 'output/' + find_latest_timestamp('output/')
robots_config = read_json_file(experiment_directory + 'robots.json')
robot_ids = list(robots_config)
n = len(robot_ids)
print('Experiment located in: {}'.format(experiment_directory))
print('Number of robots: {}'.format(n))

# ------------------------------------------------------------------
# Create Frames
# ------------------------------------------------------------------
data = []
with open(experiment_directory + 'global.csv', 'r') as file:
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
        teams = np.arange(n)
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
if timestep < 0.001:
    raise ValueError('Animation timestep is too small, increase skip.')
print('Animation time between frames: {:.5f} sec'.format(timestep))
print('Animation total duration: {:.5f} sec'.format(len(frames) * timestep))

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
lim = 1000.0
ax.set_xlim(0, lim)
ax.set_ylim(0, lim)
ax.grid(zorder=0)

anim = DevsAnimate(fig, ax, timestep, frames, maxlen=1)
# cm.coolwarm goes from 0 (blue) to 255 (red)
anim.set_teams({
    i: {
        'id': i,
        'tail': False,
        'style': {
            'color': 'k',
            'marker': f'${i}$',
            'markersize': 5,
            'markeredgewidth': 0.5,
            'zorder': 20
        }
    }
for i in range(n)})
anim.set_edgestyle(color='k', lw=0.5, zorder=10)

circles = []
for p in frames[0][1]:
    circle = plt.Circle(p, 90.0, alpha=0.1)
    circles.append(circle)
    ax.add_artist(circle)
    anim.add_extra_artists(circle)

anim.ax.legend(
    ncol=4,
    loc='upper center',
    fontsize='small',
    handletextpad=1
)
video_path = experiment_directory + 'xy_animation.mp4'
anim.run(video_path)

subprocess.run(["mpv", video_path])