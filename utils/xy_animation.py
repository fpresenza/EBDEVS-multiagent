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
        return (0.0, min(250.0 + 10*t, 1000.0))

    def set_ylim(self, t):
        return (0.0, min(250.0 + 10*t, 1000.0))

    def _update_extra_artists(self, frame):
        for i, target in enumerate(frame[4]):
            if (target == 'Passive'):
                try:
                    self._extra_artists[i].remove()
                except ValueError:
                    pass


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
n_robots = len(robot_ids)
targets_config = read_json_file(experiment_directory + 'targets.json')
target_ids = list(targets_config)
n_targets = len(target_ids)
print('Experiment located in: {}'.format(experiment_directory))
print('Number of robots: {}'.format(n_robots))

# ------------------------------------------------------------------
# Create Frames
# ------------------------------------------------------------------
data = []
with open(experiment_directory + 'global.csv', 'r') as file:
    reader = csv.reader(file)
    for row in reader:
        data.append(row)

frames = []
positions = np.zeros((n_robots, 2), dtype=float)
targets = ['Active' for _ in range(n_targets)]
edges = []
teams = np.arange(n_robots)
last_time = -1e-3
for step, step_data in enumerate(data[1:]):
    time = float(step_data[1])
    if step_data[0].startswith('Robot'):
        robot_index = robot_ids.index(step_data[0])
        positions[robot_index] = step_data[2:4]
        neighbors = [int(r[-1]) for r in step_data[5:] if r.startswith('Robot')]

        edges = [e for e in edges if robot_index not in e] + \
            [[robot_index, neighbor] for neighbor in neighbors]
    
    elif step_data[0].startswith('Target'):
        target_index = target_ids.index(step_data[0])
        targets[target_index] = step_data[5]

    if time > last_time + 1e-3:
        last_time = time
        frames.append([time, positions.copy(), edges, teams, targets.copy()])

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
            'color': 'C0',
            'marker': 'o',
            'markersize': 5,
            'markeredgewidth': 0.5,
            'zorder': 20
        }
    }
    for i in range(n_robots)
})

anim.set_edgestyle(color='k', lw=0.5, zorder=10)

for target in targets_config.values():
    circle = plt.Circle(np.array(target['position'], dtype=float), target['collect_range'], color='r', alpha=0.3)
    ax.add_artist(circle)
    anim.add_extra_artists(circle)

video_path = experiment_directory + 'xy_animation.mp4'
anim.run(video_path)

subprocess.run(["mpv", video_path])