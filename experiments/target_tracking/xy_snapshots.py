#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import numpy as np
import csv
import matplotlib.pyplot as plt
import argparse
import progressbar

from uvnpy.network import plot
from uvnpy.network.graphs import DiskGraph

from multirobot_ebdevs.utils.core import robot_id_to_index
from multirobot_ebdevs.utils.files import find_latest_timestamp


np.set_printoptions(suppress=True, precision=4)

plt.rcParams['text.usetex'] = False
plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['ps.fonttype'] = 42
plt.rcParams['mathtext.fontset'] = 'dejavuserif'
plt.rcParams['font.family'] = 'serif'


# ------------------------------------------------------------------
# Parse arguments
# ------------------------------------------------------------------
parser = argparse.ArgumentParser(description='')
parser.add_argument(
    '-i', '--init',
    default=0.0, type=float, help='init time in milli seconds'
)
parser.add_argument(
    '-e', '--end',
    default=0.0, type=float, help='end time in milli seconds'
)
parser.add_argument(
    '-j', '--jump',
    default=1, type=int, help='numbers of frames jumped'
)
arg = parser.parse_args()

# ------------------------------------------------------------------
# Read simulated data
# ------------------------------------------------------------------
experiment_directory = find_latest_timestamp('output/')
os.makedirs(experiment_directory + 'snapshots', exist_ok=True)
print('Experiment located in: {}'.format(experiment_directory))

t = np.loadtxt(experiment_directory + 'logger_time.csv', delimiter=',')
arg.end = t[-1] if (arg.end == 0) else arg.end
N = len(t) // arg.jump

# slices
k_i = int(np.argmin(np.abs(t - arg.init)))
k_e = int(np.argmin(np.abs(t - arg.end)))

time_reader = csv.reader(
    open(experiment_directory + 'logger_time.csv', newline='')
)
ids_reader = csv.reader(
    open(experiment_directory + 'logger_ids.csv', newline='')
)
positions_reader = csv.reader(
    open(experiment_directory + 'logger_positions.csv', newline='')
)
comm_ranges_reader = csv.reader(
    open(experiment_directory + 'logger_comm_ranges.csv', newline='')
)
status_reader = csv.reader(
    open(experiment_directory + 'logger_status.csv', newline='')
)


# ------------------------------------------------------------------
# Plot snapshots
# ------------------------------------------------------------------
bar = progressbar.ProgressBar(maxval=N).start()

for _ in range(k_i):
    next(time_reader)
    next(ids_reader)
    next(positions_reader)
    next(comm_ranges_reader)
    next(status_reader)

k = 1
while k < k_e:
    try:
        # Read one row from each file
        time = float(next(time_reader)[0])
        ids = list(next(ids_reader))
        positions = np.array(
            next(positions_reader), dtype=float
        ).reshape(-1, 2)
        comm_ranges = np.array(
            next(comm_ranges_reader), dtype=float
        ).reshape(-1, 1)
        status = list(next(status_reader))

        lim = 100.0
        fig, ax = plt.subplots(figsize=(3.0, 3.0))
        ax.tick_params(
            axis='both',       # changes apply to the x-axis
            which='both',      # both major and minor ticks are affected
            pad=1,
            labelsize='x-small')
        ax.grid(1, lw=0.4)
        ax.set_aspect('equal')
        ax.set_xlabel(r'$x \ (\mathrm{m})$', fontsize='x-small', labelpad=0.6)
        ax.set_ylabel(r'$y \ (\mathrm{m})$', fontsize='x-small', labelpad=0.6)

        ax.set_xlim(0.0, lim)
        ax.set_ylim(0.0, lim)

        ax.text(
                0.05, 0.01, r't = {:.3f}s'.format(time - 0.1),
                verticalalignment='bottom', horizontalalignment='left',
                transform=ax.transAxes, color='r', fontsize='x-small'
        )

        hunters = [i for i, id in enumerate(ids) if id.startswith('Hunter')]
        targets = [
            i for i, id in enumerate(ids)
            if id.startswith('Target') and status[i] == 'active'
        ]

        for hunter in hunters:
            plot.nodes(
                ax, positions[hunter],
                color='b',
                # marker='o',
                marker=f'${robot_id_to_index(ids[hunter])}$',
                s=15,
                lw=0.2
            )

        for target in targets:
            plot.nodes(
                ax, positions[target],
                color='k',
                # marker='d',
                marker=f'${robot_id_to_index(ids[target])}$',
                s=15,
                lw=0.2
            )
        for target in targets:
            circle = plt.Circle(
                positions[target], 5.0, fill=True, color='blue', alpha=0.2
            )
            ax.add_patch(circle)

        edges = DiskGraph(
            positions[hunters], dmax=comm_ranges[hunters[0]]
        ).edge_set()
        plot.edges(
            ax,
            positions[hunters],
            edges,
            color='0.0',
            alpha=0.5,
            lw=0.3,
            zorder=0
        )

        fig.savefig(
            experiment_directory + 'snapshots/{}.png'.format(k),
            format='png',
            dpi=360
        )
        plt.close()

        # Skip rows in each file
        for _ in range(arg.jump - 1):
            next(time_reader)
            next(ids_reader)
            next(positions_reader)
            next(comm_ranges_reader)
            next(status_reader)

        bar.update(k)
        k += 1
    except StopIteration:
        break  # Stop if any reader runs out of lines

#     untracked = targets[k][:, 2].astype(bool)
#     tracked = np.logical_not(untracked)
#     ax.scatter(
#         targets[k][untracked, 0], targets[k][untracked, 1],
#         marker='s', s=4, color='0.6')
#     ax.scatter(
#         targets[k][tracked, 0], targets[k][tracked, 1],
#         marker='s', s=4, color='green')


bar.finish()
