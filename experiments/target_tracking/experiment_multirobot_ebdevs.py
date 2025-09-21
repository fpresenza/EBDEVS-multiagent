#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import datetime
import argparse
import time

# Import code for model simulation:
from pypdevs.simulator import Simulator

# Import the model to be simulated
from multirobot_ebdevs.coupled.target_tracking.multirobot_ebdevs_unicycle import (
    MultiRobotSystem
)
from multirobot_ebdevs.utils.files import read_json_file, write_json_file

# Configuration (parameter sweeping)
#
# End of Configuration

# Store all results for output to file
# values = []

# ------------------------------------------------------------------
# Parse args
# ------------------------------------------------------------------
parser = argparse.ArgumentParser(description='')
parser.add_argument(
    '-t', '--time',
    default=1.0, type=float, help='simulation time length in seconds'
)
parser.add_argument(
    '-l', '--logger',
    default=1.0, type=float, help='logger period in seconds'
)
parser.add_argument(
    '-v', '--verbose',
    default=False, type=bool, help='save all events in a .out file'
)
arg = parser.parse_args()


# ------------------------------------------------------------------
# Create folder with timestamp
# ------------------------------------------------------------------
timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
log_path = 'output/' + timestamp + '/'
os.makedirs(log_path)
world_config = read_json_file('world.json')
simu_config = read_json_file('simu.json')
robots_config = read_json_file('robots.json')
write_json_file(log_path + 'world.json', world_config)
write_json_file(log_path + 'simu.json', simu_config)
write_json_file(log_path + 'robots.json', robots_config)

#    ======================================================================

# 1. Instantiate the (Coupled or Atomic) DEVS at the root of the
#  hierarchical model. This effectively instantiates the whole model
#  thanks to the recursion in the DEVS model constructors (__init__).
#
m = MultiRobotSystem(
    world_config,
    simu_config,
    robots_config,
    name="MultiRobotSystem",
    log_period=arg.logger,
    log_path=log_path,
    debug=False
)

#    ======================================================================

# 2. Link the model to a DEVS Simulator:
#  i.e., create an instance of the 'Simulator' class,
#  using the model as a parameter.
sim = Simulator(m)

#    ======================================================================

# 3. Perform all necessary configurations, the most commonly used are:

# A. Termination time (or termination condition)
#    Using a termination condition will execute a provided function at
#    every simulation step, making it possible to check for certain states
#    being reached.
#    It should return True to stop simulation, or Falso to continue.
# TODO: Add this condition
# def terminate_whenStateIsReached(clock, model):
#    return model.generatorLight.state.get() == "manual"
# sim.setTerminationCondition(terminate_whenStateIsReached)

#    A termination time is prefered over a termination condition,
#    as it is much simpler to use.
#    e.g. to simulate until simulation time 400.0 is reached
sim.setTerminationTime(arg.time)

# B. Set the use of a tracer to show what happened during the simulation run
#    Both writing to stdout or file is possible:
#    pass None for stdout, or a filename for writing to that file
if arg.verbose:
    sim.setVerbose(log_path + "simu_out.out")

# C. Use Classic DEVS instead of Parallel DEVS
#    If your model uses Classic DEVS, this configuration MUST be set as
#    otherwise errors are guaranteed to happen.
#    Without this option, events will be remapped and the select function
#    will never be called.
sim.setClassicDEVS()

#    ======================================================================

# 4. Simulate the model
a = time.perf_counter()
sim.simulate()
b = time.perf_counter()
print("Elapsed Time: {} sec".format(b - a))

# Gather information for output
# evt_list = m.collector.state.events
# values.append([e.queueing_time for e in evt_list])

# Write data to file
# with open('output.csv', 'w') as f:
#     for i in range(num):
#         f.write("%s" % i)
#         for j in range(len(values)):
#             f.write(", %5f" % (values[j][i]))
#         f.write("\n")

#    ======================================================================

# 5. (optional) Extract data from the simulated model
# TODO: Add this condition
# print(
#     "Simulation terminated with traffic light in state %s"
#     % (trafficSystem.trafficLight.state.get())
# )
