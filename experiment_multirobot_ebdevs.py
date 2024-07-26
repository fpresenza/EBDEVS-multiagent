# Copyright 2014 Modelling, Simulation and Design Lab (MSDL) at 
# McGill University and the University of Antwerp (http://msdl.cs.mcgill.ca/)
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Import code for model simulation:
from pypdevs.simulator import Simulator

# Import the model to be simulated
from multirobot_ebdevs import MultiRobotSystem

# Configuration (parameter sweeping)
#
# End of Configuration

# Store all results for output to file
# values = []

#    ======================================================================

# 1. Instantiate the (Coupled or Atomic) DEVS at the root of the 
#  hierarchical model. This effectively instantiates the whole model 
#  thanks to the recursion in the DEVS model constructors (__init__).
#
m = MultiRobotSystem(name="MultiRobotSystem", debug=True)

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
sim.setTerminationTime(1)

# B. Set the use of a tracer to show what happened during the simulation run
#    Both writing to stdout or file is possible:
#    pass None for stdout, or a filename for writing to that file
sim.setVerbose("output/simu_out.out")

# C. Use Classic DEVS instead of Parallel DEVS
#    If your model uses Classic DEVS, this configuration MUST be set as
#    otherwise errors are guaranteed to happen.
#    Without this option, events will be remapped and the select function
#    will never be called.
sim.setClassicDEVS()

#    ======================================================================

# 4. Simulate the model
sim.simulate()

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
# print("Simulation terminated with traffic light in state %s" % (trafficSystem.trafficLight.state.get()))
