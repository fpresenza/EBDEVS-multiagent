import numpy as np

from uvnpy.network.core import geodesics
from uvnpy.network.subframeworks import superframework_extents
from uvnpy.distances.core import minimum_rigidity_extents

from utils import (
    read_json_file,
    write_json_file,
    robot_id_to_index
)

robots_config = read_json_file('robots.json')
n_robots = len(robots_config)
position = np.zeros((n_robots, 2))
comm_range = np.zeros(n_robots)
for robot in robots_config.values():
    robot_id = robot['name']
    i = robot_id_to_index(robot_id)
    position[i] = [robot['x0'], robot['y0']]
    comm_range[i] = robot['comm_range']

adjacency_matrix = np.zeros((n_robots, n_robots))
for i in range(n_robots):
    for j in range(n_robots):
        if i != j:
            dist = np.sqrt(np.sum(np.square(position[i] - position[j])))
            adjacency_matrix[i, j] = (dist < comm_range[i]) and (dist < comm_range[j])

geodesic_matrix = geodesics(adjacency_matrix)
action_extents = minimum_rigidity_extents(geodesic_matrix, position)
state_extents = superframework_extents(geodesic_matrix, action_extents)

for robot in robots_config.values():
    robot_id = robot['name']
    i = robot_id_to_index(robot_id)
    robot['action_extent'] = int(action_extents[i])
    robot['state_extent'] = int(state_extents[i])

write_json_file('robots.json', robots_config)