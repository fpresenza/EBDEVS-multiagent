import numpy as np

from uvnpy.network.core import geodesics
from uvnpy.network.subframeworks import superframework_extents
from uvnpy.distances.core import minimum_rigidity_extents

from files import read_json_file, write_json_file
from core import robot_id_to_index


robots_config = read_json_file('robots.json')
n_robots = len(robots_config)
position = np.zeros((n_robots, 2))
comm_range = np.zeros(n_robots)

for robot_id, config in robots_config.items():
    i = robot_id_to_index(robot_id)
    position[i] = np.ravel(config['position'])
    comm_range[i] = config['comm_range']

adjacency_matrix = np.zeros((n_robots, n_robots))
for i in range(n_robots):
    for j in range(n_robots):
        if i != j:
            dist = np.sqrt(np.sum(np.square(position[i] - position[j])))
            adjacency_matrix[i, j] = (dist < comm_range[i]) and (dist < comm_range[j])

geodesic_matrix = geodesics(adjacency_matrix)
action_extents = minimum_rigidity_extents(geodesic_matrix, position)
state_extents = superframework_extents(geodesic_matrix, action_extents)

for robot_id, config in robots_config.items():
    i = robot_id_to_index(robot_id)
    config['token_handler']['action'] = int(action_extents[i])
    config['token_handler']['state'] = int(state_extents[i])

write_json_file('robots.json', robots_config)