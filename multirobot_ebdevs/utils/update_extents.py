import numpy as np

from uvnpy.network.core import geodesics
from uvnpy.network.subframeworks import superframework_geodesics
from uvnpy.distances.core import minimum_rigidity_extents

from files import read_json_file, write_json_file


world_config = read_json_file('world.json')
robots_config = read_json_file('robots.json')

n_hunters = 0
position = []
comm_range = []

for robot_id, config in world_config.items():
    if robot_id.startswith('Hunter'):
        n_hunters += 1
        position.append(np.ravel(config['position']))
        comm_range.append(config['comm_range'])

position = np.array(position)

adjacency_matrix = np.zeros((n_hunters, n_hunters))
for i in range(n_hunters):
    for j in range(n_hunters):
        if i != j:
            dist = np.sqrt(np.sum(np.square(position[i] - position[j])))
            adjacency_matrix[i, j] = (dist < comm_range[i])

geodesic_matrix = geodesics(adjacency_matrix)
action_extents = minimum_rigidity_extents(geodesic_matrix, position)
state_extents = superframework_geodesics(geodesic_matrix, action_extents)

i = 0
for robot_id, config in robots_config.items():
    if robot_id.startswith('Hunter'):
        config['coordinator']['action'] = int(action_extents[i])
        config['coordinator']['state'] = int(state_extents[i])
        i += 1

write_json_file('robots.json', robots_config)
