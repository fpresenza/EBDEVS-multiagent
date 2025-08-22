#!/usr/bin/env python
# -*- coding: utf-8 -*-

from multirobot_ebdevs.utils.core import robot_id_to_index
from multirobot_ebdevs.utils.files import (
    find_latest_timestamp, read_jsonl_file, write_csv_file
)

# ------------------------------------------------------------------
# Read simulated data
# ------------------------------------------------------------------
experiment_directory = find_latest_timestamp('output/')
print('Experiment located in: {}'.format(experiment_directory))

adjacency_list = read_jsonl_file(experiment_directory + 'adjacency_list.jsonl')

# ------------------------------------------------------------------
# Parse and save
# ------------------------------------------------------------------
edge_list = [
    sum([
        sum([[robot_id_to_index(k), robot_id_to_index(m)] for m in v], [])
        for k, v in adj.items()
    ], [])
    for adj in adjacency_list
]

write_csv_file(
    experiment_directory + 'edge_list.csv', edge_list
)
