import numpy as np
import argparse

from files import write_json_file
from core import target_id_to_index


parser = argparse.ArgumentParser(description='')
parser.add_argument(
    '-n', '--number',
    default=1, type=int, help='number_of_targets'
)
parser.add_argument(
    '-x', '--x',
    default=1.0, type=float, help='x_max'
)
parser.add_argument(
    '-y', '--y',
    default=1.0, type=float, help='y_max'
)
arg = parser.parse_args()

targets_config = {}
for i in range(arg.number):
    targets_config['Target_{}'.format(i)] = {
        "position": [
            [
                np.random.uniform(0.0, arg.x)
            ],
            [
                np.random.uniform(0.0, arg.y)
            ]
        ],
        "period": 1.0,
        "comm_range": 300.0,
        "collect_range": 30.0
    }

write_json_file('targets.json', targets_config)