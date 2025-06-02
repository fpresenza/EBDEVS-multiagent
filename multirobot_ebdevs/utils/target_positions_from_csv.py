import numpy as np
import argparse
import csv

from files import write_json_file


parser = argparse.ArgumentParser(description='')
parser.add_argument(
    '-f', '--file',
    default='', type=str, help='file'
)
arg = parser.parse_args()

with open(arg.file, newline='') as csvfile:
    reader = csv.reader(csvfile)
    initial_config = list(reader)[0]
    target_positions = np.array(initial_config).reshape(-1, 3)[:, :2]

targets_config = {}
for k, p in enumerate(target_positions):
    targets_config['Target_{}'.format(k)] = {
        "position": [
            [
                float(p[0])
            ],
            [
                float(p[1])
            ]
        ],
        "dynamics": {
            "x": {
                "dQMin": 0.05,
                "dQRel": 0.0,
                "gain": 1.0
            },
            "y": {
                "dQMin": 0.05,
                "dQRel": 0.0,
                "gain": 1.0
            }
        },
        "control": {
            "period": 0.1,
        },
        "coordination": {
            "position": [
                [
                    float(p[0])
                ],
                [
                    float(p[1])
                ]
            ],
            "collect_range": 5.0
        },
        "comm_range": 1e6,
    }

write_json_file('targets.json', targets_config)
