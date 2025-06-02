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
    robot_positions = np.array(initial_config).reshape(-1, 2)

robots_config = {}
for k, p in enumerate(robot_positions):
    robots_config['Robot_{}'.format(k)] = {
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
        "controller": {
            "period": 0.1,
            "dmax": [
                12.75,
                15.0
            ],
            "steepness": 2.0
        },
        "kalman_filter": {
            "position": [
                [
                    float(p[0])
                ],
                [
                    float(p[1])
                ]
            ],
            "covariance": [
                [
                    1.0,
                    0.0
                ],
                [
                    0.0,
                    1.0
                ]
            ]
        },
        "token_handler": {
            "action": 1,
            "state": 2
        },
        "target_handler": {
            "period": 1.0
        },
        "comm_range": 15.0,
        "gps_sensor": {
            "enabled": False,
            "bias": [
                [
                    0.0
                ],
                [
                    0.0
                ]
            ],
            "covariance": [
                [
                    1.0,
                    0.0
                ],
                [
                    0.0,
                    1.0
                ]
            ],
            "period": 1.0
        },
        "speed_sensor": {
            "bias": [
                [
                    0.0
                ],
                [
                    0.0
                ]
            ],
            "covariance": [
                [
                    0.0,
                    0.0
                ],
                [
                    0.0,
                    0.0
                ]
            ],
            "period": 0.1
        }
    }

write_json_file('robots.json', robots_config)
