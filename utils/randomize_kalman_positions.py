import numpy as np
import argparse

from files import read_json_file, write_json_file
from core import robot_id_to_index


parser = argparse.ArgumentParser(description='')
parser.add_argument(
    '-d', '--stddev',
    default=0.0, type=float, help='standard deviation'
)
arg = parser.parse_args()

robots_config = read_json_file('robots.json')

for config in robots_config.values():
    config['kalman_filter']['position'] = np.random.normal(loc=config['position'], scale=arg.stddev).tolist()


write_json_file('robots.json', robots_config)