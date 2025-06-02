import numpy as np

from atomics.localization.localization import Localization
from uvnpy.distances.localization import DistanceBasedKalmanFilter


class DKFilter(object):
    def __init__(self, robot_id, config):
        self.robot_id = robot_id
        self.dim = config['dim']
        ekf_config = {
            key: np.array(val)
            for key, val in config['ekf'].items()
        }
        self.ekf = DistanceBasedKalmanFilter(**ekf_config)

    def get_estimation(self):
        """
            Get the latest estimation.
        """
        return self.ekf.position()


class DistanceKalmanFilter(Localization):
    def set_loc_filter(self, robot_id, config):
        #
        #    define localization filter here
        #
        return DKFilter(robot_id, config)

    def set_in_port_names(self):
        #
        #    define the list of input ports name here
        #
        return [
            'velocity_measurement',
            'position_measurement',
            'neighbors_positions'
        ]

    def process_inputs(self, sigma, current_time, loc_filter, inputs):
        #
        #    process inputs here
        #
        port, data = inputs.popitem()

        if port == self.inPorts['velocity_measurement']:
            # if data arrives through port inPorts['velocity_measurement']
            vel_meas = data
            loc_filter.ekf.dynamic_step(
                current_time,
                np.reshape(vel_meas, (-1, 1))
            )
            sigma = 0.0  # holds last status
        elif port == self.inPorts['position_measurement']:
            # if data arrives through port inPorts['position_measurement']
            pos_meas = np.reshape(data, (-1, 1))
            loc_filter.ekf.gps_step(pos_meas)
            sigma = 0.0  # holds last status
        elif port == self.inPorts['neighbors_positions']:
            #  if token arrives through port inPorts['neighbors_positions']
            _, neighbor_pos, dist_meas = data
            loc_filter.ekf.range_step(
                dist_meas,
                np.reshape(neighbor_pos, (-1, 1)),
                np.zeros((loc_filter.dim, loc_filter.dim), dtype=float)
            )
            sigma = 0.0  # holds last status

        return sigma, loc_filter

    def loc_filter_results(self, loc_filter):
        #
        #    get estimation here
        #
        estimation = loc_filter.get_estimation()
        return estimation, None
