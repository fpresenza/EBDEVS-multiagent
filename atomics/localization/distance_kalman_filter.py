import numpy as np

from atomics.localization.localization import Localization
from uvnpy.distances.localization import FirstOrderKalmanFilter


class DistanceKalmanFilter(Localization):
    def set_loc_filter(self, config):
        #
        #    define localization filter here
        #
        return FirstOrderKalmanFilter(
            position=np.array(config['position']).ravel(),
            position_cov=np.array(config['covariance']),
            vel_meas_cov=np.array([[0.0, 0.0], [0.0, 0.0]]),
            range_meas_cov=np.array([[1.0]]),
            gps_meas_cov=np.array([[1.0, 0.0], [0.0, 1.0]])
        )

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
            loc_filter.dynamic_step(
                current_time,
                np.ravel(vel_meas)
            )
            sigma = 0.0  # holds last status
        elif port == self.inPorts['position_measurement']:
            # if data arrives through port inPorts['position_measurement']
            pos_meas = np.reshape(data, (2, 1))
            loc_filter.gps_step(pos_meas)
            sigma = 0.0  # holds last status
        elif self.inPorts['neighbors_positions'] in inputs:
            #  if token arrives through port inPorts['neighbors_positions']
            _, neighbor_pos, dist_meas = data
            loc_filter.range_step(
                dist_meas,
                np.ravel(neighbor_pos),
                np.zeros((2, 2), dtype=float)
            )
            sigma = 0.0  # holds last status

        return sigma, loc_filter

    def loc_filter_results(self, loc_filter):
        #
        #    get estimation here
        #
        return loc_filter.position(), None
