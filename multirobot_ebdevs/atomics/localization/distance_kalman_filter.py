#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np

from uvnpy.distances.localization import DistanceBasedKalmanFilter as DBKF
from multirobot_ebdevs.atomics.localization.localization import Localization


class DistanceBasedKalmanFilter(Localization):
    def set_loc_filter(self, config):
        #
        #    define localization filter here
        #
        config = {
            key: np.array(val)
            for key, val in config.items()
        }
        return DBKF(**config)

    def set_in_port_names(self):
        #
        #    define the list of input ports name here
        #
        return [
            'velocity_measurement',
            'position_measurement',
            'neighbors_positions'
        ]

    def process_inputs(self, sigma, current_time, port_name, data):
        #
        #    process inputs here
        #
        if port_name == 'velocity_measurement':
            # if data arrives through port inPorts['velocity_measurement']
            self.loc_filter.dynamic_step(
                current_time,
                self.loc_filter.last_vel_meas
            )
            self.loc_filter.last_vel_meas = np.reshape(data, (-1, 1))
            sigma = 0.0  # holds last status
        elif port_name == 'position_measurement':
            # if data arrives through port inPorts['position_measurement']
            pos_meas = np.reshape(data, (-1, 1))
            self.loc_filter.gps_step(pos_meas)
            sigma = 0.0  # holds last status
        elif port_name == 'neighbors_positions':
            #  if token arrives through port inPorts['neighbors_positions']
            _, neighbor_pos, dist_meas = data
            self.loc_filter.range_step(
                dist_meas,
                np.reshape(neighbor_pos, (-1, 1)),
                np.zeros(
                    (self.loc_filter.dim, self.loc_filter.dim), dtype=float
                )
            )
            sigma = 0.0  # holds last status

        return sigma

    def results(self):
        #
        #    get estimation here
        #
        _, _ = self.state.get()

        estimation = self.loc_filter.position()
        return estimation, None
