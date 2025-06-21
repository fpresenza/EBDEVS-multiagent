#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

from multirobot_ebdevs.atomics.sensors.exteroceptive import ExteroceptiveSensor


class GPSSensor(ExteroceptiveSensor):
    def compute_measurement(self, current_time):
        #
        #  compute measurement here
        #
        p = self.parent.getRobotPosition(current_time)
        noise_sample = np.random.multivariate_normal(
            mean=self.noise_mean.ravel(),
            cov=self.noise_covariance
        )
        return [p[0] + noise_sample[0], p[1] + noise_sample[1]]
