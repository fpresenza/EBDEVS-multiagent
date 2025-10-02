#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

from multirobot_ebdevs.atomics.sensors.exteroceptive import ExteroceptiveSensor


class GPSSensor(ExteroceptiveSensor):
    def compute_measurement(self, current_time):
        #
        #  compute measurement here
        #
        p = self.parent.getRobotPose(current_time)[:2]
        noise = np.random.multivariate_normal(
            mean=self.noise_mean.ravel(),
            cov=self.noise_covariance
        )
        return [p[0][0] + noise[0], p[1][0] + noise[1]]
