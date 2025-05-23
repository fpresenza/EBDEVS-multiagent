import numpy as np
from atomics.sensors.interoceptive import InteroceptiveSensor


class SpeedSensor(InteroceptiveSensor):
    def compute_measurement(self, current_time, internal_state):
        #
        #  compute measurement here
        #
        vx = internal_state[0][1]
        vy = internal_state[1][1]
        noise_sample = np.random.multivariate_normal(
            mean=self.noise_mean.ravel(),
            cov=self.noise_covariance
        )
        return [vx + noise_sample[0], vy + noise_sample[1]]
