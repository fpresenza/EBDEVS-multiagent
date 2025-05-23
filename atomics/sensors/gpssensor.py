import numpy as np
from atomics.sensors.exteroceptive import ExteroceptiveSensor


class GPSSensor(ExteroceptiveSensor):
    def compute_measurement(self, current_time, external_state):
        #
        #  compute measurement here
        #
        px = external_state[0][0]
        py = external_state[1][0]
        noise_sample = np.random.multivariate_normal(
            mean=self.noise_mean.ravel(),
            cov=self.noise_covariance
        )
        return [px + noise_sample[0], py + noise_sample[1]]
