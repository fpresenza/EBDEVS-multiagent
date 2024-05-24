#
import sys

# Make py2pdevs package visible. An alternative is to export the
# environment variable PYTHONPATH appeding the absolute path to build/lib
sys.path.append('../')

from pypdevs.DEVS import CoupledDEVS

# Import all models to couple
# from generators import PeriodicGenerator, SinusoidalGenerator 
from atomics.kalman_filter import KalmanFilter, KalmanGenerator

class TestKalmanFilterSystem(CoupledDEVS):
    def __init__(self, name='KalmanFilterSystem'):
        """
        A unit test for the Kalman Filter atomic model.
        """
        # Always call parent class' constructor FIRST:
        CoupledDEVS.__init__(self, name)

        # Declare the coupled model's sub-models:
        self.kalman_filter   = self.addSubModel(KalmanFilter(robot_id='2',name='Kalman_Filter'))
        self.kalman_input_generator = self.addSubModel(KalmanGenerator(period=1,name='Kalman_Input_Gen'))
        self.connectPorts(self.kalman_input_generator.out_kalman_extpos, self.kalman_filter.in_handler_extpos)
        self.connectPorts(self.kalman_input_generator.out_kalman_intact, self.kalman_filter.in_control_intact)

