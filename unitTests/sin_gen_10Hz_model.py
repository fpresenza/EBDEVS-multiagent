#
import sys

# Make py2pdevs package visible. An alternative is to export the
# environment variable PYTHONPATH appeding the absolute path to build/lib
sys.path.append('../')

from pypdevs.DEVS import CoupledDEVS

# Import all models to couple
# from generators import PeriodicGenerator, SinusoidalGenerator 
from atomics.generators import SinusoidalGenerator, SinusoidalGenerator10Hz

class TestGeneratorsSystem(CoupledDEVS):
    def __init__(self, name='GeneratorsSystem'):
        """
        A unit test for sinusoidal generator 10Hz (derived from sinusoidal generator)
        """
        # Always call parent class' constructor FIRST:
        CoupledDEVS.__init__(self, name)

        # Declare the coupled model's sub-models:
        singen1Hz = SinusoidalGenerator(name='SinGen1Hz',freq=1, amp=1)
        singen10Hz = SinusoidalGenerator10Hz(name='SinGen10Hz')
        self.singen = self.addSubModel(singen1Hz)
        self.singen = self.addSubModel(singen10Hz)