#
import sys

# Make py2pdevs package visible. An alternative is to export the
# environment variable PYTHONPATH appeding the absolute path to build/lib
sys.path.append('../')

from pypdevs.DEVS import CoupledDEVS

# Import all models to couple
# from generators import PeriodicGenerator, SinusoidalGenerator 
from atomics.misc import Splitter, SplitterGenerator

class TestSplitterSystem(CoupledDEVS):
    def __init__(self, name='ControllerSystem'):
        """
        A unit test for the Splitter atomic model.
        """
        # Always call parent class' constructor FIRST:
        CoupledDEVS.__init__(self, name)

        # Declare the coupled model's sub-models:
        self.splitter = self.addSubModel(Splitter(numoutputs=2,name='Splitter'))
        self.splitter_input_generator = self.addSubModel(SplitterGenerator(period=1,name='Splitter_Input_Gen'))

        self.connectPorts(self.splitter_input_generator.out_splitter_in, self.splitter.in_splitter_msgs)

