#
import sys

# Make py2pdevs package visible. An alternative is to export the
# environment variable PYTHONPATH appeding the absolute path to build/lib
sys.path.append('../')

from pypdevs.DEVS import CoupledDEVS

# Import all models to couple
# from generators import PeriodicGenerator, SinusoidalGenerator 
from atomics.router import Router, TokenGenerator

class TestRouterSystem(CoupledDEVS):
    def __init__(self, name='RouterSystem'):
        """
        A unit test for the Kalman Filter atomic model.
        """
        # Always call parent class' constructor FIRST:
        CoupledDEVS.__init__(self, name)

        # Declare the coupled model's sub-models:
        self.router   = self.addSubModel(Router(number_of_robots=3, name='Router'))
        self.router_input_generator = self.addSubModel(TokenGenerator(number_of_robots=3, period=1,name='Router_Input_Gen'))
        self.connectPorts(self.router_input_generator.out_router_token['0'], self.router.in_agent_token['0'])
        self.connectPorts(self.router_input_generator.out_router_token['1'], self.router.in_agent_token['1'])
        self.connectPorts(self.router_input_generator.out_router_token['2'], self.router.in_agent_token['2'])