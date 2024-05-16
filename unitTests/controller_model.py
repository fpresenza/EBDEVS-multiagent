#
import sys

# Make py2pdevs package visible. An alternative is to export the
# environment variable PYTHONPATH appeding the absolute path to build/lib
sys.path.append('../')

from pypdevs.DEVS import CoupledDEVS

# Import all models to couple
# from generators import PeriodicGenerator, SinusoidalGenerator 
from atomics.controller import Controller, ControllerGenerator

class TestControllerSystem(CoupledDEVS):
    def __init__(self, name='ControllerSystem'):
        """
        A unit test for the Controller atomic model.
        """
        # Always call parent class' constructor FIRST:
        CoupledDEVS.__init__(self, name)

        # Declare the coupled model's sub-models:
        self.controller = self.addSubModel(Controller(robot_id='2',name='Controller'))
        self.controller_input_generator = self.addSubModel(ControllerGenerator(period=1,name='Controller_Input_Gen'))
        self.connectPorts(self.controller_input_generator.out_controller_extact, self.controller.in_handler_extact)
        self.connectPorts(self.controller_input_generator.out_controller_extpos, self.controller.in_handler_extpos)
        self.connectPorts(self.controller_input_generator.out_controller_intpos, self.controller.in_kalman_intpos)

