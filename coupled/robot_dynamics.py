import numpy as np  # noqa

from pypdevs.DEVS import CoupledDEVS
from atomics.integrators.qss1tools import pad_zeros
from atomics.integrators.qss1integrator import QSS1Integrator_Yup
from atomics.misc.misc import Merger  # noqa
from atomics.dynamics.single_integrator import SingleIntegrator


class RobotDynamics(CoupledDEVS):
    def __init__(
            self,
            position,
            config,
            name='RobotDynamics',
            debug=False):
        """
          Robot's dynamic model composed of two integrators for x and y
          and a dynamics function.
        """
        # Always call parent class' constructor FIRST:
        CoupledDEVS.__init__(self, name)

        self.debug = debug

        # dictionary to save childrens' states
        self.y_up = [
            self.name,
            {
                'time': 0.0,
                'pose': [pad_zeros(coord) for coord in position],
            }
        ]
        self.current_time = 0

        # Declare childrens:
        dynamics_function = SingleIntegrator(
            num_outputs=2,
            debug=self.debug
        )
        merger = Merger(
            num_inputs=2,
            debug=self.debug
        )
        integrator_x = QSS1Integrator_Yup(
            name="x",
            **config['x'],
            x0=position[0][0],
            debug=self.debug
        )
        integrator_y = QSS1Integrator_Yup(
            name="y",
            **config['y'],
            x0=position[1][0],
            debug=self.debug
        )

        self.dynamics_function = self.addSubModel(dynamics_function)
        self.merger = self.addSubModel(merger)
        self.integrator_x = self.addSubModel(integrator_x)
        self.integrator_y = self.addSubModel(integrator_y)

        # Declare the coupled model's output ports:
        self.inPorts = {
            'control_input': self.addInPort(name="in_control_input")
            }
        self.outPorts = {
            'position_polynomial':
            self.addOutPort(name="out_position_polynomial")
            }

        # Connect coupled model's input with dynamics_function's input
        self.connectPorts(
            self.inPorts['control_input'],
            self.dynamics_function.inPorts['control_action']
        )
        # Connect dynamics_function's output with integrator's input
        self.connectPorts(
            self.dynamics_function.outPorts[0], self.integrator_x.IN_dx
        )
        self.connectPorts(
            self.dynamics_function.outPorts[1], self.integrator_y.IN_dx
        )
        # Connect integrators with merger's input
        self.connectPorts(self.integrator_x.OUT_q, self.merger.inPorts[0])
        self.connectPorts(self.integrator_y.OUT_q, self.merger.inPorts[1])
        # Connect merger's output with coupled model's output
        self.connectPorts(
            self.merger.outPort, self.outPorts['position_polynomial']
            )
        # Connect merger's output with coupled model's output
        self.connectPorts(
            self.merger.outPort, self.dynamics_function.inPorts['state']
            )

        if (self.debug):
            print("t: 0 s, Coupled name: {}, Init Function"
                  .format(self.name))

    def globalTransition(self, e_g, x_b_micro, *args, **kwargs):
        # update each coordinate separatedly, since the events in the
        # integrators for the same robot do not need to be
        # simultaneous
        # self.current_time += e_g

        micro_id, children_time, data = x_b_micro[0]

        self.y_up[1]['time'] = children_time
        if micro_id == 'x':
            self.y_up[1]['pose'][0] = data.copy()
        elif micro_id == 'y':
            self.y_up[1]['pose'][1] = data.copy()

        if (self.debug):
            print("t: {:.2f} s, Coupled name: {}, \
                   Global Transition Function, x_b_micro: {}"
                  .format(children_time, self.name, x_b_micro))

    def select(self, immChildren):
        """
        Choose a model to transition from all possible models.
        """
        # Doesn't really matter, as they don't influence each other
        return immChildren[0]
