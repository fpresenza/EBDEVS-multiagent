from pypdevs.DEVS import CoupledDEVS

from atomics.communication.communication_module import CommunicationModule
from atomics.controllers.beacon import Beacon
from atomics.coordination.token_handlers import TargetCoordinator
from coupled.robot_dynamics import RobotDynamics


class Target(CoupledDEVS):
    def __init__(
            self,
            world_config,
            simu_config,
            target_config,
            name='Target',
            debug=False):
        """
        A target model composed of the target's pysics.
        """
        # Always call parent class' constructor FIRST:
        CoupledDEVS.__init__(self, name)

        # Parameters
        position = world_config['position']
        comm_range = world_config['comm_range']
        self.debug = debug

        self.y_up = [
            self.name,
            {
                'time': 0.0,
                'pose': [coord + [0.0] * 9 for coord in position],
                'comm_range': comm_range,
                'status': 'active',
            }
        ]
        self.current_time = 0

        dynamics = RobotDynamics(
            position=position,
            config=simu_config['qss'],
            debug=self.debug
        )
        communication_module = CommunicationModule(
            robot_id=self.name,
            forward=False,
            debug=self.debug,
        )
        controller = Beacon(
            robot_id=self.name,
            config=target_config['controller'],
            debug=self.debug
        )
        coordinator = TargetCoordinator(
            robot_id=self.name,
            config=target_config['coordinator'],
            debug=self.debug,
        )

        self.dynamics = self.addSubModel(dynamics)
        self.communication_module = self.addSubModel(communication_module)
        self.controller = self.addSubModel(controller)
        self.coordinator = self.addSubModel(coordinator)

        # Declare the coupled model's output ports:
        self.outPorts = {'radio': self.addOutPort(name="out_radio")}
        self.inPorts = {'radio': self.addInPort(name="in_radio")}

        self.connectPorts(
            self.inPorts['radio'],
            self.communication_module.inPorts['radio']
        )
        self.connectPorts(
            self.communication_module.outPorts['radio'],
            self.outPorts['radio']
        )

        self.connectPorts(
            self.communication_module.outPorts['token'],
            self.coordinator.inPorts['token']
        )
        self.connectPorts(
            self.coordinator.outPorts['token'],
            self.communication_module.inPorts['token']
        )

        self.connectPorts(
            self.controller.outPorts['beacon'],
            self.coordinator.inPorts['beacon']
        )

        if (self.debug):
            print("t: 0 s, Coupled name: {}, Init Function".format(self.name))

    def globalTransition(self, e_g, x_b_micro, *args, **kwargs):
        # self.current_time += e_g

        if len(x_b_micro) == 1:
            micro_id, data = x_b_micro[0]
        else:
            micro_id, data = x_b_micro

        if micro_id == 'RobotDynamics':
            self.y_up[1]['time'] = data['time']
            self.y_up[1]['pose'] = data['pose'].copy()
        elif micro_id == 'TargetCoordinator':
            self.y_up[1]['time'] = data['time']
            self.y_up[1]['status'] = data['status']

        if (self.debug):
            print(
                "t: {:.2f} s, Coupled name: {}, \
                Global Transition Function, x_b_micro: {}"
                .format(data['time'], self.name, x_b_micro)
            )

    def select(self, immChildren):
        """
        Choose a model to transition from all possible models.
        """
        # Doesn't really matter, as they don't influence each other
        return immChildren[0]
