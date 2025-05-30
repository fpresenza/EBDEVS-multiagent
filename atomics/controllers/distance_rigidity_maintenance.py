import numpy as np

from uvnpy.distances.control import RigidityMaintenance
from uvnpy.control.core import CollisionAvoidanceVanishing

from atomics.controllers.controller import Controller


class DRMControl(object):
    def __init__(self, config):
        self.subframework = {}
        self.obstacles = []
        self.target_position = None
        self.external_action = np.zeros((2, 1), dtype=float)
        self.rigidity = RigidityMaintenance(
            dim=2,
            dmax=config['dmax'][0],
            steepness=config['steepness'],
            threshold=1e-4,
            eigenvalues='all',
            functional='log'
        )
        self.collision = CollisionAvoidanceVanishing(
            power=2.0,
            dmin=1.0,
            dmax=config['dmax'][1]
        )

    def clear(self):
        self.subframework.clear()
        self.obstacles.clear()
        self.external_action[:] = 0.0

    def compute_action(self, robot_id):
        coordination_data = {}

        if robot_id in self.subframework:
            position = self.subframework[robot_id]

            # target collection
            if self.target_position is not None:
                r = position.reshape(-1, 1) - self.target_position
                d = np.sqrt(np.square(r).sum())
                tracking_radius = 20.0    # radius
                forget_radius = 100.0     # radius
                v_collect_max = 1.25
                if d < tracking_radius:
                    v_collect = v_collect_max
                elif d < forget_radius:
                    factor = (forget_radius - d)
                    factor /= (forget_radius - tracking_radius)
                    v_collect = v_collect_max * factor
                else:
                    v_collect = 0.0
                target_action = - v_collect * r / d
            else:
                target_action = np.zeros((2, 1), dtype=float)

            # obstacle avoidance
            if len(self.obstacles) > 0:
                collision_action = self.collision.update(
                    position, self.obstacles
                ).reshape(-1, 1)
                collision_action *= 0.25
            else:
                collision_action = np.zeros((2, 1), dtype=float)

            # rigidity maintenance
            rigidity_action = self.external_action

            if len(self.subframework) > 1:
                subframework_ids, subframework_positions = list(zip(
                    *self.subframework.items()
                ))
                subframework_actions = self.rigidity.update(np.array(
                    subframework_positions
                ))
                subframework_actions = {
                    node_id: action.reshape(-1, 1)
                    for node_id, action
                    in zip(subframework_ids, subframework_actions)
                }
                rigidity_action += subframework_actions.pop(robot_id)
                rigidity_action *= 0.375
                coordination_data = subframework_actions

            # compose control action
            control_action = target_action + collision_action + rigidity_action
        else:
            control_action = np.zeros((2, 1), dtype=float)

        return control_action, coordination_data


class DistanceRigidityMaintenance(Controller):
    def set_control(self, config):
        #
        #    define controller here
        #
        return DRMControl(config)

    def set_in_ports(self):
        #
        #    define input ports here
        #
        return {
            'position': self.addInPort(name="in_position"),
            'other_position': self.addInPort(name="in_other_position"),
            'external_action': self.addInPort(name="in_external_action"),
            'target_position': self.addInPort(name="in_target_position")
        }

    def process_inputs(self, sigma, current_time, control, inputs):
        #
        #    process inputs here
        #
        port, data = inputs.popitem()

        if port == self.inPorts['position']:
            control.subframework[self.robot_id] = data.ravel()

        elif port == self.inPorts['other_position']:
            node_id, other_position, hops = data
            control.subframework[node_id] = other_position.ravel()
            if hops == 1:
                control.obstacles.append(other_position.ravel())

        elif port == self.inPorts['external_action']:
            control.external_action += data[1]

        elif port == self.inPorts['target_position']:
            control.target_position = data

        return control

    def compute_action(self, control):
        #
        #    compute control action here
        #
        control_action, coordination_data = control.compute_action(
            self.robot_id
        )

        return control_action, coordination_data, None
