from atomics.controllers.controller import Controller


class MyControl(object):
    def __init__(self, robot_id, config):
        self.robot_id = robot_id

    def clear(self):
        """
            Clear whatever needs to be cleared after
            each control action is computed.
        """
        pass

    def compute_action(self):
        """
            Compute control action and coordination data if needed.
        """
        control_action = None
        coordination_data = None

        return control_action, coordination_data


class Template(Controller):
    def set_control(self, robot_id, config):
        #
        #    define controller here
        #
        return MyControl(robot_id, config)

    def set_in_port_names(self):
        #
        #    define the list of input ports name here
        #
        return ['port_A', 'port_B']

    def process_inputs(self, sigma, current_time, control, port_name, data):
        #
        #    process inputs here
        #
        if port_name == 'port_A':
            pass
        elif port_name == 'port_B':
            pass

        return control

    def compute_action(self, control):
        #
        #    compute control action here
        #
        control_action, coordination_data = control.compute_action()

        return control_action, coordination_data, None
