from atomics.localization.localization import Localization


class MyFilter(object):
    def __init__(self, robot_id, config):
        self.robot_id = robot_id

    def get_estimation():
        """
            Get the latest estimation.
        """
        estimation = None
        return estimation


class Template(Localization):
    def set_loc_filter(self, robot_id, config):
        #
        #    define localization filter here
        #
        return MyFilter(robot_id, config)

    def set_in_port_names(self):
        #
        #    define the list of input ports name here
        #
        return ['port_A', 'port_B']

    def process_inputs(self, sigma, current_time, loc_filter, port_name, data):
        #
        #    process inputs here
        #
        if port_name == 'port_A':
            pass
        elif port_name == 'port_B':
            pass

        return sigma, loc_filter

    def loc_filter_results(self, loc_filter):
        #
        #    get estimation here
        #
        estimation = loc_filter.get_estimation()
        return estimation, None
