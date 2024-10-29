import numpy as np

from pypdevs.DEVS import CoupledDEVS
from atomics.qssintegrators import QSSIntegrator_Yup
from atomics.misc import Splitter
from atomics.speedsensor import SpeedSensor
from atomics.gpssensor import GPSSensor
# TODO: Speed and GPS sensors should be outside dynamics coupled


class RobotDynamics(CoupledDEVS):
    def __init__(
        self, 
        position,
        config,
        name='RobotDynamics', 
        enable_GPS='False', 
        debug=False):
        """
        Robot's dynamic model composed of two integrators for x and y and a splitter.
        """
        # Always call parent class' constructor FIRST:
        CoupledDEVS.__init__(self, name)

        self.debug = debug

        # dictionary to save childrens' states
        self.y_up = [
            self.name, 
            {
                'time': 0.0, 
                'pose': [coord + [0.0] * 9 for coord in position], 
            }
        ]
        self.current_time = 0

        # Declare childrens: splitterx2, QSS integ x 2
        splitter     = Splitter(
            name="splitter",
            numoutputs=2,
            debug=self.debug
        )
        integrator_x = QSSIntegrator_Yup(
            name="x", 
            **config['x'],
            x0=position[0][0], 
            debug=self.debug
        )
        integrator_y = QSSIntegrator_Yup(
            name="y",
            **config['y'],
            x0=position[1][0], 
            debug=self.debug
        )
        speed_sensor = SpeedSensor(
            bias=np.zeros((2, 1), dtype=float),
            transformation_matrix=np.eye(2, dtype=float),
            velocity_measurement_covariance=np.array([[0.15, 0.0], [0.0, 0.15]]),
            debug=self.debug
        )
        # speed_sensor = SpeedSensorDiff(name="vmeas",
        #                            period=0.1,
        #                            noisestd=0.0,
        #                            bias=np.zeros((2,1)),
        #                            transf=np.eye(2),
        #                            debug=self.debug
        #                            )
        if (enable_GPS):
            gps_sensor = GPSSensor(
                name="GPS",
                noisecov=np.zeros((2, 2), dtype=float),
                bias=np.ones((2, 1), dtype=float),
                period=1,
                debug=self.debug
        )
        self.splitter     = self.addSubModel(splitter)
        self.integrator_x = self.addSubModel(integrator_x)
        self.integrator_y = self.addSubModel(integrator_y)
        self.speed_sensor = self.addSubModel(speed_sensor)
        if (enable_GPS):
            self.gps_sensor = self.addSubModel(gps_sensor)

        # Declare the coupled model's output ports:
        self.OUT_dynamics_x    = self.addOutPort(name="OUT_dynamics_x")
        self.OUT_dynamics_y    = self.addOutPort(name="OUT_dynamics_y")
        self.OUT_measured_v    = self.addOutPort(name="OUT_measured_v")
        self.IN_dynamics_vx_vy = self.addInPort( name="IN_dynamics_vx_vy")
        if (enable_GPS):
            self.OUT_measured_pos    = self.addOutPort(name="OUT_measured_pos")

        # Connect coupled model's ports with atomic models' ports
        self.connectPorts(self.IN_dynamics_vx_vy, self.splitter.in_splitter_msgs)
        self.connectPorts(self.splitter.out_splitter_msgs[0], self.integrator_x.IN_dx)
        self.connectPorts(self.splitter.out_splitter_msgs[1], self.integrator_y.IN_dx)
        self.connectPorts(self.integrator_x.OUT_q, self.OUT_dynamics_x)
        self.connectPorts(self.integrator_y.OUT_q, self.OUT_dynamics_y)
        ## SpeedSensor
        self.connectPorts(self.IN_dynamics_vx_vy, self.speed_sensor.in_commanded_speed)
        self.connectPorts(self.speed_sensor.out_measured_speed, self.OUT_measured_v)
        ## SpeedSensorDiff
        # self.connectPorts(self.integrator_x.OUT_q, self.speed_sensor.in_position_x)
        # self.connectPorts(self.integrator_y.OUT_q, self.speed_sensor.in_position_y)
        # self.connectPorts(self.speed_sensor.out_measured_speed, self.OUT_measured_v)
        if (enable_GPS):
            self.connectPorts(self.integrator_x.OUT_q, self.gps_sensor.in_x_pos)
            self.connectPorts(self.integrator_y.OUT_q, self.gps_sensor.in_y_pos)
            self.connectPorts(self.gps_sensor.out_meas_pos, self.OUT_measured_pos)

        if (self.debug):
            print("t: 0 s, Coupled name: {}, Init Function".format(self.name))

    def globalTransition(self, e_g, x_b_micro, *args, **kwargs):
        # update each coordinate separatedly, since the events in the integrators for the same robot do not need to be simultaneous
        # self.current_time += e_g

        micro_id, children_time, data = x_b_micro[0]

        self.y_up[1]['time'] = children_time
        if micro_id == 'x':
            self.y_up[1]['pose'][0] = data.copy()
        elif micro_id == 'y':
            self.y_up[1]['pose'][1] = data.copy()

        if (self.debug):
            # print("t: {} ms, I'm {} and I received this micro state {}".format(self.current_time,self.name,x_b_micro))
            print("t: {:.2f} s, Coupled name: {}, Global Transition Function, x_b_micro: {}".format(children_time, self.name, x_b_micro))

    def select(self, immChildren):
        """
        Choose a model to transition from all possible models.
        """
        # Doesn't really matter, as they don't influence each other
        return immChildren[0]