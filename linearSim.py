import numpy as np

class LinearSystemSim:
    def __init__(self, system, measurement_std_devs=None):
        self.m_plant = system
        self.m_measurement_std_devs = measurement_std_devs
        self.m_x = np.zeros((system.get_A().shape[0], 1))
        self.m_u = np.zeros((system.get_B().shape[1], 1))
        self.m_y = np.zeros((system.get_C().shape[0], 1))

    def update(self, dt_seconds):
        # Update X. By default, this is the linear system dynamics X = Ax + Bu
        self.m_x = self.update_X(self.m_x, self.m_u, dt_seconds)

        # y = cx + du
        self.m_y = self.m_plant.calculate_Y(self.m_x, self.m_u)

        # Add measurement noise.
        if self.m_measurement_std_devs is not None:
            self.m_y += self.make_white_noise_vector(self.m_measurement_std_devs)

    def get_output(self):
        return self.m_y

    def get_output_element(self, row):
        return self.m_y[row, 0]

    def set_input(self, u):
        self.m_u = self.clamp_input(u)

    def set_input_element(self, row, value):
        self.m_u[row, 0] = value
        self.m_u = self.clamp_input(self.m_u)

    def set_input_array(self, u):
        if len(u) != self.m_u.shape[0]:
            raise ValueError("Malformed input! Got {} elements instead of {}".format(len(u), self.m_u.shape[0]))
        self.m_u = np.array(u).reshape((-1, 1))

    def set_state(self, state):
        self.m_x = state

    def get_current_draw_amps(self):
        return 0.0

    def update_X(self, current_x_hat, u, dt_seconds):
        return self.m_plant.calculate_X(current_x_hat, u, dt_seconds)

    def clamp_input(self, u):
        return np.clip(u, -12, 12)

    @staticmethod
    def make_white_noise_vector(std_devs):
        return np.random.normal(0, std_devs, std_devs.shape)
