import numpy as np

class LinearSystem:
    def __init__(self, A, B, C, D):
        self.m_A = A
        self.m_B = B
        self.m_C = C
        self.m_D = D

    def get_A(self):
        return self.m_A

    def get_B(self):
        return self.m_B

    def get_C(self):
        return self.m_C

    def get_D(self):
        return self.m_D

    def calculate_X(self, current_x_hat, u, dt_seconds):
        return np.dot(self.m_A, current_x_hat) + np.dot(self.m_B, u) * dt_seconds

    def calculate_Y(self, current_x_hat, u):
        return np.dot(self.m_C, current_x_hat) + np.dot(self.m_D, u)
