import numpy as np
import pygame as pg


def move(_x, _y, psi, w, h, obj, surface):
    x = int(w/2 + _x*1000 - 25)
    y = int(h/2 - _y*1000 - 25)
    thymio = pg.transform.rotate(obj, psi*180/np.pi)
    surface.blit(thymio, (x, y))


def drawLine(_x, _y, w, h, surface):
    count = 1
    while count < 300 and count <= len(_x):
        x = int(w/2 + _x[len(_x)-count]*1000)
        y = int(h/2 - _y[len(_x)-count]*1000)
        surface.fill((int(255-255/300*count), 0, 0), ((x, y), (3, 3)))
        count += 1


class Kalman:
    def __init__(self,  init_x: list, _acc_variance: float, _type) -> None:
        self.type = _type

        # mean of state GRV
        self._x = np.array(init_x)
        self._acc_variance = _acc_variance

        # covariance of state GRV
        self._P = 0*np.eye(len(init_x))

    def predict(self, dt: float) -> None:
        # x = Ax
        # P = A P At + Q
        if self.type == "pose":
            A = np.array([[1, 0, dt, 0],
                         [0, 1, 0, dt],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
            Q = np.array([[(dt ** 2 / 2) ** 2, 0, 0, 0],
                          [0, (dt ** 2 / 2) ** 2, 0, 0],
                          [0, 0, dt ** 2, 0],
                          [0, 0, 0, dt ** 2]]) * self._acc_variance ** 2
        elif self.type == "orientation":
            A = np.array([[1, dt],
                          [0, 1]])
            Q = np.array([[(dt ** 2 / 2) ** 2, 0],
                          [0, dt ** 2]]) * self._acc_variance ** 2
        else:
            A = None
            Q = None

        new_x = A.dot(self._x)
        new_P = A.dot(self._P).dot(A.T) + Q

        self._x = new_x
        self._P = new_P

    def update(self, meas_value: np.array, meas_variance: np.array, C : np.array):
        # y = z - Cx
        # S = C P Ct + R
        # K = P Ct S^-1
        # x = x + K y
        # P = (I - K H) P

        z = np.array(meas_value)
        R = meas_variance

        y = z - C.dot(self._x.T)
        S = C.dot(self._P).dot(C.T) + R

        if self.type == "pose":
            Sinv = np.linalg.inv(S)
            K = self._P.dot(C.T).dot(Sinv)
            new_P = self._P - self._P.dot(C.T).dot(Sinv).dot(C).dot(self._P)
        elif self.type == "orientation":
            Sinv = 1 / S
            K = self._P.dot(C.T) * Sinv
            new_P = self._P - self._P.dot(C.T).dot(C)*self._P*Sinv

        new_x = self._x + K.dot(y)

        self._P = new_P
        self._x = new_x

    @property
    def cov(self) -> np.array:
        return self._P

    @property
    def mean(self) -> np.array:
        return self._x

    @property
    def px(self) -> float:
        return self._x[0]

    @property
    def py(self) -> float:
        return self._x[1]

    @property
    def vx(self) -> float:
        return self._x[3]

    @property
    def vy(self) -> float:
        return self._x[4]


