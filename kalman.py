import numpy as np
import pygame as pg
import vision

def move(_x, _y, psi, w, h, obj, surface):
    x = int(_x*1000-25)
    y = int(h-_y*1000-25)
    thymio = pg.transform.rotate(obj, psi*180/np.pi)
    surface.blit(thymio, (x, y))


def drawLine(_x, _y, w, h, surface):
    count = 1
    while count < 300 and count <= len(_x):
        x = int(_x[len(_x)-count]*1000)
        y = int(h - _y[len(_x)-count]*1000)
        surface.fill((int(255-255/300*count), 0, 0), ((x, y), (3, 3)))
        count += 1


class Kalman:
    def __init__(self, _acc_variance: float, _type, init_x=False, videoFeed=False) -> None:
        self.type = _type

        # mean of state GRV
        if not init_x:
            if videoFeed:
                fetched = False
                i = 0
                while (not fetched) and (i < 1000):
                    odoMet = vision.fetchOdoMeters(videoFeed)
                    if type(odoMet) != bool:
                        pos = odoMet[0:2]
                        pos[0] = pos[0]
                        angle = odoMet[2]
                        fetched = True
                    i += 1
                if _type == "pose":
                    try:
                        self._x = np.hstack((pos, np.array([0, 0])))
                    except:
                        print("Did not find position")
                elif _type == "orientation":
                    try:
                        self._x = np.array([angle, 0])
                    except:
                        print("Did not find angle")
            else:
                print("Initial conditions or video feed needs to be specified.")
        else:
            self._x = np.array(init_x)

        self._acc_variance = _acc_variance

        # covariance of state GRV
        self._P = 0*np.eye(len(self._x))

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


# Initialize kalman filter
meas_variance_pos = np.array([[0.001, 0],
                              [0, 0.001]])
meas_variance_vel = np.array([[0.1, 0],
                              [0, 0.1]])
meas_variance_att = np.array([0.001])
meas_variance_omega = np.array([0.1])

