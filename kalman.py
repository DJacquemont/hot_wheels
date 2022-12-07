import numpy as np
import pygame as pg
import vision


def move(_x: float, _y: float, psi: float, w: int, h: int, obj, surface):
    """ move function: draw image of Thymio at its current state estimate
    Inputs:
        _x - estimated x-position of Thymio [m]
        _y - estimated y-position of Thymio [m]
        psi - estimated orientation of Thymio [rad]
        w - width of window given in surface [pixels]
        h - height of window given in surface [pixels]
        obj - pygame image object of Thymio
        surface - pygame window object
    """
    # Convert x,y to pixels
    x = int(_x * 1000 - 25) / 2
    y = int(h - _y * 1000 - 25) / 2
    # Draw Thymio image on surface
    thymio = pg.transform.rotate(obj, psi*180/np.pi)
    surface.blit(thymio, (x, y))


def drawLine(_x: list, _y: list, w: int, h: int, surface):
    """ drawLine function: draw trailing line indicating previous estimated positions of Thymio
    Inputs:
        _x - estimated x-positions of Thymio [m]
        _y - estimated y-positions of Thymio [m]
        w - width of window given in surface [pixels]
        h - height of window given in surface [pixels]
        surface - pygame window object
    """
    count = 1
    n_prev = 1000
    while count < n_prev and count <= len(_x):
        # Convert x,y to pixels
        x = int(_x[len(_x)-count] * 1000) / 2
        y = int(h - _y[len(_x)-count] * 1000) / 2
        # Draw dots with fading red coloring
        surface.fill(( int(255*(1-count/n_prev)),0,0 ), ((x, y), (3, 3)))
        count += 1


def path(optimal_path, w, h, surface):
    """ path function: draws the optimal path in a window
    Inputs:
        optimal_path - coordinates of the optimal path nodes [m]
        w - width of window given in surface [pixels]
        h - height of window given in surface [pixels]
        surface - pygame window object
    """
    for i in range(len(optimal_path)):
        pg.draw.circle(surface, (0, 255, 0), (int(optimal_path[i][0] * 1000), int(h - optimal_path[i][1] * 1000)), 5)


def init_state(vid):
    fetched = False
    i = 0
    pose, orientation = (None, None)
    while (not fetched) and (i < 100):
        odoMet = vision.fetchOdoMeters(vid)
        if type(odoMet) != bool:
            pose = odoMet[0:2]
            orientation = odoMet[2]
            fetched = True
        i += 1
    return pose, orientation


class Kalman:
    def __init__(self, _acc_variance: float, _type, init_x: np.array) -> None:
        self.type = _type

        # mean of state GRV
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

        try:
            new_x = A.dot(self._x)
            new_P = A.dot(self._P).dot(A.T) + Q
            self._x = new_x
            self._P = new_P
        except:
            print("Could not update state prediction")

    def update(self, meas_value: np.array, meas_variance: np.array, C : np.array) -> None:
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
            P = self._P - self._P.dot(C.T).dot(Sinv).dot(C).dot(self._P)
        elif self.type == "orientation":
            Sinv = 1 / S
            K = self._P.dot(C.T) * Sinv
            P = self._P - self._P.dot(C.T).dot(C)*self._P*Sinv
        else:
            K = None
            P = None

        try:
            new_x = self._x + K.dot(y)

            self._P = P
            self._x = new_x
        except:
            print("Could not update state estimate")

    @property
    def cov(self) -> np.array:
        return self._P

    @property
    def mean(self) -> np.array:
        return self._x


