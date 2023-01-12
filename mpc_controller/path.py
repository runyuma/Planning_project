import numpy as np

class PATH:
    def __init__(self, cx, cy, cyaw):
        #cx,cy,cyaw list of x,y,yaw
        self.cx = cx
        self.cy = cy
        self.cyaw = cyaw
        self.length = len(cx)
        self.ind_old = 0

    def nearest_index(self, robo_state, param):
        """
        calc index of the nearest node in N steps
        :param robo_state: current information
        :return: nearest index, lateral distance to ref point
        """

        dx = [robo_state.x - x for x in self.cx[self.ind_old: (self.ind_old + param["N_IND"])]]
        dy = [robo_state.y - y for y in self.cy[self.ind_old: (self.ind_old + param["N_IND"])]]
        dist = np.hypot(dx, dy)
        # return x2+y2

        ind_in_N = int(np.argmin(dist))
        ind = self.ind_old + ind_in_N
        self.ind_old = ind

        rear_axle_vec_rot_90 = np.array([[np.cos(robo_state.yaw + np.pi / 2.0)],
                                         [np.sin(robo_state.yaw + np.pi / 2.0)]])

        vec_target_2_rear = np.array([[dx[ind_in_N]],
                                      [dy[ind_in_N]]])

        er = np.dot(vec_target_2_rear.T, rear_axle_vec_rot_90)
        er = er[0][0]

        theta = robo_state.yaw
        theta_p = self.cyaw[ind]
        theta_e = pi_2_pi(theta - theta_p)

        return ind, er,theta_e
def pi_2_pi(angle):
    if angle > np.pi:
        return angle - 2.0 * np.pi

    if angle < -np.pi:
        return angle + 2.0 * np.pi
    return angle