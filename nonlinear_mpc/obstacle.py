import numpy
import numpy as np
import matplotlib.pyplot as plt
class box:
    def __init__(self, x, y, yaw, length, width):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.length = length
        self.width = width
        self.box = np.zeros((2, 4))
        # clockwise
        self.box[0, :] = [x - np.cos(yaw)*length/2 + np.sin(yaw)*width/2, x - np.cos(yaw)*length/2 - np.sin(yaw)*width/2,
                          x + np.cos(yaw)*length/2 - np.sin(yaw)*width/2, x  + np.cos(yaw)*length/2 + np.sin(yaw)*width/2]
        self.box[1, :] = [y - np.cos(yaw)*width/2-np.sin(yaw)*length/2, y + np.cos(yaw)*width/2-np.sin(yaw)*length/2,
                          y + np.cos(yaw)*width/2+np.sin(yaw)*length/2, y - np.cos(yaw)*width/2+np.sin(yaw)*length/2]
        # print(self.box,)
    def plot(self, ax):
        # print(self.box)
        ax.plot(self.box[0, :], self.box[1, :], "b")
        ax.plot([self.box[0, -1],self.box[0, 0]], [self.box[1, -1],self.box[1, 0]], "b")

    def find_hyperplane(self, robot_state, ref, r):
        dists2D = self.box - np.array([robot_state.x, robot_state.y]).reshape(2, 1)
        dists = np.linalg.norm(dists2D, axis=0)
        point_num = np.argmin(dists)
        point_num_next = (point_num + 1) % 4
        h = numpy.zeros(3)
        if dists[point_num] > 10:
            return None
        _delta = (self.box[:,point_num_next]-self.box[:, point_num])
        start_angle = np.arctan2(_delta[1], _delta[0])
        point = ref[:2]
        _dist = point - self.box[:, point_num]
        dist_angle = np.arctan2(_dist[1], _dist[0])
        if pi_2_pi(dist_angle - start_angle) > 0 and pi_2_pi(dist_angle - start_angle) < np.pi/2:
            # print(1)
            fixed_point = self.box[:, point_num]+r*np.array([np.cos(start_angle+np.pi/2), np.sin(start_angle+np.pi/2)])
            if abs(np.tan(start_angle)) < 1:
                h[1] = -1 * np.sign(_dist[1])
                h[0] = -np.tan(start_angle) * h[1]
                h[2] = -h[0] * fixed_point[0] - h[1] * fixed_point[1]
            else:
                h[0] = -1 * np.sign(_dist[0])
                h[1] = -np.tan(np.pi/2-start_angle) * h[0]
                h[2] = -h[1] * fixed_point[1] - h[0] * fixed_point[0]
        elif pi_2_pi(dist_angle - start_angle) < 0 and pi_2_pi(dist_angle - start_angle) > -np.pi/2:
            print("not possible")
        elif pi_2_pi(dist_angle - start_angle) > np.pi/2:
            fixed_point = self.box[:, point_num] + r * np.array(
                [np.cos(dist_angle), np.sin(dist_angle)])
            if abs(np.tan(dist_angle+np.pi/2))<1:
                h[1] = -1 * np.sign(_dist[1])
                h[0] = -np.tan(dist_angle+np.pi/2) * h[1]
                h[2] = -h[0] * fixed_point[0] - h[1] * fixed_point[1]
            else:
                h[0] = -1 * np.sign(_dist[0])
                h[1] = -np.tan(-dist_angle) * h[0]
                h[2] = -h[1] * fixed_point[1] - h[0] * fixed_point[0]
            # print(2)
        elif pi_2_pi(dist_angle - start_angle) < -np.pi/2:
            fixed_point = self.box[:, point_num] + r * np.array(
                [np.cos(start_angle+np.pi), np.sin(start_angle + np.pi)])
            if abs(np.tan(start_angle- np.pi / 2)) < 1:
                h[1] = -1 * np.sign(_dist[1])
                h[0] = -np.tan(start_angle- np.pi / 2) * h[1]
                h[2] = -h[0] * fixed_point[0] - h[1] * fixed_point[1]
            else:
                h[0] = -1 * np.sign(_dist[0])
                h[1] = -1/np.tan(start_angle- np.pi / 2) * h[0]
                h[2] = -h[1] * fixed_point[1] - h[0] * fixed_point[0]
            # print(3)
        # h = h / np.linalg.norm(h)
        plt.scatter(fixed_point[0], fixed_point[1], c="g")
        # print(point_num, start_angle, dist_angle, pi_2_pi(dist_angle - start_angle),fixed_point)
        print(h[0] * robot_state.x + h[1] * robot_state.y + h[2])
        return h
    def draw_hyperplane(self,h,ax):
        if h is not None:
            x1 = self.x - 10
            x2 = self.x + 10
            if abs(h[1]) > 1e-3:
                y1 = (-h[0] * x1 - h[2]) / h[1]
                y2 = (-h[0] * x2 - h[2]) / h[1]
            else:
                y1 = -(h[2] + h[0] * x1) / (h[1]+0.001)
                y2 = -(h[2] + h[0] * x2) / (h[1]+0.001)
            ax.plot([x1, x2], [y1, y2], "r")


class circle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius
        self.circle = np.zeros((2, 100))
        for i in range(100):
            self.circle[0, i] = x + radius * np.cos(i * 2 * np.pi / 100)
            self.circle[1, i] = y + radius * np.sin(i * 2 * np.pi / 100)
    def plot(self, ax):
        ax.plot(self.circle[0, :], self.circle[1, :], "b")
    def find_hyperplane(self,robot_state,ref,r):
        # find the hyperplane that is tangent to the circle
        # r is the point on the circle
        # h is the normal vector of the hyperplane
        h = np.zeros(3)
        # dx = self.x-robot_state.x
        # dy = self.y- robot_state.y
        dx = self.x - ref[0]
        dy = self.y- ref[1]
        lenth = np.linalg.norm([dx,dy])
        if lenth > 10:
            return None
        fixed_point = np.array([self.x,self.y])-(r+self.radius)/lenth*np.array([dx,dy])
        # d_angle = np.arctan2(dy,dx)-robot_state.yaw
        # sign = np.sign(np.sin(d_angle))
        # fixed_point2 = np.array([self.x, self.y]) + r*np.array([np.cos(robot_state.yaw-np.pi/2*sign), np.sin(robot_state.yaw-np.pi/2*sign)])
        # dx2 = self.x-ref[0][-1]
        # dy2 = self.y-ref[1][-1]
        # fixed_point2 = np.array([self.x, self.y]) - (r + self.radius) / np.linalg.norm([dx2,dy2])* np.array([dx, dy])
        # h[0] = -(fixed_point2[0]-fixed_point[0])
        # h[1] = (fixed_point2[1]-fixed_point[1])
        # h[2] = -h[0]*fixed_point[0]-h[1]*fixed_point[1]
        if abs(dx)>abs(dy):
            h[0] = 1*np.sign(dx)
            h[1] = +dy/dx*h[0]
            h[2] = -h[1]*fixed_point[1]-h[0]*fixed_point[0]
        else:
            h[1] = 1*np.sign(dy)
            h[0] = +dx/dy*h[1]
            h[2] = -h[1]*fixed_point[1]-h[0]*fixed_point[0]

        # print(np.sign(h[0]*robot_state.x+h[1]*robot_state.y+h[2]))
        return h
    def draw_hyperplane(self,h,ax):
        if h is not None:
            x1 = self.x - 10
            x2 = self.x + 10
            y1 = -(h[2] + h[0] * x1) / h[1]
            y2 = -(h[2] + h[0] * x2) / h[1]
            ax.plot([x1, x2], [y1, y2], "r")
def pi_2_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi
if __name__ == "__main__":
    a = box(0, 0, 1, 2, 4)
    b = circle(0, 1, 1)
    box.plot(a, plt)
    circle.plot(b, plt)
    plt.show()