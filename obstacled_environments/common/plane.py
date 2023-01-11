import pybullet as p
import os


class SmallPlane:
    def __init__(self):
        f_name = os.path.join(os.path.dirname(__file__), 'plane_small.urdf')
        p.loadURDF(f_name)


class LargePlane:
    def __init__(self):
        f_name = os.path.join(os.path.dirname(__file__), 'plane_large.urdf')
        p.loadURDF(f_name)

