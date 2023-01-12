from nonlinear_mpc.obstacle import box


# 表示1 | Representation 1
# 用于MPC约束中计算障碍的hyperplane表示
# Used for representing hyperplane obstacles in MPC constraints
def getObstacleInfoSimple():
    
    obstacles_simple = []

    return obstacles_simple

# 表示2 | Representation 2
# 用于Hybrid A-star中的障碍地图构建
# Used for building the obstacle map
def my_design_obstacles_simple(reso):
    # args:
    #   reso: expected resolution of 
    ox, oy = [], []


    return ox,oy