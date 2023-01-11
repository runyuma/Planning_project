from nonlinear_mpc.obstacle import box


# 表示1 | Representation 1
# 用于MPC约束中计算障碍的hyperplane表示
# Used for representing hyperplane obstacles in MPC constraints
def getObstacleInfoSimple():
    
    obstacles_simple = []
    obstacles_simple.append(box(2.0, -2.5, 0, 12.0, 1.0)) # 下面
    obstacles_simple.append(box(-0.25, 3.0, 0, 7.5, 3.0)) # 中左
    obstacles_simple.append(box(13.0, 3.0,0, 6.0, 3.0))   # 中右
    obstacles_simple.append(box(6.0, 5.0,0, 20.0, 1.0))   # 上

    return obstacles_simple

# 表示2 | Representation 2
# 用于Hybrid A-star中的障碍地图构建
# Used for building the obstacle map
def my_design_obstacles_simple(reso):
    # args:
    #   reso: expected resolution of 
    ox, oy = [], []

    # the simple obstacled environment

    for i in np.arange(-4.0,3.5,reso):
        ox.append(i)
        oy.append(1.5)

    for i in np.arange(10.0,16.0,reso):
        ox.append(i)
        oy.append(1.5)
    
    for i in np.arange(-4.0,8.0,reso):
        ox.append(i)
        oy.append(-2.0)

    for i in np.arange(1.5,4.5,reso):
        ox.append(3.5)
        oy.append(i)

    for i in np.arange(1.5,4.5,reso):
        ox.append(10.0)
        oy.append(i)

    for i in np.arange(3.5,10.0,reso):
        ox.append(i)
        oy.append(4.5)

    for i in np.arange(1.5,5.5,reso):
        ox.append(-4.0)
        oy.append(i)

    for i in np.arange(1.5,5.5,reso):
        ox.append(16.0)
        oy.append(i)

    for i in np.arange(-3.0,-2.0,reso):
        ox.append(-4.0)
        oy.append(i)

    for i in np.arange(-3.0,-2.0,reso):
        ox.append(8.0)
        oy.append(i)

    for i in np.arange(-4.0,8.0,reso):
        ox.append(i)
        oy.append(-3.0)

    for i in np.arange(-4.0,16.0,reso):
        ox.append(i)
        oy.append(5.5)


    return ox,oy