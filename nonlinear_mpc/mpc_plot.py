import matplotlib.pyplot as plt
import numpy as np
def plot_velprof(x,y,yaw,speed_prof):
    lenth = len(x)
    for i in range(lenth-1):
        if speed_prof[i]>0:
            plt.plot([x[i],x[i+1]],[y[i],y[i+1]],"r")
        elif speed_prof[i]<0:
            plt.plot([x[i], x[i + 1]], [y[i], y[i + 1]], "g")
        elif speed_prof[i] == 0:
            plt.plot([x[i], x[i + 1]], [y[i], y[i + 1]], "bo")
    plt.show()
def plot_ref(x,y,yaw,ref,speed_prof=None):
    lenth = len(x)
    if speed_prof == None:
        plt.plot(x, y, "r")
    else:
        for i in range(lenth - 1):
            if speed_prof[i] > 0:
                plt.plot([x[i], x[i + 1]], [y[i], y[i + 1]], "r")
            elif speed_prof[i] < 0:
                plt.plot([x[i], x[i + 1]], [y[i], y[i + 1]], "g")
            elif speed_prof[i] == 0:
                plt.plot([x[i], x[i + 1]], [y[i], y[i + 1]], "bo")
    plt.plot(ref[0], ref[1], "black",marker="^")
    plt.scatter(ref[0], ref[1], c = "black")
def plot_mpc(x,y,yaw,ref,x_pred,speed_prof=None):
    lenth = len(x)
    plt.plot(x, y, "r")
    # if speed_prof == None:
    #     plt.plot(x, y, "r")
    # else:
    #     for i in range(lenth - 1):
    #         if speed_prof[i] > 0:
    #             plt.plot([x[i], x[i + 1]], [y[i], y[i + 1]], "r")
    #         elif speed_prof[i] < 0:
    #             plt.plot([x[i], x[i + 1]], [y[i], y[i + 1]], "g")
    #         elif speed_prof[i] == 0:
    #             plt.plot([x[i], x[i + 1]], [y[i], y[i + 1]], "bo")
    # plt.plot(ref[0], ref[1], "black",marker="o")

    plt.scatter(ref[0], ref[1], c = "black")
    plt.scatter([i[0] for i in x_pred], [i[1] for i in x_pred], c="yellow")
def draw_car(x,y,yaw,steer,color='black'):
    # draw car
    RF = 2  # [m] distance from rear to vehicle front end of vehicle
    RB = 2  # [m] distance from rear to vehicle back end of vehicle
    W = 2.4  # [m] width of vehicle
    WD = 0.7 * W  # [m] distance between left-right wheels
    WB = 2.5  # [m] Wheel base
    TR = 0.44  # [m] Tyre radius
    TW = 0.7  # [m] Tyre width

    car = np.array([[-RB, -RB, RF, RF, -RB],
                    [W / 2, -W / 2, -W / 2, W / 2, W / 2]])

    wheel = np.array([[-TR, -TR, TR, TR, -TR],
                      [TW / 4, -TW / 4, -TW / 4, TW / 4, TW / 4]])

    rlWheel = wheel.copy()
    rrWheel = wheel.copy()
    frWheel = wheel.copy()
    flWheel = wheel.copy()

    Rot1 = np.array([[np.cos(yaw), -np.sin(yaw)],
                     [np.sin(yaw), np.cos(yaw)]])

    Rot2 = np.array([[np.cos(steer), -np.sin(steer)],
                     [np.sin(steer), np.cos(steer)]])

    frWheel = np.dot(Rot2, frWheel)
    flWheel = np.dot(Rot2, flWheel)

    frWheel += np.array([[WB/2], [-WD / 2]])
    flWheel += np.array([[WB/2], [WD / 2]])
    rrWheel += np.array([[-WB/2], [-WD / 2]])
    rlWheel += np.array([[-WB/2], [WD / 2]])

    frWheel = np.dot(Rot1, frWheel)
    flWheel = np.dot(Rot1, flWheel)

    rrWheel = np.dot(Rot1, rrWheel)
    rlWheel = np.dot(Rot1, rlWheel)
    car = np.dot(Rot1, car)

    frWheel += np.array([[x], [y]])
    flWheel += np.array([[x], [y]])
    rrWheel += np.array([[x], [y]])
    rlWheel += np.array([[x], [y]])
    car += np.array([[x], [y]])

    plt.plot(car[0, :], car[1, :], color)
    plt.plot(frWheel[0, :], frWheel[1, :], color)
    plt.plot(rrWheel[0, :], rrWheel[1, :], color)
    plt.plot(flWheel[0, :], flWheel[1, :], color)
    plt.plot(rlWheel[0, :], rlWheel[1, :], color)
