import matplotlib.pyplot as plt
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
    # plt.plot(ref[0], ref[1], "black",marker="o")
    plt.scatter(ref[0], ref[1], c = "black")


    plt.show()