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