import numpy as np

class kinematic_bicycle_model():
    def __int__(self):
        pass
    def linear_discrete_model(self,v0,phi0,delta0,param):
        # for linear MPC, based on kinematic bicycle model
        # calculate A,B,C metrix x = Ax + Bu +C
        # x = [x,y,v,phi]; u = [a,delta]
        # linearize around v0,phi0,delta0
        # assume sin(beta) = tan(beta)
        dt = param["dt"]
        L = param["L"] # lr+lf
        A = np.array([[1.0, 0.0, dt * np.cos(phi0), - dt * v0 * np.sin(phi0)],
                      [0.0, 1.0, dt * np.sin(phi0), dt * v0 * np.cos(phi0)],
                      [0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, dt * np.tan(delta0) / L, 1.0]])

        B = np.array([[0.0, 0.0],
                      [0.0, 0.0],
                      [dt, 0.0],
                      [0.0, dt * v0 / (L * np.cos(delta0) ** 2)]])

        C = np.array([dt * v0 * np.sin(phi0) * phi0,
                      -dt * v0 * np.cos(phi0) * phi0,
                      0.0,
                      -dt * v0 * delta0 / (L * np.cos(delta0) ** 2)])

        return A, B, C
    def nonlinear_discrete_model(self,x,v,phi,delta,u,param):
        # for nonlinear MPC, based on kinematic bicycle model
        # x = [x,y]; u = [a,delta]
        dt = param["dt"]
        lr = param["lr"]
        L = param["L"]  # lr+lf
        tri = np.array([[np.cos(phi[0]+np.arctan(np.tan(u[1])*lr/L))],
                        [np.sin(phi[0]+np.arctan(np.tan(u[1])*lr/L))]])
        x_ = x + dt * tri @ v
        phi_ = phi + dt * v * np.sin(np.arctan(np.tan(u[1])*lr/L))/lr
        v_ = v+dt*u[0]
        delta_ = u[1]
        return x_,v_,phi_,delta_

if __name__ == '__main__':
    model = kinematic_bicycle_model()
