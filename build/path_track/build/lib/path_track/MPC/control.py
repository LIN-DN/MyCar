import numpy as np
from scipy.optimize import minimize
import copy

def wraptopi(radian):
    while radian > np.pi:
        radian = radian - 2 * np.pi
    while radian < -np.pi:
        radian = radian + 2 * np.pi
    return radian


class Car_Dynamics:
    def __init__(self, x_0, y_0, v_0, psi_0, length, dt):
        self.dt = dt             # sampling time
        self.L = length          # vehicle length
        self.x = x_0
        self.y = y_0
        self.v = v_0
        self.psi = psi_0
        # self.state = np.array([[self.x, self.y, self.v, self.psi]]).T

    def move(self, accelerate, delta):
        x_dot = self.v*np.cos(self.psi)
        y_dot = self.v*np.sin(self.psi)
        v_dot = accelerate
        psi_dot = self.v*np.tan(delta)/self.L
        # print(f"速度{self.v}角速度{psi_dot}偏转角{np.rad2deg(delta), np.tan(delta)}")
        return np.array([[x_dot, y_dot, v_dot, psi_dot]]).T

    def update_state(self, state_dot):
        state = np.array([[self.x, self.y, self.v, self.psi]]).T
        state = state + self.dt*state_dot
        self.x = state[0,0]
        self.y = state[1,0]
        self.v = state[2,0]
        self.psi = wraptopi(state[3,0])

    
class MPC_Controller:
    def __init__(self):
        self.horiz = None
        self.R = np.diag([0.01, 0.01])                 # input cost matrix # 避免输出值过大(消耗能量)
        self.Rd = np.diag([0.1, 1.0])                # input difference cost matrix 避免连续两次输出差值过大(激变)
        self.Q = np.diag([10.0, 10.0, 0.5])            # state cost matrix  # 避免与目标值偏差过大(效果差)
        self.Qf = self.Q                               # state final matrix

    def mpc_cost(self, u_k, my_car, points):
        mpc_car = copy.copy(my_car)
        u_k = u_k.reshape(self.horiz, 2).T  # command
        z_k = np.zeros((3, self.horiz+1))   # state
    
        desired_state = points.T
        cost = 0.0
        test = []
        for i in range(self.horiz):
            state_dot = mpc_car.move(u_k[0,i], u_k[1,i])
            mpc_car.update_state(state_dot)
            z_k[:,i] = [mpc_car.x, mpc_car.y, mpc_car.psi]
            # print(f"输入{np.rad2deg(u_k[1,i])} 状态更新{np.rad2deg(mpc_car.psi)}")
            
            cost += np.sum(self.R@(u_k[:,i]**2))
            error = desired_state[:,i]-z_k[:,i]
            error[2] = wraptopi(error[2])
            # test.append(error[2])

            # print(f"error{np.rad2deg(error[2])}")
            cost += np.sum(self.Q@((error)**2))
            if i < (self.horiz-1):     
                cost += np.sum(self.Rd@((u_k[:,i+1] - u_k[:,i])**2))
        # print(f"error:{np.rad2deg(test[0])}")        
        return cost

    def optimize(self, my_car, points):
        self.horiz = points.shape[0]
        # 加速度和转向角的约束
        bnd = [(-2, 2),(-0.6, 0.6)]*self.horiz
        # print(bnd)
        result = minimize(self.mpc_cost, args=(my_car, points), x0 = np.zeros((2*self.horiz)), method='SLSQP', bounds = bnd)
        # print(result.x)
        return result.x[0],  result.x[1]


class Linear_MPC_Controller:
    def __init__(self,dt, L):
        self.horiz = None
        self.R = np.diag([0.01, 0.01])                 # input cost matrix
        self.Rd = np.diag([0.01, 1.0])                 # input difference cost matrix
        self.Q = np.diag([1.0, 1.0])                   # state cost matrix
        self.Qf = self.Q                               # state final matrix
        self.dt = dt   
        self.L = L                          

    def make_model(self, v, psi, delta):        
        # matrices
        # 4*4
        A = np.array([[1, 0, self.dt*np.cos(psi)         , -self.dt*v*np.sin(psi)],
                    [0, 1, self.dt*np.sin(psi)         , self.dt*v*np.cos(psi) ],
                    [0, 0, 1                           , 0                     ],
                    [0, 0, self.dt*np.tan(delta)/self.L, 1                     ]])
        # 4*2 
        B = np.array([[0      , 0                                  ],
                    [0      , 0                                  ],
                    [self.dt, 0                                  ],
                    [0      , self.dt*v/(self.L*np.cos(delta)**2)]])

        # 4*1
        C = np.array([[self.dt*v* np.sin(psi)*psi                ],
                    [-self.dt*v*np.cos(psi)*psi                ],
                    [0                                         ],
                    [-self.dt*v*delta/(self.L*np.cos(delta)**2)]])
        
        return A, B, C

    def mpc_cost(self, u_k, my_car, points):
        
        u_k = u_k.reshape(self.horiz, 2).T
        z_k = np.zeros((2, self.horiz+1))
        desired_state = points.T[0:2]
        cost = 0.0
        old_state = np.array([my_car.x, my_car.y, my_car.v, my_car.psi]).reshape(4,1)

        for i in range(self.horiz):
            delta = u_k[1,i]
            A,B,C = self.make_model(my_car.v, my_car.psi, delta)
            new_state = A@old_state + B@u_k + C
        
            z_k[:,i] = [new_state[0,0], new_state[1,0]]
            cost += np.sum(self.R@(u_k[:,i]**2))
            cost += np.sum(self.Q@((desired_state[:,i]-z_k[:,i])**2))
            if i < (self.horiz-1):     
                cost += np.sum(self.Rd@((u_k[:,i+1] - u_k[:,i])**2))
            
            old_state = new_state
        return cost

    def optimize(self, my_car, points):
        self.horiz = points.shape[0]
        bnd = [(-5, 5),(np.deg2rad(-60), np.deg2rad(60))]*self.horiz
        result = minimize(self.mpc_cost, args=(my_car, points), x0 = np.zeros((2*self.horiz)), method ='SLSQP', bounds = bnd)
        return result.x[0],  result.x[1]