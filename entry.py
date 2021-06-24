# coding : utf-8

import numpy as np
import math
from sympy import diff, symbols

import matplotlib.pyplot as plt

import lattice_planner as lp

# 右手系，xyz。x对应s，y对应l
class AdjustmentControlPointStruct_xytheta(list):
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        super().__init__([x, y, np.tan(theta*np.pi/180)])
        self.x = self[0]
        self.y = self[1]
        self.theta = np.tan(self[2]*np.pi/180)


class AdjustmentControlPointStruct_txva(list):
    def __init__(self, t=0.0, x=0.0, v=0.0, a=0.0):
        super().__init__([t, x, v, a])
        self.t = self[0]
        self.x = self[1]
        self.v = self[2]
        self.a = self[3]




def makeAdjustmentLine(positions, states,
                       min_radius=0,
                       max_angle_velocity=40,
                       step=0.1):
    """
    1. 根据给定的起终点坐标(x,y,theta),以及车辆最小转弯半径/最大转动角速度,计算连接给定点的缓行道路线[暂时还没有利用上min_radius和max_angle_velocity]
    2. 根据给定的起终状态(t,x,vx,ax)，计算速度
    这两个多项式 其实是解偶的
    input:
      start_position: (x,y,theta) 起点位置
      end_position: (x,y,theta) 终点位置
      start_state: (t,x,vx,ax) 起点状态
      end_state: (t,x,vx,ax) 终点状态
      min_radius: 最小转弯半径 (m)，对应最大曲率
      max_angle_velocity: 最大角速度 (angle/s) 对应最大曲率变化
      step(离散距离)
    output:
      x_data: x方向上的路径
      y_data: y方向上的路径
      t_data: 时间序列
      vx_data: 与时间序列对应的x方向的速度
      vy_data: 与时间序列对应的y方向的速度
    """
    # position 先计算路径
    coef1, order1, error1 = lp.solveNOrderFunction(positions)
    print("-"*20)
    print("order: ", order1)
    print("coef: ")
    print(coef1)
    print("error: ", error1)
    print("-"*20)
    x_data = np.arange(positions[0].x, positions[-1].x, step * (1 if positions[0].x < positions[-1].x else -1))
    y_data = lp.getNOrderOutput(x_data, coef1, 0)

    # 将总v和总a映射到x方向上
    for i in range(len(states)):
        y_x_1_point = lp.getNOrderOutput([states[i][1]], coef1, 1)
        states[i][2] = np.cos(np.arctan(y_x_1_point[0])) * states[i][2]
        y_x_2_point = lp.getNOrderOutput([states[i][1]], coef1, 2)
        states[i][3] = np.cos(np.arctan(y_x_2_point[0])) * states[i][3]

    # state
    coef2, order2, error2 = lp.solveNOrderFunction(states)
    print("order: ", order2)
    print("coef: ")
    print(coef2)
    print("error: ", error2)
    print("-"*20)
    t_data = np.arange(states[0].t, states[-1].t, step)
    vx_data = lp.getNOrderOutput(t_data, coef2, 1)
    # dy/dt = dy/dx * dx/dt
    x_data_with_coef2 = lp.getNOrderOutput(t_data, coef2, 0)
    y_x_1_data = lp.getNOrderOutput(x_data_with_coef2, coef1, 1)
    vy_data = [y_x_1_data[i] * vx_data[i] for i in range(len(vx_data))]


    return x_data, y_data, t_data, vx_data, vy_data


def show_result(pos,x,y,t,vx,vy):
    plt.subplot(2,1,1)
    plt.plot(x, y)
    for p in pos:
        plt.plot(p[0], p[1], 'o')
    plt.axis('equal')


    plt.subplot(2,1,2)
    plt.plot(t,vx)
    plt.plot(t,vy)
    plt.axis('equal')
    plt.show()



if __name__ == "__main__":
    # # (start_x, start_y, strat_theta)
    # # (target_x, target_y, target_theta)
    # P0 = AdjustmentControlPointStruct_xytheta(0, 0, 0)
    # P1 = AdjustmentControlPointStruct_xytheta(10, 5, 0)
    # # (start_t, start_x, start_vx, start_ax)
    # # (target_t, target_x, target_vx, target_ax)
    # S0 = AdjustmentControlPointStruct_txva(0, 0, 1, 0)
    # S1 = AdjustmentControlPointStruct_txva(8, 10, 1, 0)
    # x, y, t, vx, vy = makeAdjustmentLine([P0, P1], [S0, S1])
    # show_result([P0, P1], x, y, t, vx, vy)






    P0 = AdjustmentControlPointStruct_xytheta(0, 0, 0)
    P1 = AdjustmentControlPointStruct_xytheta(1, 0, 0)
    P2 = AdjustmentControlPointStruct_xytheta(9, 5, 0)
    P3 = AdjustmentControlPointStruct_xytheta(10, 5, 0)
    S0 = AdjustmentControlPointStruct_txva(0, 0, 1, 0)
    S1 = AdjustmentControlPointStruct_txva(1, 1, 1)
    S2 = AdjustmentControlPointStruct_txva(9, 9, 1)
    S3 = AdjustmentControlPointStruct_txva(10, 10, 1, 0)
    x, y, t, vx, vy = makeAdjustmentLine([P0, P1, P2, P3], [S0, S1, S2, S3])
    show_result([P0, P1, P2, P3], x, y, t, vx, vy)

