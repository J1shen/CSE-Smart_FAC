from typing import Union, Tuple, List
from scipy.spatial import KDTree
import numpy as np
import copy
import random
from Constants import Constants
from Order import Order, Market
from CraftTable import CraftTable
import math

class Robot:
    def __init__(self, robot_id, at_table, possession, time_factor, collision_factor,
                 angle_velocity, linear_velocity_x, linear_velocity_y, angle, x, y):
        self.id = robot_id
        self.at_table = at_table
        self.possession = possession
        self.time_factor = time_factor
        self.collision_factor = collision_factor
        self.angle_velocity = angle_velocity
        self.linear_velocity_x = linear_velocity_x
        self.linear_velocity_y = linear_velocity_y
        self.angle = angle
        self.x = x
        self.y = y
        self.order: List[Order] = []

    def estimate_gain(self, order: Order, target_market: Market, craft_tables: List[CraftTable]) \
            -> Tuple[float, Union[Order, None]]:
        if order.content[0] == 'b':
            # 当该订单为买入订单时(机器人售出)，该订单可以给机器人带来的收益仅仅是订单的价格
            return order.price, None
        else:
            # 机器人买入物品的收益是市场上“买卖价差减去机器人对卖出该物体所需的成本的估计”的最大值
            profits: List[float] = [0.0]
            buy_orders: List[Union[Order, None]] = [None]
            for buy_order in target_market.orders:
                if buy_order.content[0] == 'b' and buy_order.content[1] == order.content[1] \
                        and buy_order.status == Order.ORDER_HANGUP:
                    # 找到了对应的购买订单
                    profits.append(buy_order.price - self.estimate_cost(buy_order, target_market, craft_tables))
                    buy_orders.append(buy_order)
            return max(profits) - order.price, buy_orders[profits.index(max(profits))]

    def estimate_cost(self, order: Order, target_market: Market, craft_tables: List[CraftTable]) -> float:
        target_position = (craft_tables[order.owner].x, craft_tables[order.owner].y)
        estimate_time = self.time_estimate(target_position)
        current_time_factor = self.time_factor
        # 估计再过estimate_time后的系数
        if math.fabs(current_time_factor) < 1e-6:
            # 机器人没有持有物品--认为处于第0帧
            current_time_factor = 1.0
        current_frame = 9000 * (1 - math.sqrt(1 - (1 - current_time_factor) ** 2 / (1 - 0.8) ** 2))
        estimate_arrive_time = current_frame + estimate_time
        if estimate_arrive_time < 9000:
            estimate_factor = 0.8 + 0.2 * (1 - math.sqrt(1 - (1 - estimate_arrive_time / 9000) ** 2))
        else:
            estimate_factor = 0.8
        # 成本即为卖出价乘以时间系数之差
        return Constants.OBJECT_BUY_PRICE[order.content[1] - 1] * (current_time_factor - estimate_factor)

    def motion_control(self, target_position) -> Tuple[float, float]:
        # TODO 给定目标点，计算机器人的线速度与角速度
        if self.x > 49 or self.x < 1 or self.y > 49 or self.y < 1:
            v =1
            av = 0
            if self.x > 49:
            # 边界，开始转向
                av = math.pi
            if self.x < 1:
                av = 0
            if self.y > 49:
                av = - math.pi/2
            if self.y < 1:
                av = math.pi/2
            return v, av
        if len(self.order) == 0:
            # 无订单时，机器人不运动
            return 0, 0

        target_x = target_position[0]
        target_y = target_position[1]

        # DEBUG
        v = 0
        av = 0

        d_vec = (target_x - self.x, target_y - self.y)
        d = math.sqrt(d_vec[0] ** 2 + d_vec[1] ** 2)
        v_vec = (self.linear_velocity_x, self.linear_velocity_y)
        v = math.sqrt(v_vec[0] ** 2 + v_vec[1] ** 2)
        if v < 1e-6:
            return 0.1, 0.0
        if d < 1e-6:
            return v, self.angle_velocity
        product = v_vec[0]*d_vec[0] + v_vec[1]*d_vec[1]
        flag_product = v_vec[0]*d_vec[1] - v_vec[1]*d_vec[0]
        cos_theta = product / (d * v)
        theta = math.acos(cos_theta) * (1 if flag_product > 0 else -1)

        av = theta / 0.02
        if theta > abs(math.pi/2):
            v = 3.5
        elif theta == 0:
            v = self.angle_velocity
        else:
            v = 6.0
        if d < 1.27:
            v = 2
        elif d < 0.4:
            v = 0.8

        linear_velocity, angle_velocity = v, av

        return linear_velocity, angle_velocity


    def time_estimate(self, target_position) -> int:
        # TODO 给定目标点，估计机器人到达目标点所需时间，单位为帧
        target_x = target_position[0]
        target_y = target_position[1]
        # pass
        # estimate_time = 0.0
        # debug 使用简单算法
        d_vec = (target_x - self.x, target_y - self.y)
        d = math.sqrt(d_vec[0] ** 2 + d_vec[1] ** 2)
        v_vec = (self.linear_velocity_x, self.linear_velocity_y)
        v = math.sqrt(v_vec[0] ** 2 + v_vec[1] ** 2)
        angle_v = self.angle_velocity
        if angle_v < 1e-6 or v < 1e-6:
            return 2*int(d / 3)
        if d < 1e-6:
            return 0
        cos_theta = (d_vec[0] * v_vec[0] + d_vec[1] * v_vec[1]) / (d * v)
        estimate_time = 2*abs(math.acos(cos_theta)/angle_v) * 50
        return int(estimate_time)

    # def time_estimate(self, target_position) -> float:
    #     # TODO 给定目标点，估计机器人到达目标点所需时间，单位为帧
    #     target_x = target_position[0]
    #     target_y = target_position[1]
    #     dist = math.dist((self.x,self.y),(target_x,target_y))
    #     estimate_time = dist/10+10*math.log(100/(dist+100))
    #
    #     return int(estimate_time*50)

# class Node(object):
#     def __init__(self, x, y, cost = 0.0, parent = None):
#         self.x = x
#         self.y = y
#         self.cost = cost
#         self.parent = parent

# 为了避障，规划路线
def collision_control(robot0, robot1):
    X0 = robot0.x
    Y0 = robot0.y
    VX0 = robot0.linear_velocity_x
    VY0 = robot0.linear_velocity_y
    X1 = robot1.x
    Y1 = robot1.y
    VX1 = robot1.linear_velocity_x
    VY1 = robot1.linear_velocity_y
    rd = math.dist((X0, Y0),(X1, Y1)) # 相对距离
    rvx0 = VX0 - VX1  # 0相对于1的速度
    rvy0 = VY0 - VY1
    rv0 = math.sqrt(rvx0 * rvx0 + rvy0 * rvy0)
    rvx1 = VX1 - VX0  # 1相对于0的速度
    rvy1 = VY1 - VY0
    rv1 = math.sqrt(rvx1 * rvx1 + rvy1 * rvy1)
    if (rd < 6 and rv0 > 0):
        flag: int = 0
        r: float = 0
        if robot0.possession != 0:
            if robot1.possession != 0:
                r = 1.06
                flag = 1
            else:
                r = 0.98
                flag = 1
        else:
            if robot1.possession != 0:
                r = 0.98
                flag = 1
            else:
                r = 0.9
                flag = 1
        if (flag):
            # robotId0中心到robot1中心的单位向量
            ux01 = (X1 - X0) / rd
            uy01 = (Y1 - Y0) / rd
            # robotId1中心到robot2中心的单位向量
            ux10 = (X0 - X1) / rd
            uy10 = (Y0 - Y1) / rd
            # 计算切线与robot0中心连线的夹角
            # print("r = %f" % r)
            # print("rd = %f" % rd)
            angle = math.asin(r / rd)
            # 向正反两个方向旋转单位向量，逆时针为正，得到两条切线的向量（0到1）
            qx01_1 = ux01 * math.cos(angle) - uy01 * math.sin(angle)
            qy01_1 = ux01 * math.sin(angle) + uy01 * math.cos(angle)
            qx01_2 = ux01 * math.cos(-angle) - uy01 * math.sin(-angle)
            qy01_2 = ux01 * math.sin(-angle) + uy01 * math.cos(-angle)
            # 向正反两个方向旋转单位向量，逆时针为正，得到两条切线的向量（1到0）
            qx10_1 = ux10 * math.cos(angle) - uy10 * math.sin(angle)
            qy10_1 = ux10 * math.sin(angle) + uy10 * math.cos(angle)
            qx10_2 = ux10 * math.cos(-angle) - uy10 * math.sin(-angle)
            qy10_2 = ux10 * math.sin(-angle) + uy10 * math.cos(-angle)
            # 计算相对速度是不是在两条切线范围内，利用叉乘
            rvXq01_1 = (rvx0 * qy01_1 - rvy0 * qx01_1)
            rvXq01_2 = (rvx0 * qy01_2 - rvy0 * qx01_2)
            if (rvXq01_1 * rvXq01_2 < 0 and abs(rvXq01_1) < abs(rvXq01_2)):
                return 1  # 靠近向量1
            if (rvXq01_1 * rvXq01_2 < 0 and abs(rvXq01_1) > abs(rvXq01_2)):
                return 2  # 靠近向量2
            rvXq10_1 = (rvx1 * qy10_1 - rvy1 * qx10_1)
            rvXq10_2 = (rvx1 * qy10_2 - rvy1 * qx10_2)
            if (rvXq10_1 * rvXq10_2 < 0 and abs(rvXq10_1) < abs(rvXq10_2)):
                return 1  # 靠近向量1
            if (rvXq10_1 * rvXq10_2 < 0 and abs(rvXq10_1) > abs(rvXq10_2)):
                return 2  # 靠近向量2

    return -1


class collision_if(object):

    def collision_detect(self,robots:list[Robot],target_positions:list[Tuple[float, float]]):
        line_speed = []
        angle_speed = []
        i = 0
        for robot0 in robots:
            target_x = target_positions[i][0]
            target_y = target_positions[i][1]
            # path_x, path_y = self.plan(robot.id, robots, robot.x, robot.y, target_x, target_y)
            lv, av = robot0.motion_control(target_positions[i])
            # for robot1 in robots:
            #     if robot1 != robot0:
            #         # rob_pos0 = robot0.x, robot0.y
            #         # rob_pos1 = robot1.x, robot1.y
            #         if math.dist((robot0.x, robot0.y), (robot1.x, robot1.y)) < 6:
            #             if collision_control(robot0, robot1) > 0:
            #                 av = 0
            line_speed.append(lv)
            angle_speed.append(av)
            i = i + 1

        return line_speed, angle_speed


# class RRT(object):
#     def __init__(self, N_SAMPLE=50, N_ITERATION=500, STEP=5):
#         self.N_SAMPLE = N_SAMPLE
#         self.N_ITERATION = N_ITERATION
#         self.step = STEP
#         self.minx = 0
#         self.maxx = 50
#         self.miny = 0
#         self.maxy = 50
#         self.robot_size = 0.5
#         self.avoid_dist = 1.1
#         self.nodes = []
#
#     def getv(self, robots: List[Robot], target_positions: List[Tuple[float, float]]):
#         line_speed = []
#         angle_speed = []
#         i = 0
#         for robot in robots:
#             target_x = target_positions[i][0]
#             target_y = target_positions[i][1]
#             path_x, path_y = self.plan(robot.id, robots, robot.x, robot.y, target_x, target_y)
#             lv, av = robot.motion_control(target_positions[i])
#             line_speed.append(lv)
#             angle_speed.append(av)
#             i = i + 1
#
