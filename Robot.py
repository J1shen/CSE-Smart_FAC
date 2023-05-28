from typing import Union, Tuple, List

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
        target_x = target_position[0]
        target_y = target_position[1]

        # DEBUG
        # d_vec = (target_x - self.x, target_y - self.y)
        # d = math.sqrt(d_vec[0] ** 2 + d_vec[1] ** 2)
        # v_vec = (self.linear_velocity_x, self.linear_velocity_y)
        # v = math.sqrt(v_vec[0] ** 2 + v_vec[1] ** 2)
        # cross_product = v_vec[0]*d_vec[1] - v_vec[1]*d_vec[0]
        # if v < 1e-6:
        #     return 0.1, 0.0
        # if d < 1e-6:
        #     return v, self.angle_velocity
        # cos_theta = (d_vec[0] * v_vec[0] + d_vec[1] * v_vec[1]) / (d * v)
        # if v < 5:
        #     linear_velocity = v + 0.1
        # else:
        #     linear_velocity = v
        # angle_velocity = 2*v*math.sqrt(1-cos_theta**2)/d * (1 if cross_product > 0 else -1)
        # DEBUG
        linear_velocity, angle_velocity = 0.0, 0.0
        return linear_velocity, angle_velocity

    def time_estimate(self, target_position) -> int:
        # TODO 给定目标点，估计机器人到达目标点所需时间，单位为帧
        target_x = target_position[0]
        target_y = target_position[1]
        pass
        estimate_time = 0.0
        # debug 使用简单算法
        # d_vec = (target_x - self.x, target_y - self.y)
        # d = math.sqrt(d_vec[0] ** 2 + d_vec[1] ** 2)
        # v_vec = (self.linear_velocity_x, self.linear_velocity_y)
        # v = math.sqrt(v_vec[0] ** 2 + v_vec[1] ** 2)
        # angle_v = self.angle_velocity
        # if angle_v < 1e-6 or v < 1e-6:
        #     return 2*int(d / 3)
        # if d < 1e-6:
        #     return 0
        # cos_theta = (d_vec[0] * v_vec[0] + d_vec[1] * v_vec[1]) / (d * v)
        # estimate_time = 2*abs(math.acos(cos_theta)/angle_v) * 50
        return int(estimate_time)
