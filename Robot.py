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
        self.order: Union[Order, None] = None

    def estimate_gain(self, order: Order, target_market: Market, craft_tables: List[CraftTable]) -> float:
        if order.content[0] == 'b':
            # 当该订单为买入订单时(机器人售出)，该订单可以给机器人带来的收益仅仅是订单的价格
            return order.price
        else:
            # 机器人买入物品的收益是市场上“买卖价差减去机器人对卖出该物体所需的成本的估计”的最大值
            profits: List[float] = []
            for buy_order in target_market.orders:
                if buy_order.content[0] == 'b' and buy_order.content[1] == order.content[1]:
                    # 找到了对应的购买订单
                    profits.append(buy_order.price - self.estimate_cost(buy_order, target_market, craft_tables))
            return max(profits) - order.price

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
        pass
        linear_velocity, angle_velocity = 3, 1.5
        return linear_velocity, angle_velocity

    def time_estimate(self, target_position) -> int:
        # TODO 给定目标点，估计机器人到达目标点所需时间，单位为帧
        target_x = target_position[0]
        target_y = target_position[1]
        pass
        estimate_time = 0.0
        # 这里为了方便debug，利用欧式距离除以速度(5m/s)乘以每秒帧率
        estimate_time = math.sqrt((target_x - self.x) ** 2 + (target_y - self.y) ** 2) / 5 * 50
        return int(estimate_time)
