from typing import Union, Tuple
from Order import Order


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

    def motion_control(self, target_position) -> Tuple[float, float]:
        # TODO 给定目标点，计算机器人的线速度与角速度
        target_x = target_position[0]
        target_y = target_position[1]
        pass
        linear_velocity, angle_velocity = 0.0, 0.0
        return linear_velocity, angle_velocity

    def time_estimate(self, target_position) -> float:
        # TODO 给定目标点，估计机器人到达目标点所需时间
        target_x = target_position[0]
        target_y = target_position[1]
        pass
        estimate_time = 0.0
        return estimate_time
