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


    def motion_control(self, target_position, path_x, path_y) -> Tuple[float, float]:
        # TODO 给定目标点，计算机器人的线速度与角速度

        target_x = target_position[0]
        target_y = target_position[1]

        ro_x = self.x
        ro_y = self.y

        if math.dist((ro_x,ro_y),(target_x,target_y)) >= 0.45:
            
            target_index = len(path_x)-2

            ro_x = self.x
            ro_y = self.y
            ro_o = self.angle
            vc = 0
            wc = 0
            if target_index>=1 and math.dist((ro_x,ro_y),(path_x[target_index-1],path_y[target_index-1])) <= 8:
                target_index -= 1

            if len(path_x)>=2 and math.dist((ro_x,ro_y),(path_x[target_index],path_y[target_index])) >= 2:
                ro_x = self.x
                ro_y = self.y
                ro_o = self.angle
                dist = math.dist((ro_x,ro_y),(path_x[target_index],path_y[target_index]))
                yaw = math.atan2(path_y[target_index] - ro_y, path_x[target_index] - ro_x)
                 
                af = yaw - ro_o
                if af > math.pi:
                    af -= 2 * math.pi
                elif af < -math.pi:
                    af += 2 * math.pi

                
                vc = 1000/dist + 5/(math.fabs(af)+0.5)
                wc = 3*af
                if dist <= 2.5 :
                    vc = 2/(math.fabs(af)+1) 
                    wc = 4*af
                        
            if math.dist((ro_x,ro_y),(path_x[target_index],path_y[target_index])) < 1.5:
                    target_index -= 1
                
            linear_velocity = vc
            angle_velocity = wc

        if math.dist((ro_x,ro_y),(target_x,target_y)) < 0.45:
            linear_velocity = 0
            angle_velocity = math.pi
            
        return linear_velocity, angle_velocity

    def time_estimate(self, target_position) -> float:
        # TODO 给定目标点，估计机器人到达目标点所需时间，单位为帧
        target_x = target_position[0]
        target_y = target_position[1]
        dist = math.dist((self.x,self.y),(target_x,target_y))
        estimate_time = dist/10+10*math.log(100/(dist+100))
        
        return int(estimate_time*50)
    
class Node(object):
    def __init__(self, x, y, cost = 0.0, parent = None):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent

# 为了避障，规划路线
class RRT(object):
    def __init__(self, N_SAMPLE=50, N_ITERATION=500, STEP=5):
        self.N_SAMPLE = N_SAMPLE
        self.N_ITERATION = N_ITERATION
        self.step = STEP
        self.minx = 0
        self.maxx = 50
        self.miny = 0
        self.maxy = 50
        self.robot_size = 0.5
        self.avoid_dist = 0.8
        self.nodes = []
        
    def getv(self, robots: List[Robot], target_positions: List[Tuple[float, float]]):
        line_speed = []
        angle_speed = []
        i = 0
        for robot in robots:
            target_x = target_positions[i][0]
            target_y = target_positions[i][1]
            path_x, path_y = self.plan(robot.id, robots, robot.x, robot.y, target_x, target_y)
            lv, av = robot.motion_control(target_positions[i], path_x, path_y)
            line_speed.append(lv)
            angle_speed.append(av)
            i = i + 1

        return line_speed, angle_speed

    def plan(self, id, robots: List[Robot], start_x, start_y, goal_x, goal_y):
        # Obstacles
        self.obstacle_x = [-999999]
        self.obstacle_y = [-999999]

        for robot in robots:
            if robot.id != id:
                self.obstacle_x.append(robot.x)
                self.obstacle_y.append(robot.y)
        
        # Obstacle KD Tree
        self.obstree = KDTree(np.vstack((self.obstacle_x, self.obstacle_y)).T)
        self.start = Node(start_x, start_y)
        self.goal = Node(goal_x, goal_y)
        self.obstacle = np.vstack((self.obstacle_x, self.obstacle_y)).T
        self.nodes = [self.start]

        for i in range(self.N_ITERATION):
            samp_nodes = self.sampling()
            samp_x = samp_nodes[0]
            samp_y = samp_nodes[1]
            distances = [math.sqrt((node.x-samp_x)**2+(node.y-samp_y)**2) for node in self.nodes]
            nearest_index = distances.index(min(distances))
            nearest = self.nodes[nearest_index]

            yaw = math.atan2(samp_y-nearest.y, samp_x-nearest.x)
            nextnode = self.get_nextnode(yaw, nearest_index)
           
            if self.check_obs(nextnode.x, nextnode.y, nearest.x, nearest.y):
                continue

            self.nodes.append(nextnode)

            if math.hypot(nextnode.x-self.goal.x, nextnode.y-self.goal.y) < self.step:
                if self.check_obs(nextnode.x, nextnode.y, self.goal.x, self.goal.y):
                    continue
                else:
                    break

        path_x = []
        path_y = []
        index = len(self.nodes) - 1
        while self.nodes[index].parent is not None:
            path_x.append(self.nodes[index].x)
            path_y.append(self.nodes[index].y)
            index = self.nodes[index].parent
        path_x.append(self.start.x)
        path_y.append(self.start.y)

        return path_x, path_y

    def get_nextnode(self, yaw, index):
        nearestNode = self.nodes[index]
        nextnode = copy.deepcopy(nearestNode)
        nextnode.parent = index
        nextnode.x += self.step * math.cos(yaw)
        nextnode.y += self.step * math.sin(yaw)
        return nextnode

    def sampling(self):
        if random.randint(0, 100) > self.N_SAMPLE:
            return [random.uniform(self.minx, self.maxx),
                    random.uniform(self.miny, self.maxy)]
        return [self.goal.x, self.goal.y]

    def check_obs(self, ix, iy, nx, ny):
        x = ix
        y = iy
        dx = nx - ix
        dy = ny - iy
        angle = math.atan2(dy, dx)
        dis = math.hypot(dx, dy)

        step_size = self.robot_size + self.avoid_dist
        steps = round(dis/step_size)
        for i in range(steps):
            distance, index = self.obstree.query(np.array([x, y]))
            if distance <= self.robot_size + self.avoid_dist:
                return True
            x += step_size * math.cos(angle)
            y += step_size * math.sin(angle)

        # check for goal point
        distance, index = self.obstree.query(np.array([nx, ny]))
        if distance <= self.robot_size + self.avoid_dist:
            return True

        return False
