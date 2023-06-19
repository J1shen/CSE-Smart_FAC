#!/bin/bash
import sys
import time
from typing import List, Tuple, Union
from Order import Order, Market
from Robot import Robot,  collision_if
from CraftTable import CraftTable
import func_timeout
import math

# 读取过程状态机状态
READ_MAP = 0
READ_FRAME_AND_MONEY = 1
READ_CRAFT_TABLE_NUM = 2
READ_EACH_CRAFT_TABLE = 3
READ_EACH_ROBOT = 4
READ_FINISH = 5


def read_util_ok(env=None, read_map=False, debug_input=None):
    # 从判题器的输出中读取数据，当read_map为True时，读取地图
    if debug_input is None:
        debug_input = sys.stdin
    in_data = debug_input.readline().strip()
    current_status = READ_FRAME_AND_MONEY
    read_work_map = []
    line = 100
    if read_map:
        current_status = READ_MAP
    next_status = current_status
    first_read_env = False
    if env is None:
        env = dict()
        first_read_env = True
    craft_table_to_read = -1
    robots_to_read = 4
    while in_data != "OK":
        sys.stderr.write(f"[STDIN]: {in_data}\n")
        if current_status == READ_MAP:
            # 读取地图
            read_work_map.append(list(in_data))
            next_status = READ_MAP
            if line == 0:
                return read_work_map
            line -= 1
        elif current_status == READ_FRAME_AND_MONEY:
            # 读取当前帧数和金钱数
            env['frame_id'] = int(in_data.split(' ')[0])
            env['money'] = int(in_data.split(' ')[1])
            next_status = READ_CRAFT_TABLE_NUM
        elif current_status == READ_CRAFT_TABLE_NUM:
            # 读取工作台数量
            craft_table_to_read = int(in_data)
            assert 1 <= craft_table_to_read <= 50
            if first_read_env:
                env['craft_tables']: List[CraftTable] = []
            next_status = READ_EACH_CRAFT_TABLE
        elif current_status == READ_EACH_CRAFT_TABLE:
            # 读取每一个工作台
            datas_in = in_data.split(' ')
            if first_read_env:
                craft = CraftTable(table_id=len(env['craft_tables']),
                                   table_type=int(datas_in[0]),
                                   x=float(datas_in[1]),
                                   y=float(datas_in[2]),
                                   rest_time=int(datas_in[3]),
                                   raw_status=int(datas_in[4]),
                                   product_status=int(datas_in[5]))
                env['craft_tables'].append(craft)
            else:
                craft = env['craft_tables'][len(env['craft_tables']) - craft_table_to_read]
                craft.rest_time = int(datas_in[3])
                craft.raw_status_from_int(int(datas_in[4]))
                craft.product_with_rest_time(status=int(datas_in[5]))
                env['craft_tables'][len(env['craft_tables']) - craft_table_to_read] = craft

            craft_table_to_read -= 1
            if craft_table_to_read == 0:
                if first_read_env:
                    env['robots']: List[Robot] = []
                next_status = READ_EACH_ROBOT
        elif current_status == READ_EACH_ROBOT:
            robot_data_in = in_data.split(' ')
            if first_read_env:
                robot = Robot(robot_id=len(env['robots']),
                              at_table=int(robot_data_in[0]),
                              possession=int(robot_data_in[1]),
                              time_factor=float(robot_data_in[2]),
                              collision_factor=float(robot_data_in[3]),
                              angle_velocity=float(robot_data_in[4]),
                              linear_velocity_x=float(robot_data_in[5]),
                              linear_velocity_y=float(robot_data_in[6]),
                              angle=float(robot_data_in[7]),
                              x=float(robot_data_in[8]),
                              y=float(robot_data_in[9]))
                env['robots'].append(robot)
            else:
                robot = env['robots'][len(env['robots']) - robots_to_read]
                robot.at_table = int(robot_data_in[0])
                robot.possession = int(robot_data_in[1])
                robot.time_factor = float(robot_data_in[2])
                robot.collision_factor = float(robot_data_in[3])
                robot.angle_velocity = float(robot_data_in[4])
                robot.linear_velocity_x = float(robot_data_in[5])
                robot.linear_velocity_y = float(robot_data_in[6])
                robot.angle = float(robot_data_in[7])
                robot.x = float(robot_data_in[8])
                robot.y = float(robot_data_in[9])
                env['robots'][len(env['robots']) - robots_to_read] = robot

            robots_to_read -= 1
            if robots_to_read == 0:
                next_status = READ_FINISH
        elif current_status == READ_FINISH:
            break
        current_status = next_status
        in_data = debug_input.readline().strip()
    return env


def finish():
    # 通知判题器程序已经结束
    sys.stdout.write('OK\n')
    sys.stderr.write("[STDOUT]: OK\n")
    sys.stdout.flush()


def update_orders(craft_tables: List[CraftTable], target_market: Market):
    # 工作台更新市场订单：当市场内没有订单时，创建订单；当市场内有订单时，更新订单价格及状态
    for craft_table in craft_tables:
        if len(craft_table.orders) == 0:
            # 若还没有订单则新建订单
            craft_table.create_orders(target_market)
        else:
            # 仅更新订单价格及状态
            craft_table.update_orders(target_market)


def take_orders(robots: List[Robot], target_market: Market, craft_tables: List[CraftTable]):
    for robot in robots:
        if len(robot.order) == 0:
            # 当机器人当前没有订单或该订单已经完成时，从市场接取订单
            available_orders = filter_possible_orders(robot, target_market)
            if len(available_orders) == 0:
                return
            profits = list()
            orders: List[Union[Order, None]] = list()
            for order in available_orders:
                # 估算利润，其中成本的估计较为简单，仅考虑到了时间成本；而收益的估计较为复杂
                # 对于买入单，机器人还需要同时接下对应的卖出单以防买入后无法卖出
                gain, pair_order = robot.estimate_gain(order, target_market, craft_tables)
                cost = robot.estimate_cost(order, target_market, craft_tables)
                profits.append(gain - cost)
                orders.append(pair_order)
            # 选择利润最大订单达成交易
            max_profit_index = profits.index(max(profits))
            max_profit_order = available_orders[max_profit_index]
            max_profit_order.executor = robot.id
            max_profit_order.status = Order.ORDER_RECEIVED
            sell_order = orders[max_profit_index]
            sell_order.executor = robot.id
            sell_order.status = Order.ORDER_RECEIVED
            robot.order = [max_profit_order, sell_order]


def filter_possible_orders(robot: Robot, target_market: Market) -> List[Order]:
    # 从市场中筛选出机器人可以接取的订单
    available_orders = []
    robot_possession = robot.possession
    order_type = 'b'
    if robot_possession == 0:
        # 未携带物品 -- 只能购买， 机器人购买对应卖出订单
        order_type = 's'
    if order_type == 's':
        # 购买可以买所有物品
        for order in target_market.orders:
            if order.content[0] == order_type and order.status == Order.ORDER_HANGUP:
                available_orders.append(order)
    else:
        # 售出仅能售出相同类型的商品
        for order in target_market.orders:
            if order.content[0] == order_type \
                    and order.content[1] == robot_possession \
                    and order.status == Order.ORDER_HANGUP:
                available_orders.append(order)
    return available_orders


def multi_robot_control(robots: List[Robot], target_positions: List[Tuple[float, float]]):
    # TODO 可以在这里写多机器人联合控制的代码，在这里写的控制代码可以获取到所有机器人的位置，故能够较好的完成避障代码的实现
    # 注意1， robot.motion_control()中的代码与这里不同，那里的代码仅能获取到单个机器人的信息，无法进行全局的控制
    # 注意2，机器人在部分情况下会出现目标位置与自身位置重合的情况，代码需注意不会出现除零bug等
    planner = collision_if()
    line_speed, angle_speed = planner.collision_detect(robots, target_positions)

    return line_speed, angle_speed



def action_generate(robots: List[Robot], crafts_table: List[CraftTable]) -> Tuple[List[float], List[float], List[str]]:
    # 决策生成、包括运动决策和买/卖决策
    command: List[str] = []
    # 买、卖决策生成
    target_positions: List[Tuple[float, float]] = []
    for robot in robots:
        if len(robot.order) != 0:
            # 有订单时，机器人移动到订单所在位置
            if robot.order[0].owner == robot.at_table:
                # 机器人已经到达工作台，完成订单
                robot.order[0].status = Order.ORDER_FIN
                if robot.order[0].content[0] == 'b':
                    # 注意b指该订单为控制台买入，即机器人卖出
                    command.append('sell %d\n' % robot.id)
                else:
                    # 否则为机器人买入
                    command.append('buy %d\n' % robot.id)
                robot.order.pop(0)
            if len(robot.order) != 0:
                order = robot.order[0]
                target_position = (crafts_table[order.owner].x, crafts_table[order.owner].y)
            else:
                target_position = (robot.x, robot.y)
            target_positions.append(target_position)
        else:
            # 机器人无订单时，机器人将当前位置设为目标位置
            target_position = (robot.x, robot.y)
            target_positions.append(target_position)

    # 多机器人联合运动控制 -- 该函数最多仅运行5ms
    try:
        line_speeds, angle_speeds = \
            func_timeout.func_timeout(0.005, multi_robot_control, (robots, target_positions))
    except func_timeout.FunctionTimedOut:
        line_speeds, angle_speeds = [], []

    return line_speeds, angle_speeds, command


if __name__ == '__main__':
    # file_input = open("debug/debug_input", "r")
    # work_map = read_util_ok(read_map=True, debug_input=file_input)
    work_map = read_util_ok(read_map=True)
    finish()
    market = Market()
    env = None
    while True:
        # env = read_util_ok(env, debug_input=file_input)
        env = read_util_ok(env)
        start_time = time.time()
        # 工作台更新订单
        update_orders(env['craft_tables'], market)
        # 机器人接单
        take_orders(env['robots'], market, env['craft_tables'])
        # 机器人根据订单进行运动规划
        line_speed, angle_speed, cmds = action_generate(env['robots'], env['craft_tables'])
        sys.stdout.write('%d\n' % env['frame_id'])
        if len(line_speed) == 4 and len(angle_speed):
            for robot_id in range(4):
                sys.stdout.write('forward %d %f\n' % (robot_id, line_speed[robot_id]))
                sys.stderr.write('[STDOUT]: forward %d %f\n' % (robot_id, line_speed[robot_id]))
                sys.stdout.write('rotate %d %f\n' % (robot_id, angle_speed[robot_id]))
                sys.stderr.write('[STDOUT]: rotate %d %f\n' % (robot_id, angle_speed[robot_id]))
        for cmd in cmds:
            sys.stdout.write(cmd)
            sys.stderr.write('[STDOUT]: %s' % cmd)
        finish()
        finish_time = time.time()
        delta_time = (finish_time - start_time) * 1000
        sys.stderr.write('[DEBUG]: TIME %d MS\n' % int(delta_time))
