#!/bin/bash
import sys
from Order import *
from Constants import Constants

READ_MAP = 0
READ_FRAME_AND_MONEY = 1
READ_CRAFT_TABLE_NUM = 2
READ_EACH_CRAFT_TABLE = 3
READ_EACH_ROBOT = 4
READ_FINISH = 5


def read_util_ok(read_map = False):
    in_data = input()
    current_status = READ_FRAME_AND_MONEY
    if read_map:
        work_map = []
        current_status = READ_MAP
        line = 100
    next_status = current_status
    env = dict()
    craft_table_to_read = -1
    robots_to_read = 4
    while in_data != "OK":
        sys.stderr.write(f"[STDIN]: {in_data}\n")
        if current_status == READ_MAP:
            # 读取地图
            work_map.append(list(in_data))
            next_status = READ_MAP
            if line == 0:
                return work_map
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
            env['craft_tables'] = []
            next_status = READ_EACH_CRAFT_TABLE
        elif current_status == READ_EACH_CRAFT_TABLE:
            # 读取每一个工作台
            craft = dict()
            datas_in = in_data.split(' ')
            craft['id'] = len(env['craft_tables'])
            craft['type'] = int(datas_in[0])
            craft['x'] = float(datas_in[1])
            craft['y'] = float(datas_in[2])
            craft['rest_time'] = int(datas_in[3])
            craft['raw_status'] = int(datas_in[4])
            craft['product_status'] = int(datas_in[5])
            craft['orders'] = list()
            env['craft_tables'].append(craft)
            craft_table_to_read -= 1
            if craft_table_to_read == 0:
                env['robots'] = []
                next_status = READ_EACH_ROBOT
        elif current_status == READ_EACH_ROBOT:
            robot = dict()
            robot_data_in = in_data.split(' ')
            robot['id'] = len(env['robots'])
            robot['at_table'] = int(robot_data_in[0])
            robot['possession'] = int(robot_data_in[1])
            robot['time_factor'] = float(robot_data_in[2])
            robot['collision_factor'] = float(robot_data_in[3])
            robot['angle_v'] = float(robot_data_in[4])
            robot['linear_v_x'] = float(robot_data_in[5])
            robot['linear_v_y'] = float(robot_data_in[6])
            robot['angle'] = float(robot_data_in[7])
            robot['x'] = float(robot_data_in[8])
            robot['y'] = float(robot_data_in[9])
            robot['order'] = None
            env['robots'].append(robot)
            robots_to_read -= 1
            if robots_to_read == 0:
                next_status = READ_FINISH
        elif current_status == READ_FINISH:
            break
        current_status = next_status
        in_data = input()
    return env


def finish():
    sys.stdout.write('OK\n')
    sys.stderr.write("[STDOUT]: OK\n")
    sys.stdout.flush()


def update_orders(craft_tables, market: Market):
    """
    craft_tables
    [{
        "type", "x", "y", "rest_time", "raw_status", "product_status",
        "orders"
    }]
    """
    for craft_table in craft_tables:
        if len(craft_table['orders']) == 0:
            # 若还没有订单则新建订单
            create_orders_for(craft_table, market)
        else:
            # 仅更新订单价格及状态
            update_order_prize_status(craft_table, market)


def create_orders_for(craft_table, market: Market):
    craft_table_id = craft_table['id']
    craft_table_type = craft_table['type']
    table_info = Constants.TABLE_ALL[craft_table_type - 1]
    for info in table_info:
        market.create_order(craft_table_id, info[0], info[1])


def update_order_prize_status(craft_table, market):
    pass


def take_orders(robots, market):
    robot_id = 0
    for robot in robots:
        if robot['order'] is None:
            # 当机器人当前没有订单时，从市场接取订单
            available_orders = filter_possible_orders(robot, market)
            profits = list()
            for order in available_orders:
                gain = calculate_gain(robot, order)
                cost = calculate_cost(robot, order)
                profits.append(gain - cost)
            # 选择利润最大订单达成交易
            max_profit_index = profits.index(max(profits))
            max_profit_order = available_orders[max_profit_index]
            max_profit_order.executor = robot_id
            max_profit_order.status = Order.ORDER_RECEIVED
            robot['order'] = max_profit_order
        robot_id += 1


def motion_control(robots, crafts_table):
    # 运动控制
    line_speeds = []
    angle_speeds = []
    pass
    return line_speeds, angle_speeds


if __name__ == '__main__':
    work_map = read_util_ok(read_map=True)
    finish()
    while True:
        env = read_util_ok()
        market = Market()
        # 工作台更新订单
        update_orders(env['craft_tables'], market)
        # 机器人接单
        take_orders(env['robots'], market)
        # 机器人根据订单进行运动规划
        line_speed, angle_speed = motion_control(env['robots'], env['craft_tables'])

        for robot_id in range(4):
            sys.stdout.write('forward %d %d\n' % (robot_id, line_speed[robot_id]))
            sys.stderr.write('[STDOUT]: forward %d %d\n' % (robot_id, line_speed[robot_id]))
            sys.stdout.write('rotate %d %f\n' % (robot_id, angle_speed[robot_id]))
            sys.stderr.write('[STDOUT]: rotate %d %f\n' % (robot_id, angle_speed[robot_id]))
        finish()
