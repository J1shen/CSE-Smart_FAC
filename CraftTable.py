from Order import Order, Market
from typing import List
from Constants import Constants


class CraftTable:
    def __init__(self, table_id: int, table_type: int,
                 x: float, y: float, rest_time: int,
                 raw_status: int, product_status: int):
        self.id = table_id
        self.type = table_type
        self.x = x
        self.y = y
        self.rest_time = rest_time
        self.raw_status = raw_status
        self.product_status = product_status
        self.orders: List[Order] = []

    def create_orders(self, market: Market):
        craft_table_id = self.id
        craft_table_type = self.type
        table_info = Constants.TABLE_ALL[craft_table_type - 1]
        for info in table_info:
            # 建立买单和无条件卖单(指无需原材料既可以生产的物品)
            if info[0] == 'b':
                self.orders.append(market.create_order(owner=craft_table_id,
                                                       order_type='b',
                                                       obj_id=info[1]))
            else:
                if info[1] in [1, 2, 3]:
                    # 1, 2, 3号物品无需原材料即可以生产
                    self.orders.append(market.create_order(owner=craft_table_id,
                                                           order_type='s',
                                                           obj_id=info[1]))

    def update_orders(self, target_market):
        pass
