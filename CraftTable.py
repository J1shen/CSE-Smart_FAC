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
            market.create_order(owner=craft_table_id,
                                order_type=info[0],
                                obj_id=info[1])

    def update_orders(self, target_market):
        pass
