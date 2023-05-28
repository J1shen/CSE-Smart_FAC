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
        self.raw_status = [False] * 7
        for i in range(1, 8):
            self.raw_status[i - 1] = raw_status & (1 << i) > 0
        self.product_status = product_status > 0
        self.orders: List[Order] = []
        # Timers: Update Order

    def raw_status_from_int(self, raw_status: int):
        for i in range(1, 8):
            self.raw_status[i - 1] = raw_status & (1 << i) > 0

    def product_with_rest_time(self, status: int):
        self.product_status = (status > 0) and self.rest_time == 0

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
                if info[0] == 's' and info[1] in [1, 2, 3]:
                    # 1, 2, 3号物品无需原材料即可以生产
                    self.orders.append(market.create_order(owner=craft_table_id,
                                                           order_type='s',
                                                           obj_id=info[1]))

    def update_orders(self, target_market: Market):
        # 对原有的订单进行更新
        sell_order_hangup = False
        for order in self.orders:
            if order.content[0] == 's':
                sell_order_hangup = True
            if order.status == Order.ORDER_RECEIVED:
                # 等待订单完成即可
                continue
            elif order.status == Order.ORDER_HANGUP:
                # 对于买入订单，依据订单市场调整价格，这里先不作调整
                continue
            elif order.status == Order.ORDER_FIN:
                # 订单已经完成
                if order.content[0] == 'b':
                    # 买入订单，等到原材料空缺时更新订单
                    if not self.raw_status[order.content[1] - 1]:
                        # 原材料已经空缺，更新订单
                        target_market.refresh(order.id)
                else:
                    # 卖出订单，等到产品生成完成时更新订单
                    if self.product_status:
                        target_market.refresh(order.id)
        # 当产品第一次被生产出来时，放出订单
        if not sell_order_hangup and self.product_status:
            self.orders.append(target_market.create_order(owner=self.id,
                                                          order_type='s',
                                                          obj_id=self.type))
