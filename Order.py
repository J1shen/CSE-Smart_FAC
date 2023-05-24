from Constants import Constants


class Order:
    # 订单状态
    ORDER_HANGUP = 0            # 订单挂在市场，还未成交
    ORDER_RECEIVED = 1          # 订单已成交，还未完成
    ORDER_FIN = 2               # 订单已完成

    def __init__(self, order_id, owner: int, content, prize: float):
        # 唯一标识号
        self.id = order_id
        # 订单所属工作台
        self.owner = owner
        # 订单执行人
        self.executor = -1
        # 订单内容: ('b', id) or ('s', id)
        self.content = content
        # 订单报价
        self.prize = prize
        # 订单状态
        self.status = Order.ORDER_HANGUP


class Market:
    def __init__(self):
        self.orders = []

    def create_order(self, owner, order_type, obj_id):
        prize = Constants.OBJECT_SELL_PRIZE[obj_id - 1] if order_type == 's' else Constants.OBJECT_BUY_PRIZE[obj_id - 1]
        new_order = Order(order_id=len(self.orders),
                          owner=owner,
                          content=(order_type, obj_id),
                          prize=prize)
        self.orders.append(new_order)
