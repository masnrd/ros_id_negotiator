from __future__ import annotations
import rclpy
from time import time_ns
from rclpy.node import Node
from typing import Dict, Tuple
from os import environ
from negotiator_interfaces.srv import Connection

CYCLE_INTERVAL = 0.5
CYCLE_TIMEOUT  = (1 / CYCLE_INTERVAL) * 30  # Number of seconds until we switch the ros domain ID.

class Client(Node):
    def __init__(self):
        super().__init__("client")

        self.is_connected = False
        self.cycles = 0
        self.srv_conn = self.create_service(
            Connection,
            "/negotiator/srv/conn",
            self.srv_handle_connection
        )
        
        self.check_timer = self.create_timer(CYCLE_INTERVAL, self.cycle)
    
    def cycle(self):
        if self.is_connected:
            self.exit(success=True)
        
        if self.cycles >= CYCLE_TIMEOUT:
            self.exit(success=False)

        self.cycles += 1

    def exit(self, success=False):
        self.destroy_timer(self.check_timer)
        if success:
            print("0")
        else:
            print("1")
        raise SystemExit()

    def srv_handle_connection(self, req: Connection.Request, res: Connection.Response):
        self.log("Received connection.")
        has_error = False
        if self.is_connected:
            has_error = True
            self.warn(f"Already connected.")
        
        res.timestamp_s = time_ns() // (10 ** 9)
        res.error = has_error
        self.is_connected = True
        return res

    def log(self, msg: str):
        # self.get_logger().info(f"Client: {msg}")
        pass

    def warn(self, msg: str):
        # self.get_logger().warn(f"Client: {msg}")
        pass

def main(args=None):
    rclpy.init(args=args)
    node = Client()

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()