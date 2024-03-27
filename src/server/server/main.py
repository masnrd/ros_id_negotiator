from __future__ import annotations
import rclpy
from time import time_ns
from rclpy.node import Node
from rclpy.task import Future
from negotiator_interfaces.srv import Connection

CYCLE_INTERVAL = 1
CYCLE_TIMEOUT = 300

class Server(Node):
    def __init__(self):
        super().__init__("server")
        self.cli_conn = self.create_client(
            Connection,
            "/negotiator/srv/conn"
        )
        self.check_timer = self.create_timer(CYCLE_INTERVAL, self.cycle)
        self.attempted_fut = self.attempt_connection()
        self.cycles = 0

    def cycle(self):
        if self.attempted_fut is None:
            return
        self.cycles += 1
        if not self.attempted_fut.done():
            if self.cycles >= CYCLE_TIMEOUT:
                self.log(f"Failed to connect to client after {CYCLE_TIMEOUT} cycles.")
                self.exit(success=False)
            elif (self.cycles * CYCLE_INTERVAL) % 2 == 0:
                self.attempted_fut.cancel()
                self.attempted_fut = self.attempt_connection()
            return
        response: Connection.Response = self.attempted_fut.result()
        if response.error:
            self.log(f"Failed to connect to client, client-side error.")
            self.exit(success=False)

        self.exit(success=True)
        
    def exit(self, success=False):
        self.destroy_timer(self.check_timer)
        if success:
            print("0")
        else:
            print("1")
        raise SystemExit()
    
    def attempt_connection(self) -> Future:
        msg = Connection.Request()
        msg.timestamp_s = time_ns() // pow(10, 9)
        self.log(f"Attempting connection...")
        return self.cli_conn.call_async(msg)
        
    def log(self, msg: str):
        # self.get_logger().info(f"Server: {msg}")
        pass

    def warn(self, msg: str):
        # self.get_logger().warn(f"Server: {msg}")
        pass

def main(args=None):
    rclpy.init(args=args)
    node = Server()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass

    node.destroy_node()

if __name__ == '__main__':
    main()