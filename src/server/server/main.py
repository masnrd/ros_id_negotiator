from __future__ import annotations
import struct
import rclpy
from time import time_ns
from socket import socket, SOCK_DGRAM, AF_INET
from rclpy.node import Node
from rclpy.task import Future
from typing import Dict
from negotiator_interfaces.srv import Connection

CYCLE_INTERVAL = 1
CYCLE_TIMEOUT = 300
TIME_FRAME_NS = pow(10, 9) * 1  # ROS_DOMAIN_ID would be different each second (so if restarted, new ROS_DOMAIN_ID)
MIN_ROS_DOMAIN_ID = 1
MAX_ROS_DOMAIN_ID = 232

SERVER_HOST = ("127.0.0.1", 6969) #("10.0.0.2", 6969)
SYN_PACKET_SIZE = 8

def get_domain_id() -> int:
    val = time_ns() // TIME_FRAME_NS
    return (val % (MAX_ROS_DOMAIN_ID - MIN_ROS_DOMAIN_ID + 1)) + MIN_ROS_DOMAIN_ID

def get_syn_packet(ros_domain_id: int) -> bytes:
    return b"NEGO" + struct.pack("!i", ros_domain_id)

def unpack_syn_packet(pkt: bytes) -> Dict[str, int]:
    if pkt[0:4] != b"NEGO":
        raise RuntimeError("Received invalid packet.")
    
    ros_domain_id = struct.unpack("!i", pkt[4:])[0]
    return {
        "ros_domain_id": ros_domain_id
    }

class Server(Node):
    def __init__(self, ros_domain_id: int):
        super().__init__("server")
        
        self.domain_id = ros_domain_id
        self.cli_conn = self.create_client(
            Connection,
            "/negotiator/srv/conn"
        )
        self.check_timer = self.create_timer(CYCLE_INTERVAL, self.cycle)
        self.attempted_fut = self.attempt_connection()
        self.cycles = 0

    def cycle(self):
        global domain_id
        if self.attempted_fut is None:
            return
        self.cycles += 1
        if not self.attempted_fut.done():
            if self.cycles >= CYCLE_TIMEOUT:
                self.log(f"Failed to connect to client after {CYCLE_TIMEOUT} cycles.")
                self.exit()
            return
        response: Connection.Response = self.attempted_fut.result()
        if response.error:
            self.log(f"Failed to connect to client, client-side error.")
            domain_id = -1
            self.exit()

        self.exit()
        
    def exit(self):
        self.destroy_timer(self.check_timer)
        raise SystemExit
    
    def attempt_connection(self) -> Future:
        msg = Connection.Request()
        msg.timestamp_s = time_ns() // pow(10, 9)
        self.log(f"Attempting connection...")
        return self.cli_conn.call_async(msg)
        
    def srv_handle_connection(self, req: Connection.Request, res: Connection.Response):
        has_error = False
        if self.is_connected:
            has_error = True
            self.warn(f"Already connected.")
        
        res.timestamp_s = time_ns() // (10 ** 9)
        res.error = has_error
        return res
        
    def log(self, msg: str):
        self.get_logger().info(f"Server: {msg}")

    def warn(self, msg: str):
        self.get_logger().warn(f"Server: {msg}")

def main(args=None):
    if args is None:
        args = []
    global domain_id
    domain_id = -1

    # 1. Set up socket
    sock = socket(AF_INET, SOCK_DGRAM)

    # 2. Listen until we get an attempted connection
    sock.bind(SERVER_HOST)

    # 2. Spawn node
    while True:
        print("Server: Listening for connections.")
        client_msg, client_addr = sock.recvfrom(SYN_PACKET_SIZE)
        print(f"Server: Received connection from {client_addr}.")
        data = unpack_syn_packet(client_msg)
        domain_id = data["ros_domain_id"]
        rclpy.init(args=[f"ROS_DOMAIN_ID={domain_id}"])
        node = Server(domain_id)
        try:
            rclpy.spin(node)
        except SystemExit:
            pass

        node.destroy_node()

        if domain_id == -1:
            print(f"Server: Failed to send ROS data to {client_addr}. Ending connection.")
        else:
            print(f"Received working ROS_DOMAIN_ID from drone: {domain_id}")
            exit(0)

if __name__ == '__main__':
    main()