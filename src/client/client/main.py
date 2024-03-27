from __future__ import annotations
import struct
import rclpy
from time import time_ns
from socket import socket, SOCK_DGRAM, AF_INET
from rclpy.node import Node
from typing import Dict, Tuple
from os import environ
from negotiator_interfaces.srv import Connection

CYCLE_INTERVAL = 1
CYCLE_TIMEOUT  = 30  # Number of cycles until we switch the ros domain ID.

def get_syn_packet(ros_domain_id: int) -> bytes:
    return b"NEGO" + struct.pack("!i", ros_domain_id)

def unpack_syn_packet(pkt: bytes) -> Dict[str, int]:
    if pkt[0:4] != b"NEGO":
        raise RuntimeError("Received invalid packet.")
    
    ros_domain_id = struct.unpack("!i", pkt[4:])[0]
    return {
        "ros_domain_id": ros_domain_id
    }

class Client(Node):
    def __init__(self, client_sock: socket, ros_domain_id: int, server_addr: Tuple[str, int]):
        super().__init__("client")

        self.sock = client_sock
        self.server_addr = server_addr
        self.is_connected = False
        self.cycles = 0
        self.srv_conn = self.create_service(
            Connection,
            "/negotiator/srv/conn",
            self.srv_handle_connection
        )
        self.ros_domain_id = ros_domain_id
        
        self.check_timer = self.create_timer(CYCLE_INTERVAL, self.cycle)
    
    def cycle(self):
        if self.is_connected:
            self.exit(success=True)
        
        if self.cycles >= CYCLE_TIMEOUT:
            self.exit(success=False)

        pkt = get_syn_packet(self.ros_domain_id)
        self.sock.sendto(pkt, self.server_addr)
        self.cycles += 1

    def exit(self, success=False):
        self.destroy_timer(self.check_timer)
        if success:
            print("0")
        else:
            print("1")
        raise SystemExit

    def srv_handle_connection(self, req: Connection.Request, res: Connection.Response):
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
    sock = socket(AF_INET, SOCK_DGRAM)
    domain_id = -1
    server_ip = None
    server_port = -1
    try:
        domain_id = int(environ.get("ROS_DOMAIN_ID", None))
        server_ip = str(environ.get("SERVER_IP"))
        server_port = int(environ.get("SERVER_PORT", None))
        if len(server_ip) == 0:
            raise Exception()
    except Exception:
        raise RuntimeError("ROS_DOMAIN_ID or SERVER_IP not known, please run this with `client_run.py`.")

    server_addr = (server_ip, server_port)

    rclpy.init(args=args)
    node = Client(sock, int(domain_id), server_addr)

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()