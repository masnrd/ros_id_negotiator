from __future__ import annotations
import struct
import rclpy
from time import time_ns
from pathlib import Path
from socket import socket, SOCK_DGRAM, AF_INET
from rclpy.node import Node
from typing import Dict
from negotiator_interfaces.srv import Connection

CYCLE_INTERVAL = 1
CYCLE_TIMEOUT  = 30  # Number of cycles until we switch the ros domain ID.
TIME_FRAME_NS = pow(10, 9) * 1  # ROS_DOMAIN_ID would be different each second (so if restarted, new ROS_DOMAIN_ID)
MIN_ROS_DOMAIN_ID = 1
MAX_ROS_DOMAIN_ID = 232
DATA_FILE = Path.home().joinpath(".droneconfig.sh")

domain_id = -1

SERVER_HOST = ("127.0.0.1", 6969) #("10.0.0.2", 6969)

def get_domain_id() -> int:
    val = time_ns() // TIME_FRAME_NS
    return (val % (MAX_ROS_DOMAIN_ID - MIN_ROS_DOMAIN_ID + 1)) + MIN_ROS_DOMAIN_ID

def get_syn_packet(drone_id: int, ros_domain_id: int) -> bytes:
    return b"NEGO" + struct.pack("!ii", drone_id, ros_domain_id)

def unpack_syn_packet(pkt: bytes) -> Dict[str, int]:
    if pkt[0:4] != b"NEGO":
        raise RuntimeError("Received invalid packet.")
    
    drone_id, ros_domain_id = struct.unpack("!ii", pkt[4:])
    return {
        "drone_id": drone_id,
        "ros_domain_id": ros_domain_id
    }

class Client(Node):
    def __init__(self, client_sock: socket):
        super().__init__("client")
        global domain_id
        self.declare_parameter("droneId", -1)
        drone_id = self.get_parameter("droneId").get_parameter_value().integer_value
        if drone_id == -1:
            print("Please run with droneId parameter.")
            exit(1)

        self.drone_id = drone_id
        self.sock = client_sock
        self.is_connected = False
        self.cycles = 0
        self.srv_conn = self.create_service(
            Connection,
            "/negotiator/srv/conn",
            self.srv_handle_connection
        )
        
        self.log(f"Connecting with ROS_DOMAIN_ID: {domain_id}")
        self.check_timer = self.create_timer(CYCLE_INTERVAL, self.cycle)
    
    def cycle(self):
        global domain_id
        if self.is_connected:
            self.log("Connected to server, exiting.")
            self.exit()
        
        if self.cycles >= CYCLE_TIMEOUT:
            self.cycles = 0
            self.log(f"Failed to connect.")
            domain_id = -1
            self.exit()

        pkt = get_syn_packet(self.drone_id, domain_id)
        self.sock.sendto(pkt, SERVER_HOST)
        self.cycles += 1

    def exit(self):
        self.destroy_timer(self.check_timer)
        raise SystemExit

    def srv_handle_connection(self, req: Connection.Request, res: Connection.Response):
        has_error = False
        if req.server_id != 0:
            has_error = True
            self.warn(f"Received connection request from invalid ID {req.server_id}.")
        if self.is_connected:
            has_error = True
            self.warn(f"Already connected.")
        
        res.timestamp_s = time_ns() // (10 ** 9)
        res.client_id = self.drone_id
        res.error = has_error
        self.is_connected = True
        return res

    def log(self, msg: str):
        self.get_logger().info(f"Client: {msg}")

    def warn(self, msg: str):
        self.get_logger().warn(f"Client: {msg}")

def main(args=None):
    global domain_id

    # 1. Set up socket
    sock = socket(AF_INET, SOCK_DGRAM)

    # 2. Spawn node
    while True:
        domain_id = get_domain_id()
        print(f"Client: Attempting connection with domain ID: {domain_id}")
        rclpy.init(args=args, domain_id=domain_id)
        node = Client(sock)

        try:
            rclpy.spin(node)
        except SystemExit:
            pass
        
        node.destroy_node()
        rclpy.shutdown()

        if domain_id == -1:
            print(f"Client: Failed to connect with domain ID {domain_id}.")
        else:
            print(f"Client: Successful connection with domain ID {domain_id}.")
            DATA_FILE.touch(exist_ok=True)
            with DATA_FILE.open("w") as fp:
                fp.write(f"export ROS_DOMAIN_ID={domain_id}")
            exit(0)


if __name__ == '__main__':
    main()