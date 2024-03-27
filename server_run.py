from __future__ import annotations
from os import environ
import struct
import subprocess
from time import time_ns
from socket import socket, SOCK_DGRAM, AF_INET
from typing import Dict

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

def main(args=None):
    if args is None:
        args = []
    global domain_id
    domain_id = -1

    # 1. Set up socket
    sock = socket(AF_INET, SOCK_DGRAM)

    # 2. Listen until we get an attempted connection
    sock.bind(SERVER_HOST)

    print("Server listening for connections...")

    # 2. Spawn node
    while True:
        client_msg, client_addr = sock.recvfrom(SYN_PACKET_SIZE)
        data = unpack_syn_packet(client_msg)
        domain_id = data["ros_domain_id"]
        print(f"Received connection from {client_addr} with domain ID {domain_id}")

        proc_env = environ.copy()
        proc_env["ROS_DOMAIN_ID"] = str(domain_id)
        proc = subprocess.Popen(
            ["ros2", "run", "server", "main"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            env=proc_env
        )
        out, err = proc.communicate()
        """         out = subprocess.check_output(
            ["ros2", "run", "server", "main"],
            shell=True,
            stderr=subprocess.DEVNULL,
            env=proc_env
        ) """
        if out.strip() != b"0":
            print("Failed to connect, reattempting.")
        else:
            print(f"Domain ID: {domain_id}")
            exit(0)

if __name__ == '__main__':
    main()