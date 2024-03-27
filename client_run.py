from __future__ import annotations
from os import environ
import struct
import subprocess
from pathlib import Path
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
DATA_FILE = Path.home().joinpath(".droneconfig.sh")

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

    while True:
        # Generate domain ID
        domain_id = get_domain_id()
        print(f"Attempting connection with ROS_DOMAIN_ID: {domain_id}")

        # Start node
        proc_env = environ.copy()
        proc_env["ROS_DOMAIN_ID"] = str(domain_id)
        proc = subprocess.Popen(
            ["ros2", "run", "client", "main"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            env=proc_env
        )
        out, err = proc.communicate()

        if out.strip() != b"0":
            print("Failed to connect, reattempting with new ID...")
        else:
            print(f"Established connection successfully.")
            DATA_FILE.touch(exist_ok=True)
            DATA_FILE.write_text(f"export ROS_DOMAIN_ID={domain_id}\n")
            exit(0)

if __name__ == '__main__':
    main()