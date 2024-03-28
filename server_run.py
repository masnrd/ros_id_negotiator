from __future__ import annotations
from os import environ
import struct
import subprocess
from time import time_ns
from socket import socket, SOCK_DGRAM, AF_INET
from typing import Dict

TIME_FRAME_NS = pow(10, 9) * 30  # ROS_DOMAIN_ID would be different every 30 seconds (so if restarted, new ROS_DOMAIN_ID)
MIN_ROS_DOMAIN_ID = 1
MAX_ROS_DOMAIN_ID = 232
def get_domain_id() -> int:
    val = time_ns() // TIME_FRAME_NS
    return (val % (MAX_ROS_DOMAIN_ID - MIN_ROS_DOMAIN_ID + 1)) + MIN_ROS_DOMAIN_ID

def main():
    domain_id = -1
    whitelist_file = environ.get("FASTRPS_DEFAULT_PROFILES_FILE", None)
    if whitelist_file is None:
        raise RuntimeError("Whitelist file not defined, please define an environment variable FASTRPS_DEFAULT_PROFILES_FILE.")

    while True:
        domain_id = get_domain_id()
        print(f"Attempting with domain ID: {domain_id}...")
        proc_env = environ.copy()
        proc_env["ROS_DOMAIN_ID"] = str(domain_id)
        proc = subprocess.Popen(
            ["ros2", "run", "server", "main"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            env=proc_env
        )
        out, err = proc.communicate()

        if out.strip() != b"0":
            print("Failed to connect, reattempting.")
        else:
            print(f"Domain ID: {domain_id}")
            exit(0)

if __name__ == '__main__':
    main()
