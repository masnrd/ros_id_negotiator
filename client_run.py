from __future__ import annotations
from os import environ
import subprocess
from pathlib import Path
from time import time_ns

TIME_FRAME_NS = pow(10, 9) * 30  # ROS_DOMAIN_ID would be different every 30 seconds (so if restarted, new ROS_DOMAIN_ID)
MIN_ROS_DOMAIN_ID = 1
MAX_ROS_DOMAIN_ID = 232
def get_domain_id() -> int:
    val = time_ns() // TIME_FRAME_NS
    return (val % (MAX_ROS_DOMAIN_ID - MIN_ROS_DOMAIN_ID + 1)) + MIN_ROS_DOMAIN_ID

DATA_FILE = Path.home().joinpath(".droneconfig.sh")

def main():
    domain_id = -1

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