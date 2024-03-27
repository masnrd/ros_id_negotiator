# ros_id_negotiator

After `pppd` establishes an IP link, we need to establish a shared `ROS_DOMAIN_ID`. 
- The intent is to allow for the drone to automatically establish ROS2 communications with the mission control, without needing to SSH into the drone's onboard computer.

This establishes this shared ID in the following manner:
1. Server listens on UDP socket `6969`. 
2. At the same time, the drone:
    1. Generates a `ROS_DOMAIN_ID` based on a coarse timestamp (to allow for retries later).
    2. Starts a ROS2 node with a service, on that `ROS_DOMAIN_ID`.
2. Drone sends a UDP packet containing the `ROS_DOMAIN_ID`.
3. Upon receiving that `ROS_DOMAIN_ID`, the server spins up a ROS2 node with that domain ID context, and sends a ROS2 Service Request to the drone.
4. The drone responds with a ROS2 Service Response. Through these steps, both the drone and the server now confirm that they can communicate over ROS2.
5. The drone automatically saves this `ROS_DOMAIN_ID` to `~/.droneconfig.sh`, allowing it to be sourced by a script later in the pipeline.

## Setup
Modify `env.sh` to source your ROS distribution.

### Server
```bash
colcon build
```

### Drone
```bash
colcon build
```

## Usage
### Server
```
source env.sh
ros2 run server main
```

This would output the established `ROS2_DOMAIN_ID` after a connection is established.

### Drone
```
source env.sh
ros2 run client main
```

The `ROS_DOMAIN_ID` would be stored in the `~/.droneconfig.sh` shell script. To automatically set it for later scripts, simply add `source ~/.droneconfig.sh` to the next script in the pipeline.