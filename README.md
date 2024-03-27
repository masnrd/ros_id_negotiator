# ros_id_negotiator

After `pppd` establishes an IP link, we need to establish a shared `ROS_DOMAIN_ID`. 
- The intent is to allow for the drone to automatically establish ROS2 communications with the mission control, without needing to SSH into the drone's onboard computer.

This establishes this shared ID in the following manner:
1. The drone:
    1. Generates a `ROS_DOMAIN_ID` based on a coarse timestamp which changes every 30 seconds.
    2. Starts a ROS2 node with a service, on that `ROS_DOMAIN_ID`.
2. The server:
    1. Generates a `ROS_DOMAIN_ID` based on a coarse timestamp which changes every 30 seconds.
    2. Starts a ROS2 node on that `ROS_DOMAIN_ID`, and sends ROS2 Service Requests to the drone every few seconds.
3. The drone knows that the `ROS_DOMAIN_ID` is synchronised when the drone receives this service request. The drone responds with a ROS2 Service Response.
4. The server now knows that the `ROS_DOMAIN_ID` it has guessed is the correct one. Through these steps, both the server and the drone have synchronised their `ROS_DOMAIN_ID`.
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
python3 server_run.py
```

This would output the established `ROS2_DOMAIN_ID` after a connection is established.

### Drone
```
source env.sh
python3 client_run.py
```

The `ROS_DOMAIN_ID` would be stored in the `~/.droneconfig.sh` shell script. To automatically set it for later scripts, simply add `source ~/.droneconfig.sh` to the next script in the pipeline.