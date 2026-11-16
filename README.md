# ECEN 433 - Final Project: Campus Navigation

Your Duckiebot gets a starting road, a list of places to visit and a parking
stall. It drives the route through the city on its own, then parks.

The project description lives on the course site. This README covers the
repository: what is in it, and how to build and run it.


## What's here

`packages/intersection_control/`

| File | What it is |
| --- | --- |
| `src/duckietown_map.py` | The city: intersections, roads, lengths, and which turn leads where |
| `src/route_gui_node.py` | Map window for sending routes and watching progress |
| `src/tag_pose.py` | AprilTag detection → tag pose in the robot frame |
| `src/tag_db.py` | AprilTag id → sign type (from Lab 6) |
| `src/switch_on_node.py` | Turns on the stop line filter, lane filter and tag detector |
| `config/parking_lot.yaml` | Parking lot tags and the transforms between them |
| `launch/route_gui.launch` | Runs on your laptop |

Bring in from Lab 6: your nodes, your robot's `config/DUCKIEBOT_NAME.yaml`, and
`launch/intersections.launch`. Inside its `<group>`, add

```xml
<rosparam file="$(find intersection_control)/config/parking_lot.yaml" ns="parking_lot"/>
```


## Building and running

On the robot, with your Lab 4 stack already running:

```bash
dts devel build -H DUCKIEBOT_NAME -f
dts devel run   -H DUCKIEBOT_NAME -L intersections
```

On your laptop, the route GUI talking to the robot:

```bash
dts devel build -f
dts devel run -R DUCKIEBOT_NAME -L route-gui -X
```

Or with no robot at all:

```bash
dts devel run -L route-gui -X ++ test:=true
```

Anything after `++` is passed to `roslaunch`.

To attach a second terminal to a container that is already running:

```bash
dts devel run attach
```

New Python nodes must be executable or ROS will not find them:

```bash
chmod +x packages/intersection_control/src/<node_name>.py
```
