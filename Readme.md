# Node

## Swarm_node
The Swarm_node creates a swarm vizualization on Rviz, this takes Twists inputs and generates Rviz vizualization for the robots

### Subscribed Topics
/"swarm_name"/robot_"i"/cmd_vel
    That's topic represents the Twist input for robot number "i" of "swarm_name" swarm

### Published topics
/"swarm_name"/robot_"i"/cmd_vel
        That's topic represents the Odometry of robot number "i" of "swarm_name" swarm
/"swarm_name"/marker
        That's topic publisher the marker array swarm for Rviz vizualization

### Parameters
~mesh_file (string, default: "package://swarm_visualization/meshes/test.stl")
    The mesh file for vizualization
~num_robots (int, default: 1)
    The number of robots
~hit_radius (float, default: 1.1)
    The hiting radius
~warning_radius (float, default: 2)
    The warning radius
~frame (string, default: "map")
    The fixed frame attached to the map
~x_"i" (float, default: 0)
    The initial x position position of robot "i"
~y_"i" (float, default: 0)
    The initial y position position of robot "i"

### Provided tf Transforms
map → "swarm_name"_"i"
    the current pose of the robot's "i" within the map frame
