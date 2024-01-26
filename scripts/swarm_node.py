#!/usr/bin/env python3
import rospy
from modules.swarm_class import SwarmClass
from modules.swarm_ros_interface import RosPublish, RosSubscriber

rospy.init_node("swarm_name")
swarm = SwarmClass()
publisher = RosPublish()
subscriber = RosSubscriber()

dt = 0.02
while True:
    velocity_vx_setpoit, velocity_wx_setpoit = subscriber.get_velocity_set_point()
    robot_state = swarm.update(dt, velocity_vx_setpoit, velocity_wx_setpoit)
    publisher.publish_all(robot_state)

    rospy.sleep(dt)


