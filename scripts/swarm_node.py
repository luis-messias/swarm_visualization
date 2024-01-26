#!/usr/bin/env python3
import rospy
from swarm import SwarmClass, RosPublish, RosSubscriber

rospy.init_node("swarm_name")

swarm = SwarmClass()
publisher = RosPublish()
subscriber = RosSubscriber()
dt = 0.02
while True:
    (velocity_vx_setpoit,
    velocity_wx_setpoit) = subscriber.get_velocity_set_point()
    swarm.update(dt, velocity_vx_setpoit, velocity_wx_setpoit)
    robot_state = swarm.get_state()
    publisher.publish_all(robot_state)  


    rospy.sleep(dt)


