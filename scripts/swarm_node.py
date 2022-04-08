#!/usr/bin/env python3
import rospy
from swarm import swarmClass




# def update_pose(this,dt):
#     if (rospy.Time.now() - this.cmd_time).secs > 0.5:
#         this.vx = 0
#         this.wz = 0


rospy.init_node("swarm_name")

swarm = swarmClass()

while True:
    swarm.update()
    rospy.sleep(0.02)


