#!/usr/bin/env python3

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import rospy
import math
import tf2_ros
import geometry_msgs.msg
import numpy as np

class robot():   

    def __init__(this, swarm_name, id, meshes, frame_id, X0 = 0, Y0 = 0, Z0 = 0, YAW0 = 0 ):
        this.x = X0
        this.y = Y0
        this.yaw = YAW0
        this.vx = 0
        this.wz = 0
        

        this.marker = Marker()
        this.marker.header.frame_id = frame_id
        this.marker.type = this.marker.MESH_RESOURCE
        this.marker.mesh_resource = meshes
        this.marker.action = this.marker.ADD
        this.marker.scale.x = 0.2
        this.marker.scale.y = 0.2
        this.marker.scale.z = 0.2
        this.marker.color.a = 1.0
        this.marker.color.r = 0.0
        this.marker.color.g = 1.0
        this.marker.color.b = 0.0

        quaternion = tf.transformations.quaternion_from_euler(0, 0, this.yaw - math.pi/2)
        this.marker.pose.orientation.x = quaternion[0]
        this.marker.pose.orientation.y = quaternion[1]
        this.marker.pose.orientation.z = quaternion[2]
        this.marker.pose.orientation.w = quaternion[3]

        this.marker.pose.position.x = X0
        this.marker.pose.position.y = Y0 
        this.marker.pose.position.z = Z0 

        this.odom = Odometry()
        this.odom.header.frame_id = frame_id
        this.odom.pose.pose.position.x = X0
        this.odom.pose.pose.position.y = Y0
        this.odom.pose.pose.position.z = Z0

 
        this.subscriber = rospy.Subscriber("/" + swarm_name + "/robot_" + str(id) + "/cmd_vel",
                                             Twist, this.cb_cmd)
        this.publisher = rospy.Publisher("/" + swarm_name + "/robot_" + str(id) + "/odom", Odometry, queue_size = 10)

        this.tf_broadcast = tf2_ros.TransformBroadcaster()
        this.transform = geometry_msgs.msg.TransformStamped()

        this.transform.header.stamp = rospy.Time.now()
        this.transform.header.frame_id = frame_id
        this.transform.child_frame_id = swarm_name + "_" + str(id)
        this.transform.transform.translation.x = X0
        this.transform.transform.translation.y = Y0
        this.transform.transform.translation.z = Z0
        this.transform.transform.rotation.x = quaternion[0]
        this.transform.transform.rotation.y = quaternion[1]
        this.transform.transform.rotation.z = quaternion[2]
        this.transform.transform.rotation.w = quaternion[3]

    def set_marker_green(this):
        this.marker.color.r = 0.0
        this.marker.color.g = 1.0
        this.marker.color.b = 0.0
    
    def set_marker_red(this):
        this.marker.color.r = 1.0
        this.marker.color.g = 0.0
        this.marker.color.b = 0.0

    def set_marker_yellow(this):
        this.marker.color.r = 1.0
        this.marker.color.g = 1.0
        this.marker.color.b = 0.0

    def get_marker(this):
        return this.marker

    def cb_cmd(this, msg):
        this.vx = msg.linear.x
        this.wz = msg.angular.z

    def update_pose(this,dt):
        this.x += this.vx*dt*math.cos(this.yaw)
        this.y += this.vx*dt*math.sin(this.yaw)
        this.yaw += this.wz*dt

        quaternion = tf.transformations.quaternion_from_euler(0, 0, this.yaw - math.pi/2)
        this.marker.pose.position.x = this.x
        this.marker.pose.position.y = this.y
        this.marker.pose.orientation.x = quaternion[0]
        this.marker.pose.orientation.y = quaternion[1]
        this.marker.pose.orientation.z = quaternion[2]
        this.marker.pose.orientation.w = quaternion[3]

        quaternion = tf.transformations.quaternion_from_euler(0, 0, this.yaw)
        this.odom.pose.pose.position.x = this.x
        this.odom.pose.pose.position.y = this.y
        this.odom.pose.pose.orientation.x = quaternion[0]
        this.odom.pose.pose.orientation.y = quaternion[1]
        this.odom.pose.pose.orientation.z = quaternion[2]
        this.odom.pose.pose.orientation.w = quaternion[3]

        this.transform.header.stamp = rospy.Time.now()
        this.transform.transform.translation.x = this.x
        this.transform.transform.translation.y = this.y
        this.transform.transform.rotation.x = quaternion[0]
        this.transform.transform.rotation.y = quaternion[1]
        this.transform.transform.rotation.z = quaternion[2]
        this.transform.transform.rotation.w = quaternion[3]

        this.tf_broadcast.sendTransform(this.transform)
        this.publisher.publish(this.odom)

rospy.init_node("swarm_name")
swarm_name = rospy.get_name()[1:]

frame = str(rospy.get_param("/" + swarm_name + '/frame',"map"))
number = int(rospy.get_param("/" + swarm_name + '/num_robots',1))
mesh = str(rospy.get_param("/" + swarm_name + '/mesh_file',"package://swarm_visualization/meshes/test.stl"))
r1 = float(rospy.get_param("/" + swarm_name + '/r1',1.1))
r2 = float(rospy.get_param("/" + swarm_name + '/r2',2))

robots = [] 
markerArray = MarkerArray()
distancias = np.zeros((number,number))

for i in range(number):
    robots.append(robot(swarm_name, i, mesh, frame,
                  rospy.get_param("/" + swarm_name + '/x_' + str(i),0),
                  rospy.get_param("/" + swarm_name + '/y_' + str(i),0),
                  0,
                  0))
    markerArray.markers.append(robots[i].get_marker())

id = 0
for m in markerArray.markers:
    m.id = id
    id += 1

now = rospy.get_rostime()
t_last_update = now.secs +  now.nsecs/1000000000
publisher = rospy.Publisher("/" + swarm_name + "/marker", MarkerArray,queue_size = 1)

while not rospy.is_shutdown():

    now = rospy.get_rostime()
    dt = now.secs +  now.nsecs/1000000000 - t_last_update
    t_last_update += dt

    for i in range(number):
        for j in range(number):
            distancias[i][j] = ((robots[i].x - robots[j].x)**2 + (robots[i].y - robots[j].y)**2)**0.5

    for i in range(number):
        flag = 0
        for j in range(number):
            if i != j:
                if distancias[i][j] < r1:
                    flag = max(flag,2)
                elif distancias[i][j] < r2:
                    flag = max(flag,1)
                else:
                    flag = max(flag,0)

        if flag == 2:
            robots[i].set_marker_red()
        elif flag == 1:
            robots[i].set_marker_yellow()
        else:
            robots[i].set_marker_green()
        print(distancias)
        robots[i].update_pose(dt)
        markerArray.markers[i] = robots[i].get_marker()
        
    publisher.publish(markerArray)

    rospy.sleep(0.01)