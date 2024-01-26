import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import tf2_ros
import tf
import geometry_msgs.msg

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import numpy as np
import copy

class RosPublish():
    def __init__(self):
        # SWARM name
        self.swarm_name = rospy.get_name()[1:]
        print("Init swarm Publisher: ", self.swarm_name)

        # Getting ROSPARAM
        self.fixed_frame = str(rospy.get_param("/" + self.swarm_name + '/frame',"map"))
        self.robots_number = int(rospy.get_param("/" + self.swarm_name + '/num_robots',1))
        robot_mesh_path = str(rospy.get_param("/" + self.swarm_name + '/mesh_file',"package://swarm_visualization/meshes/test.stl"))

        self.hit_radius = float(rospy.get_param("/" + self.swarm_name + '/hit_radius',1.1))
        self.warning_radius = float(rospy.get_param("/" + self.swarm_name + '/warning_radius',2))        
        self.marker_array = MarkerArray()

        # Initializing arrays
        initial_marker = Marker()
        initial_marker.header.frame_id = self.fixed_frame
        initial_marker.type = initial_marker.MESH_RESOURCE
        initial_marker.mesh_resource = robot_mesh_path
        initial_marker.action = initial_marker.ADD
        initial_marker.scale.x = 0.1
        initial_marker.scale.y = 0.1
        initial_marker.scale.z = 0.1
        initial_marker.color.a = 1.0
        initial_marker.color.r = 0.0
        initial_marker.color.g = 1.0
        initial_marker.color.b = 0.0

        for i in range(self.robots_number):
            initial_marker.id = i
            self.marker_array.markers.append(copy.deepcopy(initial_marker))

        # Publishers
        self.marker_publisher = rospy.Publisher("/" + self.swarm_name + "/marker", MarkerArray,queue_size = 1)
        self.odom_publisher_list = []
        for id in range(self.robots_number):
            self.odom_publisher_list.append(rospy.Publisher("/" + self.swarm_name + "/robot_" + str(id) + "/odom", Odometry, queue_size = 10))
    
    def publish_tf(self, robots_position_x, robots_position_y, robots_position_yaw):
        tf_broadcast = tf2_ros.TransformBroadcaster()
        transform = geometry_msgs.msg.TransformStamped()
        
        transform.header.frame_id = self.fixed_frame
        transform.header.stamp = rospy.Time.now()        

        for i in range(self.robots_number):
            transform.child_frame_id = self.swarm_name + "_" + str(i)

            transform.transform.translation.x  = robots_position_x[i]
            transform.transform.translation.y = robots_position_y[i]
            quaternion = tf.transformations.quaternion_from_euler(0, 0, robots_position_yaw[i])
            transform.transform.rotation.x = quaternion[0]
            transform.transform.rotation.y = quaternion[1]
            transform.transform.rotation.z = quaternion[2]
            transform.transform.rotation.w = quaternion[3]

            tf_broadcast.sendTransform(transform)

    def publish_odom(self, robots_position_x, robots_position_y, robots_position_yaw, robots_velocity_vx, robots_velocity_wz):
        odom = Odometry()
        odom.header.frame_id = self.fixed_frame
        odom.header.stamp = rospy.Time.now()
        for i in range(self.robots_number):
            odom.pose.pose.position.x = robots_position_x[i]
            odom.pose.pose.position.y = robots_position_y[i]
            quaternion = tf.transformations.quaternion_from_euler(0, 0, robots_position_yaw[i])
            odom.pose.pose.orientation.x = quaternion[0]
            odom.pose.pose.orientation.y = quaternion[1]
            odom.pose.pose.orientation.z = quaternion[2]
            odom.pose.pose.orientation.w = quaternion[3]

            odom.twist.twist.linear.x = robots_velocity_vx[i]
            odom.twist.twist.angular.z = robots_velocity_wz[i]

            self.odom_publisher_list[i].publish(odom) 
    
    def set_marker_green(this, marker):
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
    
    def set_marker_red(this, marker):
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

    def set_marker_yellow(this, marker):
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
    
    def publish_marker_array(self, robots_position_x, robots_position_y, robots_position_yaw, robots_distance_matrix):
        for i in range(self.robots_number):
            self.marker_array.markers[i].header.stamp = rospy.Time.now()
            self.marker_array.markers[i].pose.position.x = robots_position_x[i]
            self.marker_array.markers[i].pose.position.y = robots_position_y[i]
            
            if np.min(robots_distance_matrix[i]) < self.hit_radius:
                self.set_marker_red(self.marker_array.markers[i])
            elif np.min(robots_distance_matrix[i]) < self.warning_radius:
                self.set_marker_yellow(self.marker_array.markers[i])
            else:
                self.set_marker_green(self.marker_array.markers[i])

            quaternion = tf.transformations.quaternion_from_euler(0, 0, robots_position_yaw[i] - np.pi/2)
            self.marker_array.markers[i].pose.orientation.x = quaternion[0]
            self.marker_array.markers[i].pose.orientation.y = quaternion[1]
            self.marker_array.markers[i].pose.orientation.z = quaternion[2]
            self.marker_array.markers[i].pose.orientation.w = quaternion[3]

        self.marker_publisher.publish(self.marker_array)

    def publish_all(self, robot_state):
        (robots_position_x,
        robots_position_y,
        robots_position_yaw,
        robots_velocity_vx,
        robots_velocity_wz,
        robots_distance_matrix) = robot_state
        self.publish_tf(robots_position_x,
                        robots_position_y,
                        robots_position_yaw)    
        self.publish_odom(robots_position_x,
                        robots_position_y,
                        robots_position_yaw,
                        robots_velocity_vx,
                        robots_velocity_wz)
        self.publish_marker_array(robots_position_x,
                        robots_position_y,
                        robots_position_yaw,
                        robots_distance_matrix) 

class RosSubscriber():
    def __init__(self):
          # SWARM name
        self.swarm_name = rospy.get_name()[1:]
        print("Init swarm subscriber: ", self.swarm_name)
        self.robots_number = int(rospy.get_param("/" + self.swarm_name + '/num_robots',1))

        #Subscriber
        self.twister_subcriber_list = []
        for id in range(self.robots_number):
            self.twister_subcriber_list = rospy.Subscriber("/" + self.swarm_name + "/robot_" + str(id) + "/cmd_vel",
                                                Twist, self.callback_twist, id)
        
        self.velocity_vx_setpoit = np.zeros(self.robots_number)
        self.velocity_wz_setpoit = np.zeros(self.robots_number)

    def callback_twist(self, msg, i):
        self.velocity_vx_setpoit[i] = msg.linear.x
        self.velocity_wz_setpoit[i] = msg.angular.z

    def get_velocity_set_point(self):
        return (self.velocity_vx_setpoit,
               self.velocity_wz_setpoit)
