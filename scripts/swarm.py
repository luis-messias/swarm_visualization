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

class swarmClass():   

    def __init__(self):
        # SWARM name
        self.swarm_name = rospy.get_name()[1:]
        print("Init swarm: ", self.swarm_name)
        now = rospy.get_rostime()
        self.t_last_update = now.secs +  now.nsecs/1000000000


        # Getting ROSPARAM
        self.fixed_frame = str(rospy.get_param("/" + self.swarm_name + '/frame',"map"))
        self.robots_number = int(rospy.get_param("/" + self.swarm_name + '/num_robots',1))
        robot_mesh_path = str(rospy.get_param("/" + self.swarm_name + '/mesh_file',"package://swarm_visualization/meshes/test.stl"))

        self.hit_radius = float(rospy.get_param("/" + self.swarm_name + '/hit_radius',1.1))
        self.warning_radius = float(rospy.get_param("/" + self.swarm_name + '/warning_radius',2))
        #self.cmd_time_out = float(rospy.get_param("/" + self.swarm_name + '/cmd_time_out',1))
        
        #Arrays
        self.robots_position_x = np.zeros(self.robots_number)
        self.robots_position_y = np.zeros(self.robots_number)
        self.robots_position_yaw = np.zeros(self.robots_number)
        self.robots_distance_matrix = np.zeros((self.robots_number,self.robots_number))

        self.robots_velocity_vx = np.zeros(self.robots_number)
        self.robots_velocity_wz = np.zeros(self.robots_number)
        self.velocity_vx_setpoit = np.zeros(self.robots_number)
        self.velocity_wz_setpoit = np.zeros(self.robots_number)

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
            self.robots_position_x[i] = rospy.get_param("/" + self.swarm_name + '/x_' + str(i),0)
            self.robots_position_y[i] = rospy.get_param("/" + self.swarm_name + '/y_' + str(i),0)
            
            initial_marker.id = i

            self.marker_array.markers.append(copy.deepcopy(initial_marker))

        # Publishers
        self.marker_publisher = rospy.Publisher("/" + self.swarm_name + "/marker", MarkerArray,queue_size = 1)
        self.odom_publisher_list = []
        for id in range(self.robots_number):
            self.odom_publisher_list.append(rospy.Publisher("/" + self.swarm_name + "/robot_" + str(id) + "/odom", Odometry, queue_size = 10))
        
        #Subscriber
        self.twister_subcriber_list = []
        for id in range(self.robots_number):
            self.twister_subcriber_list = rospy.Subscriber("/" + self.swarm_name + "/robot_" + str(id) + "/cmd_vel",
                                                Twist, self.callback_twist, id)

    def callback_twist(self, msg, i):
        self.velocity_vx_setpoit[i] = msg.linear.x
        self.velocity_wz_setpoit[i] = msg.angular.z

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
    
    def publish_marker_array(self):
        for i in range(self.robots_number):
            self.marker_array.markers[i].pose.position.x = self.robots_position_x[i]
            self.marker_array.markers[i].pose.position.y = self.robots_position_y[i]
            
            if np.min(self.robots_distance_matrix[i]) < self.hit_radius:
                self.set_marker_red(self.marker_array.markers[i])
            elif np.min(self.robots_distance_matrix[i]) < self.warning_radius:
                self.set_marker_yellow(self.marker_array.markers[i])
            else:
                self.set_marker_green(self.marker_array.markers[i])

            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.robots_position_yaw[i] - np.pi/2)
            self.marker_array.markers[i].pose.orientation.x = quaternion[0]
            self.marker_array.markers[i].pose.orientation.y = quaternion[1]
            self.marker_array.markers[i].pose.orientation.z = quaternion[2]
            self.marker_array.markers[i].pose.orientation.w = quaternion[3]

        self.marker_publisher.publish(self.marker_array)

    def publish_odom(self):
        odom = Odometry()
        odom.header.frame_id = self.fixed_frame
        
        for i in range(self.robots_number):
            odom.pose.pose.position.x = self.robots_position_x[i]
            odom.pose.pose.position.y = self.robots_position_y[i]
            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.robots_position_yaw[i])
            odom.pose.pose.orientation.x = quaternion[0]
            odom.pose.pose.orientation.y = quaternion[1]
            odom.pose.pose.orientation.z = quaternion[2]
            odom.pose.pose.orientation.w = quaternion[3]

            odom.twist.twist.linear.x = self.robots_velocity_vx[i]
            odom.twist.twist.angular.z = self.robots_velocity_wz[i]

            self.odom_publisher_list[i].publish(odom)

    def publish_tf(self):
        tf_broadcast = tf2_ros.TransformBroadcaster()
        transform = geometry_msgs.msg.TransformStamped()
        
        transform.header.frame_id = self.fixed_frame
        transform.header.stamp = rospy.Time.now()        

        for i in range(self.robots_number):
            transform.child_frame_id = self.swarm_name + "_" + str(i)

            transform.transform.translation.x  = self.robots_position_x[i]
            transform.transform.translation.y = self.robots_position_y[i]
            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.robots_position_yaw[i])
            transform.transform.rotation.x = quaternion[0]
            transform.transform.rotation.y = quaternion[1]
            transform.transform.rotation.z = quaternion[2]
            transform.transform.rotation.w = quaternion[3]

            tf_broadcast.sendTransform(transform)

    def publish_all(self):
        self.publish_marker_array()
        self.publish_odom()
        self.publish_tf()

    def update_distance_matrix(self):
        for i in range(self.robots_number):
            for j in range(self.robots_number): 
                if i == j:
                    self.robots_distance_matrix[i][j] = np.inf
                else:
                    self.robots_distance_matrix[i][j] =  (self.robots_position_x[i] - self.robots_position_x[j]) ** 2
                    self.robots_distance_matrix[i][j] += (self.robots_position_y[i] - self.robots_position_y[j]) ** 2
                    self.robots_distance_matrix[i][j] =   self.robots_distance_matrix[i][j] ** 0.5
    
    def update_velocity_limiter(self, dt):
        self.robots_velocity_vx = self.velocity_vx_setpoit
        self.robots_velocity_wz = self.velocity_wz_setpoit

    def integrate_system(self, dt):
        self.robots_position_x   += self.robots_velocity_vx*np.cos(self.robots_position_yaw)*dt
        self.robots_position_y   += self.robots_velocity_vx*np.sin(self.robots_position_yaw)*dt
        self.robots_position_yaw += self.robots_velocity_wz*dt

    def update(self, ):
        now = rospy.get_rostime()
        dt = now.secs +  now.nsecs/1000000000 - self.t_last_update
        self.t_last_update += dt

        #print("Update: ", dt)
        self.update_velocity_limiter(dt)
        self.integrate_system(dt)
        self.update_distance_matrix()
        self.publish_all()
