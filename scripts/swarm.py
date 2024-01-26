import rospy
import numpy as np

class SwarmClass():

    def __init__(self):
        # SWARM name
        self.swarm_name = rospy.get_name()[1:]
        print("Init swarm: ", self.swarm_name)

        # Getting ROSPARAM
        self.fixed_frame = str(rospy.get_param("/" + self.swarm_name + '/frame',"map"))
        self.robots_number = int(rospy.get_param("/" + self.swarm_name + '/num_robots',1))
        
        #Velocity and aceleration limiter
        self.has_velocity_limiter = bool(rospy.get_param("/" + self.swarm_name + '/has_velocity_limiter', 0))
        self.max_vx = float(rospy.get_param("/" + self.swarm_name + '/max_vx', 1))
        self.max_wz = float(rospy.get_param("/" + self.swarm_name + '/max_wz', 1))

        self.has_acceleration_limiter = bool(rospy.get_param("/" + self.swarm_name + '/has_acceleration_limiter',0))
        self.max_acc_x = float(rospy.get_param("/" + self.swarm_name + '/max_acc_x', 1))
        self.max_acc_z = float(rospy.get_param("/" + self.swarm_name + '/max_acc_z', 1))
        
        #Arrays
        self.robots_position_x = np.zeros(self.robots_number)
        self.robots_position_y = np.zeros(self.robots_number)
        self.robots_position_yaw = np.zeros(self.robots_number)
        self.robots_distance_matrix = np.zeros((self.robots_number,self.robots_number))

        self.robots_velocity_vx = np.zeros(self.robots_number)
        self.robots_velocity_wz = np.zeros(self.robots_number)
        

        for i in range(self.robots_number):
            self.robots_position_x[i] = rospy.get_param("/" + self.swarm_name + '/x_' + str(i),0)
            self.robots_position_y[i] = rospy.get_param("/" + self.swarm_name + '/y_' + str(i),0)


    def update_distance_matrix(self):
        for i in range(self.robots_number):
            for j in range(self.robots_number): 
                if i == j:
                    self.robots_distance_matrix[i][j] = np.inf
                else:
                    self.robots_distance_matrix[i][j] =  (self.robots_position_x[i] - self.robots_position_x[j]) ** 2
                    self.robots_distance_matrix[i][j] += (self.robots_position_y[i] - self.robots_position_y[j]) ** 2
                    self.robots_distance_matrix[i][j] =   self.robots_distance_matrix[i][j] ** 0.5
    
    def update_velocity_limiter(self, dt, velocity_vx_setpoit, velocity_wz_setpoit):
        #Limiting acceleration
        if self.has_acceleration_limiter:
            Dvx_max = self.max_acc_x * dt
            Dwz_max = self.max_acc_z * dt
            self.robots_velocity_vx = np.clip(velocity_vx_setpoit, self.robots_velocity_vx - Dvx_max, self.robots_velocity_vx + Dvx_max)
            self.robots_velocity_wz = np.clip(velocity_wz_setpoit, self.robots_velocity_wz - Dwz_max, self.robots_velocity_wz + Dwz_max)
        else:
            self.robots_velocity_vx = velocity_vx_setpoit
            self.robots_velocity_wz = velocity_wz_setpoit

        #Limiting velocity
        if self.has_velocity_limiter:
            self.robots_velocity_vx = np.clip(self.robots_velocity_vx, -self.max_vx, self.max_vx)
            self.robots_velocity_wz = np.clip(self.robots_velocity_wz, -self.max_wz, self.max_wz)
        else:
            self.robots_velocity_vx = velocity_vx_setpoit
            self.robots_velocity_wz = velocity_wz_setpoit

    def integrate_system(self, dt):
        self.robots_position_x   += self.robots_velocity_vx*np.cos(self.robots_position_yaw)*dt
        self.robots_position_y   += self.robots_velocity_vx*np.sin(self.robots_position_yaw)*dt
        self.robots_position_yaw += self.robots_velocity_wz*dt

    def update(self, dt, velocity_vx_setpoit, velocity_wz_setpoit):
        self.update_velocity_limiter(dt, velocity_vx_setpoit, velocity_wz_setpoit)
        self.integrate_system(dt)
        self.update_distance_matrix()
        return self.get_state()

    def get_state(self):
        return  (self.robots_position_x,
                self.robots_position_y,
                self.robots_position_yaw,
                self.robots_velocity_vx,
                self.robots_velocity_wz,
                self.robots_distance_matrix)
    

    
