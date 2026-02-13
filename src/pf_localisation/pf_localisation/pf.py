from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point
from . pf_base import PFLocaliserBase

from . util import rotateQuaternion, getHeading
import random
import math


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self, logger, clock):
        # ----- Call the superclass constructor
        super().__init__(logger, clock)
        
        # ----- Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0.02
        self.ODOM_TRANSLATION_NOISE = 0.02
        self.ODOM_DRIFT_NOISE =0.02

        self.INITIAL_POSE_NOISE_X=0.2
        self.INITIAL_POSE_NOISE_Y=0.2
        self.INITIAL_POSE_NOISE_THETA=0.2

        self.NUMBER_OF_PARTICLE= 1000
        
 
        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict
        
       
    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """

        particle_cloud = PoseArray()
        particle_cloud.header.frame_id = "map"
        particle_cloud.header.stamp = self._clock.now().to_msg()

        init_x = initialpose.pose.pose.position.x
        init_y = initialpose.pose.pose.position.y
        init_theta = getHeading(initialpose.pose.pose.orientation)
     



        for i in range(self.NUMBER_OF_PARTICLE): 
            x = random.gauss(init_x,self.ODOM_TRANSLATION_NOISE)
            y = random.gauss(init_y,self.ODOM_DRIFT_NOISE)
            theta = random.gauss(init_theta,self.ODOM_ROTATION_NOISE)

            particle = Pose()
            particle.position = Point(x=x,y=y,z=0.0)
            particle.orientation = rotateQuaternion(Quaternion(w=1.0),theta)
            particle_cloud.poses.append(particle)
            
        self.particlecloud = particle_cloud
        return particle_cloud
    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
        particle_weights = []
        for particle in self.particlecloud.poses:
            weight = self.sensor_model.get_weight(scan,particle)
            particle_weights.append(weight)

        total_weight = sum(particle_weights)
        if total_weight ==0:
            particle_weights = [1.0/len(particle_weights) ]* len(particle_weights)
        else:
            for i in range(len(particle_weights)):
                particle_weights[i] = particle_weights[i]/total_weight

        square_total = 0.0
        for j in particle_weights:
            square_total += j*j
        effective_particle = 1.0/square_total
        
        
        if effective_particle < self.NUMBER_OF_PARTICLE:
            resampled = []
            num_particles = len(self.particlecloud.poses)
            
            num_random_particles = int(num_particles*0.001)
            num_resampled_particles = num_particles - num_random_particles
            
            ran = random.uniform(0,1.0/num_resampled_particles)
            wei = particle_weights[0]
            i = 0
            for q in range(num_resampled_particles):
                a = ran + q*(1.0/num_resampled_particles)
                while a > wei:
                    i+=1
                    wei+=particle_weights[i]
                selected_particle = self.particlecloud.poses[i]
                x = selected_particle.position.x + random.gauss(0,0.01)
                y = selected_particle.position.y + random.gauss(0,0.01)
                theta = getHeading(selected_particle.orientation) + random.gauss(0,0.01)

                new_particle = Pose()
                new_particle.position = Point(x=x,y=y,z=0.0)
                new_particle.orientation = rotateQuaternion(Quaternion(w=1.0),theta)
                resampled.append(new_particle)
                
                
            for i in range(num_random_particles):
                random_x = random.uniform(10,25)
                random_y = random.uniform(10,25)
                random_theta = random.uniform(-math.pi,math.pi)
                random_particle = Pose()
                random_particle.position = Point(x=random_x,y=random_y,z=0.0)
                random_particle.orientation = rotateQuaternion(Quaternion(w=1.0),random_theta)
                resampled.append(random_particle)
            self.particlecloud.poses = resampled


        

        

    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """
        if not self.particlecloud:
            return Pose()
        
        particles = self.particlecloud.poses
        num_of_par = len(particles)
        #coordinate of x and y
        x_coords = sorted([p.position.x for p in particles])
        y_coords = sorted([p.position.y for p in particles])
        #calculated the median of x and y
        median_x = x_coords[num_of_par // 2]
        median_y = y_coords[num_of_par // 2]
        #the distance of particle from center of cluster
        distance = 5.0
        selected_particles = []
        #calculate the distance of each particle from the center of cluster and select the particles within the distance
        for p in particles:
            dist = math.sqrt((p.position.x - median_x)**2 + (p.position.y - median_y)**2)
            if dist < distance:
                selected_particles.append(p)
    
        if len(selected_particles) < num_of_par * 0.4:
            selected_particles = particles
    
        num_selected = len(selected_particles)
        
        
        
        #mean of x
        total_x = 0.0
        for particle in selected_particles:
            total_x += particle.position.x
        mean_x = total_x/num_selected

        #mean of y
        total_y = 0.0
        for particle in selected_particles:
            total_y += particle.position.y
        mean_y = total_y/num_selected
        
        #quarternion mean of x
        total_quat_x = 0.0
        for particle in selected_particles:
            total_quat_x += particle.orientation.x
        mean_quat_x = total_quat_x/num_selected

        #quarternion mean of y
        total_quat_y = 0.0
        for particle in selected_particles:
            total_quat_y += particle.orientation.y
        mean_quat_y = total_quat_y/num_selected

        #quarternion mean of z
        total_quat_z = 0.0
        for particle in selected_particles:
            total_quat_z += particle.orientation.z
        mean_quat_z = total_quat_z/num_selected

        #quarternion mean of w
        total_quat_w = 0.0
        for particle in selected_particles:
            total_quat_w += particle.orientation.w
        mean_quat_w = total_quat_w/num_selected

        euclidean = math.sqrt(mean_quat_x**2 + mean_quat_y**2+mean_quat_z**2+mean_quat_w**2)
        mean_quat_x /=euclidean
        mean_quat_y /= euclidean
        mean_quat_z /=euclidean
        mean_quat_w /=euclidean

        estimate_pose = Pose()
        estimate_pose.position = Point(x=mean_x,y=mean_y,z=0.0)
        estimate_pose.orientation = Quaternion(x=mean_quat_x,y=mean_quat_y,z=mean_quat_z,w=mean_quat_w)
        return estimate_pose





        
