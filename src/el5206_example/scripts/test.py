#!/usr/bin/env python3
"""
This is the main script used for the EL5206 Robotics component. This script is
written Python 3 and defines a EL5206_Robot node for you to program. You will
find notes on how the code works, auxiliary functions and some for you to 
program.

Authors: Your dear 2022.Spring.teaching_staff
(Eduardo Jorquera, Ignacio Dassori & Felipe Vergara)
"""
import cv2
import matplotlib.pyplot as plt
import math
import numpy as np
import rospkg
import rospy
import sys
import tf
import tf2_ros
import time
import yaml

from PIL import Image
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image

class EL5206_Robot:
    def __init__(self):
        # Initialize ROS Node
        rospy.init_node('EL5206_Main_Node', anonymous=False)

        # Attributes
        self.robot_frame_id = 'base_footprint'
        self.odom_frame_id  = 'odom'
        self.currentScan =  None
        self.currentImage = None
        self.currentDepth = None

        self.odom_x   = None
        self.odom_y   = None
        self.odom_yaw = None
        
        self.gt_x     = None
        self.gt_y     = None
        self.gt_yaw   = None
        self.odom_lst = []
        self.gt_lst   = []
        
        self.poses_to_save = 300 

        self.target_x   = None
        self.target_y   = None
        self.target_yaw = None
        self.path = rospkg.RosPack().get_path('el5206_example')

        self.camera_fov = 1.089

        # Extra variable to print odometry
        self.odom_i = 0

        # Subscribers
        rospy.Subscriber("/odom",               Odometry,  self.odometryCallback)
        rospy.Subscriber("/ground_truth/state", Odometry,  self.groundTruthCallback)
        rospy.Subscriber("/scan",               LaserScan, self.scanCallback)
        rospy.Subscriber("/target_pose",        Pose2D,    self.poseCallback)
        rospy.Subscriber("/stalker/camera_link_camera/camera_link_camera/color/image_raw",  Image,     self.imageCallback)
        rospy.Subscriber("/stalker/camera_link_camera/camera_link_camera/depth/image_raw",  Image,     self.depthCallback)

        # Publishers
        self.vel_pub = rospy.Publisher('/stalker/cmd_vel', Twist, queue_size=1)

        # Timer
        self.update_timer = rospy.Timer( rospy.Duration(1.0), self.timerCallback )


    """
    Callback functions are executed every time a ROS messaage is received by the
    Subscirbers defined in the __init__ method. You have to be careful with 
    these methods, because they are executed several times a second and should 
    be quick enough to finnish before the next message arrives. 

    Be careful with the variables stored in a callback, because these methods 
    can be threaded, they may change some of the variables you are using in the 
    main code.
    """

    def imageCallback(self, msg):
        image_np = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        self.currentImage = image_np #[:,:,:3]
        #self.currentDepth = image_np[:,:,3]
        #self.currentImage = self.bridge.imgmsg_to_cv(msg, "brg8")


    def depthCallback(self, msg):
        image_np = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        self.currentDepth = image_np#[:,:,3]


    def scanCallback(self, msg):
        """
        Receives a LaserScan message and saves it self.currentScan.
        """
        self.currentScan = msg


    def odometryCallback(self, msg):
        """
        Receives an Odometry message. Uses the auxiliary method odom2Coords()
        to get the (x,y,yaw) coordinates and save the in the self.odom_x, 
        self.odom_y and self.odom_yaw attributes.
        """
        """
        self.odom_i += 1
        if self.odom_i%30==0:
            # Print one every 30 msgs
            print("This is the Odometry message:")
            print(msg)
        """
        self.odom_x, self.odom_y, self.odom_yaw = self.odom2Coords(msg)


    def groundTruthCallback(self, msg):
        """
        Receives an Odometry message. Uses the auxiliary method odom2Coords()
        to get the (x,y,yaw) coordinates ans saves the in the self.gt_x, 
        self.gt_y and self.gt_yaw attributes
        """
        self.gt_x, self.gt_y, self.gt_yaw = self.odom2Coords(msg)


    def poseCallback(self, msg):
        """
        This method is activated whenever a message in the /target_pose topic 
        is published. For the assignments that require a target pose, you can 
        call your assignment functions from here or from the __main__ section
        at the end.

        Hint: in your terminal you can use the following command to send a pose
        of x,y,theta coordinates of 1, 2 and 3 respectively.
        $ rostopic pub /target_pose geometry_msgs/Pose2D '1.0' '2.0' '3.0'
        """
        # START: YOUR CODE HERE
        self.target_x   = msg.x
        self.target_y   = msg.y
        self.target_yaw = msg.theta

        # END: YOUR CODE HERE


    def timerCallback(self,event):
        """
        This timer function will save the odometry and Ground Truth values
        for the position in the self.odom_lst and self.gt_lst so you can
        compare them afterwards. It is updated every 1 second for 300 poses 
        (about 5 mins).
        """
        if self.odom_x is not None and self.gt_x is not None and len(self.odom_lst)<self.poses_to_save:
            self.odom_lst.append( (self.odom_x, self.odom_y) )
            self.gt_lst.append( (self.gt_x, self.gt_y) )
            

    """
    Now let's define some auxiliary methods. These are used to solve the 
    problems in the assignments.
    """
    def odom2Coords(self, odom_msg):
        """
        This is an auxiliary method used to get the (x, y, yaw) coordinates from 
        an Odometry message. You can use the cmd "$ rostopic echo -n 1 /odom"
        in the terminal to check out the Odometry message attributes.

        Hint: Check the built-in functions in the tf.transformations package to
        see if there is one that handles the euler-quaternion transformation.
        http://docs.ros.org/en/melodic/api/tf/html/python/transformations.html
        """
        
        # START: YOUR CODE HERE
        posicion = odom_msg.pose.pose.position
        x   = posicion.x
        y   = posicion.y
        
        orientacion = odom_msg.pose.pose.orientation
        quaternion = np.array([orientacion.x, orientacion.y, orientacion.z, orientacion.w])
             
        yaw = tf.transformations.euler_from_quaternion(quaternion, axes='sxyz')[2]

        # END: YOUR CODE HERE
        return (x, y, yaw)


    def saveLaser(self): 
        """
        For the RANSAC experience, it is very common for students to save the
        Laser array and work form home with Jupyter Notebook or Google Colab.
        """
        # Wait for laser to arrive
        while self.currentScan is None:
            pass
        
        ranges = np.array(self.currentScan.ranges)
        a_min  = self.currentScan.angle_min
        a_inc  = self.currentScan.angle_increment
        angles = np.array([a_min + i*a_inc for i in range(len(ranges))])

        array_to_save = np.stack([ranges, angles], axis=1)
        with open(self.path+"/results/laser_ranges.npy", "wb") as f:
            np.save(f, array_to_save)
            print("Saved array of shape (%i, %i)"%array_to_save.shape)
            print("Look it up in the %s/results directory"%self.path)


    def plotOdomVsGroundTruth(self):
        """
        Imports a map image and plots the trajectory of the robot according to 
        the Odometry frame and Gazebo's Ground Truth.
        """
        if len(self.odom_lst)>0:
            img = plt.imread(self.path+'/maps/map.pgm')
            print('Image imported')
            # Import map YAML (This is a text file with information about the map)
            with open(self.path+"/maps/map.yaml", 'r') as stream:
                data       = yaml.safe_load(stream)
                origin     = data['origin']
                resolution = data['resolution']
                height     = img.shape[0]
            
            odom_arr = np.array(self.odom_lst)
            gt_arr   = np.array(self.gt_lst)
            
            odom_x_px = ((odom_arr[:,0] - origin[0])/resolution).astype(int)
            odom_y_px = (height-1+ (origin[1]-odom_arr[:,1])/resolution).astype(int)
            gt_x_px = ((gt_arr[:,0] - origin[0])/resolution).astype(int)
            gt_y_px = (height-1+ (origin[1]-gt_arr[:,1])/resolution).astype(int)

            # plt.imshow(img,cmap='gray')
            plt.plot(odom_x_px, odom_y_px, color="red", linewidth=1, label='Odometry')
            plt.plot(gt_x_px, gt_y_px, color="blue", linewidth=1, label='Ground Truth')
            plt.legend()
            plt.grid()
            plt.title('Trajectory of the Robot')
            plt.axis('off')
            plt.savefig(self.path+'/results/trajectories.png')


    def printOdomvsGroundTruth(self):
        """
        Prints the robot odometry and ground truth position/angle.
        """
        if self.odom_x is not None and self.gt_x is not None:
            print("                  Odometry         -        GroundTruth")
            print("(x,y,yaw):  (%6.2f,%6.2f,%6.2f) - (%6.2f,%6.2f,%6.2f)"%(self.odom_x,self.odom_y,self.odom_yaw,self.gt_x,self.gt_y,self.gt_yaw))
    
    def get_laser(self): 
        """
        For the RANSAC experience, it is very common for students to save the
        Laser array and work form home with Jupyter Notebook or Google Colab.
        """
        # Wait for laser to arrive
        while self.currentScan is None:
            pass
        
        ranges = np.array(self.currentScan.ranges)
        a_min  = self.currentScan.angle_min
        a_inc  = self.currentScan.angle_increment
        angles = np.array([a_min + i*a_inc for i in range(len(ranges))])

        array_to_save = np.stack([ranges, angles], axis=1)
        
        return array_to_save
    
    def detect_famous(self, img):
        lower = np.array([100, 0, 0])
        upper = np.array([255, 0, 0])

        mask = cv2.inRange(img, lower, upper)
        
        mask_bin = mask
        mask_bin[mask_bin > 122] = 1

        column_histogram = mask_bin.sum(axis=0)
        row_histogram = mask_bin.sum(axis=1)

        column_sum = column_histogram.sum()  
        row_sum = row_histogram.sum()

        x_famous = 0
        y_famous = 0

        for x in range(column_histogram.shape[0]):
            x_famous += x*column_histogram[x]

        for y in range(row_histogram.shape[0]):
            y_famous += y*row_histogram[y]
        
        x_famous = int(x_famous/column_sum) if column_sum > 0 else 0
        y_famous = int(y_famous/row_sum) if row_sum > 0 else 0

        mask_bin[y_famous, x_famous] = 10

        x_center = img.shape[1]/2

        rotate_angle = (x_famous - x_center) * self.camera_fov / img.shape[1]

        print(rotate_angle)

        return cv2.circle(mask_bin, (x_famous, y_famous), 10, (10, 0, 0)), rotate_angle

    def rotate_to(self, angle):
        
        vel_angular = -0.2
        twist_msg = Twist()
        twist_msg.linear.x  = 0.0
        twist_msg.angular.z = vel_angular * np.sign(angle)
        self.vel_pub.publish(twist_msg)
        time.sleep(np.abs(angle/vel_angular))

        twist_msg = Twist()
        twist_msg.linear.x  = 0.0
        twist_msg.angular.z = 0.0
        self.vel_pub.publish(twist_msg)

    def angle_follow(self, iterations=60):
        for _ in range(iterations):
            image = self.currentImage

            detection, rotate_angle = self.detect_famous(image)
            self.rotate_to(rotate_angle)
            time.sleep(0.5)


    def show_video(self, frames=60):
        for _ in range(frames):
            from matplotlib.colors import hsv_to_rgb


            image = self.currentImage
            image_rgb = hsv_to_rgb(image)
            #print(image_rgb.shape)
            #print(image)

            #cv2.imshow('a', image)
            detection, rotate_angle = self.detect_famous(image)
            print(rotate_angle)
            

            plt.imshow(detection)
            #self.rotate_to(rotate_angle)
            #plt.draw()
            plt.colorbar()
            plt.pause(1)
            plt.close()


    def cam_test(self, counts = 1):
        for _ in range(counts):
            image = self.currentImage
            #print(self.currentImage)
            #print(type(self.currentImage),"\n")
            print(image.shape)
            
            #depth = np.copy(image_np[:,:,3])

            depth = self.currentDepth
            #depth[depth>100] = 0
            print(depth.shape, depth.min(), depth.max())
            
            plt.imshow(depth[:,:,2])
            plt.colorbar()
            plt.show()
            time.sleep(1)

if __name__ == '__main__':
    node = EL5206_Robot()
    print("EL5206 Node started!")
    time.sleep(2)

    try:
        #node.angle_follow()
        node.show_video(10)
        #node.cam_test()

    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")