#!/usr/bin/env python3
"""
This is the main script used for the EL5206 Robotics component. This script is
written Python 3 and defines a EL5206_Robot node for you to program. You will
find notes on how the code works, auxiliary functions and some for you to 
program.

Authors: Your dear 2022.Spring.teaching_staff
(Eduardo Jorquera, Ignacio Dassori & Felipe Vergara)
"""
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

        # Extra variable to print odometry
        self.odom_i = 0

        # Subscribers
        rospy.Subscriber("/odom",               Odometry,  self.odometryCallback)
        rospy.Subscriber("/ground_truth/state", Odometry,  self.groundTruthCallback)
        rospy.Subscriber("/scan",               LaserScan, self.scanCallback)
        rospy.Subscriber("/target_pose",        Pose2D,    self.poseCallback)
        rospy.Subscriber("/stalker/image_raw",  Image,     self.imageCallback)

        # Publishers
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

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

    def imageCallback(self,msg):
        self.currentImage = msg


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


    def dance(self, timeout=30):
        """
        Demo function. Moves the robot with the vel_pub Publisher.
        """
        start_time = time.time()
        while time.time() - start_time < timeout and not rospy.is_shutdown():          
            #Comparamos los datos de odoemtría
            
            # Move forward
            twist_msg = Twist()
            twist_msg.linear.x  = 0.2
            twist_msg.angular.z = 0.0
            self.vel_pub.publish(twist_msg)
            time.sleep(1)

            # Move backward
            twist_msg = Twist()
            twist_msg.linear.x  = -0.2
            twist_msg.angular.z = 0.0
            self.vel_pub.publish(twist_msg)
            time.sleep(1)

            # Turn left
            twist_msg = Twist()
            twist_msg.linear.x  = 0.0
            twist_msg.angular.z = 0.2
            self.vel_pub.publish(twist_msg)
            time.sleep(1)

            # Turn right
            twist_msg = Twist()
            twist_msg.linear.x  = 0.0
            twist_msg.angular.z = -0.2
            self.vel_pub.publish(twist_msg)
            time.sleep(1)


    def assignment_1(self, timeout=300):
        # You can use this method to solve the Assignment 1.
        # START: YOUR CODE HERE

        start_time = time.time()
        while time.time() - start_time < timeout and not rospy.is_shutdown():
            self.printOdomvsGroundTruth()
            # odom2Coords(self, odom_msg)
            time.sleep(4)

        
        # END: YOUR CODE HERE
        pass


    def assignment_2(self, path, timeout=300):
        # You can use this method to solve the Assignment 2.
        # START: YOUR CODE HERE
        start_time = time.time()
        # Definimos mensaje para detener al robot
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0

        print('Waiting for a target pose')
        while time.time() - start_time < timeout and not rospy.is_shutdown():
            time.sleep(0.2)
            if self.target_x is not None:
                # Una vez que haya un target aseguramos que el robot esta detenido
                self.vel_pub.publish(stop_msg)
                print('Target pose received!')
                time.sleep(0.2)
                break            
        
        if self.target_x is None:
            print('Timeout exceeded')
            return -1
        
        # Obtenemos posiciones iniciales y finales
        x0 = self.odom_x
        y0 = self.odom_y
        theta0 = self.odom_yaw
        
        xf = self.target_x
        yf = self.target_y
        thetaf = self.target_yaw
        
        # Camino girando y luego recto por separado
        if path == 'A':
            step = 0.2
            # delta angulo para apuntar al punto final
            delta_theta = np.arctan2(yf-y0, xf-x0) - theta0
            
            # rotamos
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = step*np.sign(delta_theta)
            self.vel_pub.publish(twist_msg)
            time.sleep(np.abs(delta_theta)/step)
            
            # detenemos
            self.vel_pub.publish(stop_msg)
            
            # distancia al punto
            delta_dist = np.linalg.norm(np.array([yf-y0, xf-x0]))
            
            # se mueve al punto
            twist_msg = Twist()
            twist_msg.linear.x = step
            twist_msg.angular.z = 0.0
            self.vel_pub.publish(twist_msg)
            time.sleep(delta_dist/step)
            
            # detenemos
            self.vel_pub.publish(stop_msg)
            
            # giramos nuevamente para apuntar hacia thetaf
            delta_theta = thetaf - self.odom_yaw
            
            # rotamos para apuntar hacia thetaf
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = step*np.sign(delta_theta)
            self.vel_pub.publish(twist_msg)
            time.sleep(np.abs(delta_theta)/step)
            
            # detenemos
            self.vel_pub.publish(stop_msg)
            
        # Camino girando y avanzando simultaneamente
        elif path == 'B':
            delta_theta = np.arctan2(yf-y0, xf-x0) - theta0
            delta_dist = np.linalg.norm(np.array([yf-y0, xf-x0]))
            umbral_theta = 0.01
            umbral_dist = 0.05
            while delta_dist >= umbral_dist:
            
                # movemos un poco el robot
                twist_msg = Twist()
                twist_msg.linear.x = np.min((0.075 * delta_dist, 0.3))
                twist_msg.angular.z = 0.1 * delta_theta * np.sign(delta_theta)
                self.vel_pub.publish(twist_msg)
                time.sleep(0.2)
                
                x0 = self.odom_x
                y0 = self.odom_y
                theta0 = self.odom_yaw
                
                delta_theta = np.arctan2(yf-y0, xf-x0) - theta0
                #if yf >=y0:
                #    delta_theta = np.pi-delta_theta
                delta_dist = np.linalg.norm(np.array([yf-y0, xf-x0]))
            
            # detenemos una vez muy cerca del punto final
            self.vel_pub.publish(stop_msg)
            
            # giramos nuevamente para apuntar hacia thetaf
            delta_theta = thetaf - self.odom_yaw
            
            # rotamos para apuntar hacia thetaf
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.2*np.sign(delta_theta)
            self.vel_pub.publish(twist_msg)
            time.sleep(np.abs(delta_theta)/0.2)
            
            # detenemos
            self.vel_pub.publish(stop_msg)
            
        else:
            print('Path is not defined')
            return -1
        
        # END: YOUR CODE HERE
        pass


    def assignment_3(self):
        # You can use this method to solve the Assignment 3.
        # START: YOUR CODE HERE

        # END: YOUR CODE HERE
        pass


    
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
    
    def assignment_4(self, K_a, K_r, tresh_dist = 0.01, R_max = 0.1, v_max = 0.25,t_s = 0.2, timeout = 90):
        # You can use this method to solve the Assignment 4.
        # START: YOUR CODE HERE
        
        start_time = time.time()
        # Definimos mensaje para detener al robot
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.vel_pub.publish(stop_msg)
        
        print('Waiting for a target pose')
        while time.time() - start_time < timeout and not rospy.is_shutdown():
            time.sleep(0.2)
            if self.target_x is not None:
                # Una vez que haya un target aseguramos que el robot esta detenido
                self.vel_pub.publish(stop_msg)
                print('Target pose received!')
                time.sleep(0.2)
                
                xf = self.target_x
                yf = self.target_y
                thetaf = self.target_yaw
                
                if tresh_dist >= np.linalg.norm(np.array([yf-self.odom_y, xf-self.odom_x])):
                    print('A PORQUE ESTOY AQCA')
                    break
            else:
                continue        
            
            
        
            
            
            while tresh_dist < np.linalg.norm(np.array([yf-self.odom_y, xf-self.odom_x])) and time.time() - start_time < timeout:
                
                print(f'target \t {xf:.04f} {yf:.04f}')
                print(f'odom \t {self.odom_x:.04f} {self.odom_y:.04f} {self.odom_yaw:.04f}')    
                print(f"gt \t {self.gt_x:.04f} {self.gt_y:.04f} {self.gt_yaw:.04f}")

                    
                # calculamos los potenciales y fzas:
                distancia =  np.linalg.norm(np.array([yf-self.odom_y, xf-self.odom_x]))
                
                if distancia < R_max:
                    #f_att_x = K_a*distancia*np.cos(self.odom_yaw)
                    #f_att_y = K_a*distancia*np.sin(self.odom_yaw)
                    f_att_x = K_a * (xf - self.odom_x)
                    f_att_y = K_a * (yf - self.odom_y)
                else:
                    #f_att_x = K_a*R_max*np.cos(self.odom_yaw)
                    #f_att_y = K_a*R_max*np.sin(self.odom_yaw)
                    f_att_x = K_a * R_max / distancia * (xf - self.odom_x)
                    f_att_y = K_a * R_max / distancia * (yf - self.odom_y)
                lectura_laser = self.get_laser()
                
                F = 0
                S = 0
                n = 0
                for d, theta in lectura_laser:
                    if not np.isinf(d):
                        F += np.cos(theta)/(d*d)
                        S += np.sin(theta)/(d*d) 
                        n += 1
                
                f_rep_x = K_r*F*np.cos(self.odom_yaw) - K_r*S*np.sin(self.odom_yaw)
                f_rep_y = K_r*F*np.sin(self.odom_yaw) + K_r*S*np.cos(self.odom_yaw)
                
                f_rep_x = f_rep_x/n if n > 0 else f_rep_x
                f_rep_y = f_rep_y/n if n > 0 else f_rep_y
                
                P_x = f_att_x - f_rep_x
                P_y = f_att_y - f_rep_y
                
                P = np.linalg.norm(np.array([P_x, P_y]))
                c_P = np.arctan2(P_y, P_x)-self.odom_yaw
                
                print("Variables de interés")
                print(f"Potenciales: \t{P_x:.04f} \t{P_y:.04f} \t{P:.04f}  \t{c_P:.04f} ")
                print(f"Velocidades: \t {0.1*c_P / t_s:.04f} \t{0.1*P / t_s:.04f}")
                print(f"Puntos: \t{n}\n")
                 
                
                # Movemos el roboc
                
                twist_msg = Twist()
                twist_msg.linear.x = 0
                twist_msg.angular.z = 0.1*c_P / t_s
                self.vel_pub.publish(twist_msg)
                time.sleep(t_s)
                
                twist_msg = Twist()
                twist_msg.linear.x = 0.1*P / t_s
                twist_msg.angular.z = 0
                self.vel_pub.publish(twist_msg)
                time.sleep(t_s)
                
                self.vel_pub.publish(stop_msg)
                time.sleep(0.2)
                
                       

                            
            self.vel_pub.publish(stop_msg)
            
                
        # END: YOUR CODE HERE
        pass

#    def prueba_giro()

    def cam_test(self,counts = 1):
        for _ in range(counts):
            print(self.currentImage)
            print(type(self.currentImage),"\n")
            time.sleep(1)



if __name__ == '__main__':
    node = EL5206_Robot()
    print("EL5206 Node started!")
    assignment = 5
    time.sleep(2)

    try:
        # Demo function
        # node.dance()
        if assignment == 1:
            print('Running assignment_1...')
            node.assignment_1(60)
            print('assignment_1 done!')
            print('Printing')
            node.plotOdomVsGroundTruth()

        elif assignment == 2:
            print('Running assignment_2')
            # Cambiar path entre 'A' y 'B' cambia el tipo de camino
            node.assignment_2(path='A')
            print('assignment_2 done!')
            print('Printing')
            node.printOdomvsGroundTruth()
            node.plotOdomVsGroundTruth()
            
        elif assignment == 4:
            print('Running assignment_4')
            node.assignment_4(5, 0.5)
            print('assignment_4 done! :)')
            node.plotOdomVsGroundTruth()
        
        else:
            node.cam_test()

    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")
