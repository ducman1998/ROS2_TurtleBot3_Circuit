import rclpy
import time 
import cv2 
import math 
import numpy as np 
import traceback
import tf2_geometry_msgs
from math import *
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import LaserScan, Image 
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from numpy.linalg import inv
from interfaces.msg import RobotPositions 
from tf2_ros import Buffer, TransformListener

# define captured image dimensions: height and width
IM_W = 320
IM_H = 240
# define reference colors for color-based segmentation
LANE_COLOR = (224,224,224) # color for lane parts
YELLOW_LINE_COLOR = (255, 255, 11) # color for outer lines
WHITE_LINE_COLOR = (255, 255, 255) # color for inner lines

# define min & max linear velocity (x direction only)
MIN_VX = 0.5
# this variable will be used in the future, not current version, so we set its value to MIN_VX to keep a constant Vx 
MAX_VX = MIN_VX
# gain constants for PID controller (to control angular velocity)
KP = 5.0
KI = 0.001
KD = 0.25 

# parameters for laser scan
MAX_LIDAR_DISTANCE = 3.5  # in meters
COLLISION_DISTANCE = 0.4 # in meters
# only stop if detect obstacles in range 0.25m ahead of the robot, ranging: [-60,60]
ANGLE_MAX = 30  # degree
ANGLE_MIN = -30 # degree 

last_vel = 0.0 
last_omegaz = 0.0

class Commander(Node):
    def __init__(self):
        super().__init__('turtlebot3_commander')
        # subcribe this topic to read camera images in simulation
        self.scan_sub = self.create_subscription(Image, '/Pi_Camera/image_raw', self.cam_callback, 10)
        
        # subcribe this topic to read laser scan
        self.scan_sub = self.create_subscription(LaserScan, "scan", self.scan_callback, 10)
        
        # publish command velocities to control the turtlebot (a Twist includes both linear and angular velocities)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # publish robot position (real from gazebo, current position with respect to (wrt.) odom frame, 
        # and measured point position (center of mass of a segmented lane) wrt. odom frame)
        self.robot_visualizer = self.create_publisher(RobotPositions, '/robot_positions', 10)
        
        # subcribe to this topic to receive real robot position from Gazebo engine
        self.gazebo_state_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.gazebo_state_callback,
            10)
        # subcribe this topic to receive an estimated robot position wrt. odom frame
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        # two variables below are used to determine transformation (T) between robot base_footprint frame and odom frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # set a timer 20hz using as sampling time to control the robot
        self.timer = self.create_timer(1/20, self.control_loop)
        
        self.br = CvBridge()
        self.last_frame = None 
        self.timestamp = None 
        
        ## setup transformation matrices for calibration & rectification
        # intrinsic parameters of the real camera
        self.K = np.array([[93.31658, 0, 160.5], [0, 93.31658, 120.5], [0, 0, 1]])
        # T: base frame (b) wrt. base_footprint frame (f)
        self.Tfb = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.010], [0, 0, 0, 1]])
        # T: real camera frame (c1) wrt. base frame (b)
        self.Tbc = np.array([[0, 0, 1, 0.04], [-1, 0, 0, 0], [0, -1, 0, 0.125], [0, 0, 0, 1]])
        # put a world frame (w) 1.0 meter ahead in X-direction compared to base_footprint frame of the turtlebot3
        # T: world frame wrt. base_footprint frame
        self.Tfw = np.array([[1, 0, 0, 1.0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

        # specially for rectification
        tprime = 0.15 # in meter, used to determine scaling factor (pixel to m) later on
        # T: world frame wrt. fronto-parallel virtual camera frame (c2)
        self.Tc2w = np.array([[0, -1, 0, -0.05], [-1, 0, 0, -0.665], [0, 0, -1, tprime], [0, 0, 0, 1]])
        # H: world frame wrt. real camera frame (c1), mapping from 3D point to pixel positions
        # this is a homography matrix, not a simple homogeneous transformation 
        self.Hc1w = self.K @ (inv(self.Tbc) @ inv(self.Tfb) @ self.Tfw)[:3,[0,1,3]]
        # H: world frame  wrt. image c2 frame
        self.Hc2w = self.K @ self.Tc2w[:3,[0,1,3]]
        # H: real camera frame (c1) wrt. virtual camera frame (c2)
        self.Hc2c1 = self.Hc2w @ inv(self.Hc1w)
        # convert pixel to m in virtual camera (c2)
        self.scaling_factor = tprime/self.K[0,0]
        print(f"scaling factor (pix to m): {self.scaling_factor}")
        
        # variables for PID control
        self.total_err = 0
        self.last_steering_angle = 0
        self.Kp = KP
        self.Ki = KI
        self.Kd = KD 
        
        # Create a Twist message to send to /cmd_vel topic
        self.twist = Twist()
        
        # a variable stores minimum laser distance
        self.min_dis_to_obstacles = 100000
        
        # variables for visulization (real robot position, odom position, 
        # planned position - center of mass of lane transformed into odom frame), xy only
        self.positions = RobotPositions()
        self.real_pos = [0, 0]
        self.odom_pos = [0, 0]
        self.planned_pos = [0, 0]
        
        # these arrays to store historical velocities used to adapt robot 
        # velocity with lane characteristics such as curve or straight
        self.his_velocities = []
        self.his_straight_scores = []
        
    def update_vel(self, vel, num_keep=60):
        '''
        This function takes a velocity (vx) as input, using moving average to behave like a lowpass filter, avoiding
        high fluctuation in commanding vx 
        '''
        self.his_velocities.append(vel if vel is not None and vel > 0 else 0)
        if len(self.his_velocities) > num_keep:
            self.his_velocities = self.his_velocities[-num_keep:]
        return np.mean(self.his_velocities)
    
    def update_straight_score(self, score, num_keep=10):
        '''
        This function takes straight score of robot (range [0,1], measured based on a segmented lane, close to 1.0 in 
        straight parts and smaller in curve parts). Behaving like a lowpass filter using moving average technique
        '''
        self.his_straight_scores.append(score if score is not None and score > 0 else 0)
        if len(self.his_straight_scores) > num_keep:
            self.his_straight_scores = self.his_straight_scores[-num_keep:]
        return np.mean(self.his_straight_scores)
        
    def get_tranform_odom_to_footprint(self):
        '''
        This function is used to get transformation matrix of the base_footprint frame wrt. odom frame
        '''
        try:
            # Wait for the transform from 'footprint' to 'odom'
            transform = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            return transform
            
        except Exception as e:
            self.get_logger().error(f"Error: {str(e)}")
            return 
        
    def control_loop(self):
        '''
        Use a timer 20hz serving as sampling time for controller, this function will:
        (1) rectify image 
        (2) image processing on the rectified image
        (3) predict straight score of the lane sensed by the carmera
        (4) segment lane to find center of mass point (CoM)
        (5) measure error and generate pid control signal (control angular velocity)
        (6) send velocity command to /cmd_vel topic
        '''
        global last_vel, last_omegaz
        
        if self.min_dis_to_obstacles <= COLLISION_DISTANCE:
            self.move(0.0, 0.0)
            # stop the robot immediately
            exit()
        
        if self.last_frame is None:
            # ignore this timestep due to no new captured image
            self.get_logger().warn(f"Waiting for new frame comming... Last received frame timestamp: {self.timestamp}")
            return 
        
        # rectify the captured image
        rec_im_rgb = cv2.warpPerspective(self.last_frame, self.Hc2c1, (IM_W, IM_H))
        # predict straight score --> use to adapt the velocity
        straight_score = self.predict_straight_score(rec_im_rgb) # [0,1]
        
        # crop the near part to ingore farther lane parts
        T = 100
        processed_im = rec_im_rgb[T:,:,:]
        # lane segmentation based on distance from rgb pixels to a predefined reference color
        segmented_rgb = self.segment_lane(processed_im, LANE_COLOR, tol=5)
        # convert segmented lane to binary and remove noises, forming into a largest connected component
        post_processed_im = self.post_process(segmented_rgb)
        if post_processed_im is None:
            return 
        
        # find center of mass (CoM) of the segmented lane
        moments = cv2.moments(post_processed_im)
        # Calculate the centroid (cx, cy)
        if moments['m00'] != 0:
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])
        else:
            cx, cy = 0, 0  # Avoid division by zero
            return 
        
        # project CoM in image frame to world frame 
        Pw = inv(self.Hc2w) @ np.array([cx, cy+T, 1])
        Pw = Pw/Pw[2]
        # project CoM in world frame to base_footprint frame
        Pf = self.Tfw @ np.array([Pw[0], Pw[1], 0, 1])
        Pf = (Pf/Pf[3])[:2]
        print(f"Location of landmark wrt. the footprint frame: {Pf}")
        
        # measurement error in steering angle (rad) is implemented in base_footprint frame 
        # for more stable control signals
        steering_angle = math.atan2(-Pf[1], Pf[0])
        err = 0 - steering_angle 
        derr = 0 - (steering_angle - self.last_steering_angle)
        self.total_err += err 
        self.last_steering_angle = steering_angle 
        
        # determine velocity (vx) based on measured straight score of the lane
        # if straight score >= 0.7 --> increase by a linear function
        # this step is not optimized yet, so that we put a small maximum value
        vx = MIN_VX
        straight_score = self.update_straight_score(straight_score)
        if straight_score >= 0.7:
            vx = MIN_VX + (MAX_VX - MIN_VX) * (straight_score - 0.7)/(1.0-0.7)
            
        # lowpass filter the commanding velocity
        moving_avg_vx = self.update_vel(vx)
        # PID controller 
        omega_command = self.Kp* err + self.Ki * self.total_err + self.Kd * derr 
        # send command to robot
        self.move(moving_avg_vx, omegaz=omega_command)
        
        # store for next iteration
        last_vel = moving_avg_vx 
        last_omegaz = omega_command
        print(f"Steering angle: {steering_angle} (rad), {steering_angle/math.pi*180} (deg) | straight score: {straight_score}, Vx: {moving_avg_vx} m/s")
        
        # publish positions to /robot_positions topic for visualization
        odom2footprint = self.get_tranform_odom_to_footprint()
        if odom2footprint is not None:
            # transform CoM from base_footprint frame to odom frame for visualization on Rviz2 
            point_stamped = PointStamped()
            point_stamped.header.frame_id = 'base_footprint'  # The original frame of the point
            point_stamped.header.stamp = rclpy.time.Time().to_msg()
            planned_point = Point(x=Pf[0], y=Pf[1], z=0.0)
            point_stamped.point = planned_point
            point_in_odom = tf2_geometry_msgs.do_transform_point(point_stamped, odom2footprint)
            self.planned_pos[0] = point_in_odom.point.x
            self.planned_pos[1] = point_in_odom.point.y
        
        # publish real position (gazebo engine), current position (odom frame) and CoM position (odom frame)
        self.positions.real_pos_x = float(self.real_pos[0])
        self.positions.real_pos_y = float(self.real_pos[1])
        self.positions.odom_pos_x = float(self.odom_pos[0])
        self.positions.odom_pos_y = float(self.odom_pos[1])
        self.positions.planned_pos_x = float(self.planned_pos[0])
        self.positions.planned_pos_y = float(self.planned_pos[1])
        self.robot_visualizer.publish(self.positions)
        
    def move(self, vx: float, omegaz: float):
        # Set linear velocity in the x direction, angular velocity comes from a PID controller
        self.twist.linear.x = vx 
        self.twist.angular.z = omegaz
        self.cmd_vel_publisher.publish(self.twist)
        self.get_logger().info(f"Sending a command: vx={vx} m/sec, ang.z={omegaz} rad/sec")
        
    def cam_callback(self, msg):
        # this function processes image from camera, it stores image in a variable for being used in control loop
        current_frame = self.br.imgmsg_to_cv2(msg)
        current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        if current_frame.shape[0] != IM_H or current_frame.shape[1] != IM_W:
            raise Exception(f"Received image shapes are different with predefined shapes: {current_frame.shape} vs {(IM_H, IM_W)}")
        self.last_frame = current_frame
        self.timestamp = time.time()
        
    def scan_callback(self, msg):
        # self.get_logger().info(f"Get msg from lazerScan: {len(msg.ranges)}")
        (lidar, angles) = lidarScan(msg)
        lidar_horizon = np.concatenate((lidar[360+ANGLE_MIN:], lidar[0:ANGLE_MAX]))
        # angles_horizon = np.linspace(90+ANGLE_MIN, 90+ANGLE_MAX, ANGLE_MAX - ANGLE_MIN) 
        self.min_dis_to_obstacles = np.min(lidar_horizon)
        
    def gazebo_state_callback(self, msg: ModelStates):
        '''
        This callback will receive real position from the Gazebo simulation engine, using for visualization to 
        evaluate robot performance only
        '''
        try:
            # Access specific model information, e.g., burger
            model_name = 'burger' 
            if model_name in msg.name:
                index = msg.name.index(model_name)
                position = msg.pose[index].position
                self.real_pos = [position.x, position.y]
                self.get_logger().info(
                    f'Position: x={position.x}, y={position.y}, z={position.z}')
        except Exception as e:
            self.get_logger().error(f'Error processing message: {e}')
            
    def odom_callback(self, msg: Odometry):
        '''
        This callback will receive an estimated position of the robot wrt. odom frame
        '''
        position = msg.pose.pose.position
        self.odom_pos = [position.x, position.y]
        
    def segment_lane(self, in_rgb, ref_color, tol=20):
        # return segmented lane in a grayscale image
        dis = np.linalg.norm(in_rgb - np.array(ref_color), axis=2)
        mask = dis < tol
        return np.uint8(mask*255)
    
    def predict_straight_score(self, rectified_im_rgb):
        '''
        This function predicts a traight score for an input image by segmenting lane regions like 
        in determing CoM step but using a bigger and taller image. The straight score will be measured
        as ratio of lane area per image size 
        '''
        processed_im = rectified_im_rgb[50:,100:220,:]
        segmented_rgb = self.segment_lane(processed_im, LANE_COLOR, tol=10)
        post_process_im = self.post_process(segmented_rgb)
        if post_process_im is None:
            return -1.0
        num_nonzero = np.count_nonzero(post_process_im)
        return num_nonzero/post_process_im.size

    def post_process(self, segmented_lane_bi):
        '''
        This function will remove noise in segmented lane, then extract the largest connected component only
        '''
        try:
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            closing_im = cv2.morphologyEx(segmented_lane_bi, cv2.MORPH_CLOSE, kernel)
            
            # Find all contours in the binary image
            contours, _ = cv2.findContours(closing_im, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # Find the largest contour by area
            largest_contour = max(contours, key=cv2.contourArea)
            # Create an empty mask the same size as the image
            mask = np.zeros_like(closing_im)
            # Draw the largest contour on the mask
            cv2.drawContours(mask, [largest_contour], -1, 255, thickness=cv2.FILLED)
            # Apply the mask to extract the largest connected component
            largest_component = cv2.bitwise_and(closing_im, mask)
        except KeyboardInterrupt:
            exit()
        except Exception:
            self.get_logger().error(f"Error: {traceback.format_exc()}")
            return 
        
        return largest_component
    
def lidarScan(msgScan):
    distances = np.array([])
    angles = np.array([])

    for i in range(len(msgScan.ranges)):
        angle = degrees(i * msgScan.angle_increment)
        if (msgScan.ranges[i] > MAX_LIDAR_DISTANCE):
            distance = MAX_LIDAR_DISTANCE
        elif (msgScan.ranges[i] < msgScan.range_min):
            distance = msgScan.range_min
            # # For real robot - protection
            # if msgScan.ranges[i] < 0.01:
            #     distance = MAX_LIDAR_DISTANCE
        else:
            distance = msgScan.ranges[i]

        distances = np.append(distances, distance)
        angles = np.append(angles, angle)

    # distances in [m], angles in [degrees]
    return ( distances, angles )

def stop_robot():
    rclpy.init()    
    node = rclpy.create_node('stopping_robot')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)
    twist = Twist()
    # reduce effect of robot inertia
    for i in range(20):
        twist.linear.x = -last_vel
        twist.angular.z = -last_omegaz
        pub.publish(twist)
        time.sleep(0.001)
    for i in range(2000):
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        time.sleep(0.001)

def main(args=None):
    rclpy.init(args=args)
    commander = Commander()
    try:
        rclpy.spin(commander)
    except:
        try:
            commander.destroy_node()
            rclpy.try_shutdown()
        except:
            pass 
        traceback.print_exc()
        commander.get_logger().info('KeyboardInterrupt detected!')
        print("Finished main program. Starting to stop the robot...")
        stop_robot()
        print("Stopping the robot is now finished.")
    
if __name__ == '__main__':
    main()
    