import rclpy
import time 
import os 
from math import *
from rclpy.node import Node
from sensor_msgs.msg import Image 
from numpy.linalg import inv
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from interfaces.msg import RobotPositions 

# limit number of visualized points to 5000 points
MAX_MARKER_INDEX = 5000
LEGEND_ID = -100
BACKGROUND_ID = -1000

class RobotPathVisualizer(Node):
    def __init__(self):
        super().__init__('robot_path_visualizer')
        # subscribe the below topic to receive robot positions
        self.pos_sub = self.create_subscription(RobotPositions, '/robot_positions', self.listener_callback, 10)
        # publish visualized robot positions into a MarkerArray
        self.points_publisher = self.create_publisher(MarkerArray, 'points_topic', 10)
        
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # check background image filepath (only once)
        self.bg_fp = os.path.join(get_package_share_directory("turtlebot3_visualizer"), "materials", "track1.dae")
        self.idx = 0
        self.legend_idx = LEGEND_ID
        
    def control_loop(self):
        # do nothing to keep this package running
        pass 
        
    def create_marker(self, idx, point: Point, color="red", ignore_color=False, name_space="points_namespace"):
        # Create a Marker message to add points
        marker = Marker()
        marker.header.frame_id = "world"  # Make sure this matches your rviz fixed frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = name_space
        marker.id = idx 
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01  # Size of points
        marker.scale.y = 0.01  # Size of points
        marker.scale.z = 0.01
        marker.color.a = 1.0
        if not ignore_color:
            if color == "red":
                marker.color.r = 1.0  # Red points
            elif color == "green":
                marker.color.g = 1.0  # Green points
            else:
                marker.color.b = 1.0  # Blue points
        
        marker.points = [point]
        return marker
    
    def create_legend_markers(self, text="", pos=[0.0,0.0], point_color="red", name_space="points_namespace"):
        '''
        This function is used to create markers for legends.
        '''
        self.legend_idx += 1
        marker = self.create_marker(self.legend_idx, Point(), color=point_color)
        marker.ns = name_space
        marker.type = Marker.CUBE
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.pose.position.x = pos[0]
        marker.pose.position.y = pos[1]
        
        self.legend_idx += 1
        marker_label = self.create_marker(self.legend_idx, Point(), color=point_color)
        marker_label.ns = name_space
        marker_label.type = Marker.TEXT_VIEW_FACING
        marker_label.text = text 
        marker_label.scale.x = 0.2
        marker_label.scale.y = 0.2
        marker_label.scale.z = 0.2
        marker_label.pose.position.x = pos[0]
        marker_label.pose.position.y = pos[1] - 1.5
        return [marker, marker_label]
    
    def publish_background_marker(self):
        '''
        The circuit in PNG format is transformed to 3D '.dae' file for visualizing in Rviz2 directly
        '''
        marker = self.create_marker(BACKGROUND_ID, Point(), color="red", ignore_color=True)
        marker.ns = "background"
        marker.type = Marker.MESH_RESOURCE
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 50.0
        # Don't set a color to preserve the .dae file's material color
        marker.color.a = 1.0  # Keep the opacity
        marker.color.r = 1.0  # Avoid overriding the color by keeping it default
        marker.color.g = 1.0
        marker.color.b = 1.0
        # Set the image as texture (path to your image file)
        marker.mesh_resource = f"file://{self.bg_fp}"
        marker.points = []
        # Using sin to create a rotation around the Z-axis
        # marker.pose.orientation.z = math.pi/2
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.points_publisher.publish(marker_array)
        self.get_logger().info(f"Published background texture.")
        return 
    
    def listener_callback(self, msg):
        # Get robot positions from the message
        start_time = time.time()
        marker_array = MarkerArray()
        
        # reduce overhead in computating for rendering background image
        if self.idx % 300 == 0:
            self.publish_background_marker()
        
        marker_array.markers.extend(self.create_legend_markers("Real Positions (Got from Gazebo)", pos=[2.5,2.5], point_color="green", name_space="RealPosition"))
        marker_array.markers.extend(self.create_legend_markers("Estimated Positions (Odom)", pos=[2.3, 2.5], point_color="red", name_space="OdomPosition"))
        marker_array.markers.extend(self.create_legend_markers("Measured Positions (Odom)", pos=[2.1, 2.5], point_color="blue", name_space="MeasuredPosition"))
        self.legend_idx = LEGEND_ID
        
        # add real position
        self.idx += 1
        odom_marker = self.create_marker(self.idx, Point(x=msg.odom_pos_x, y=msg.odom_pos_y, z=0.15), color="red", name_space="RealPosition")
        # add robot position in the odom frame
        self.idx += 1
        r_marker = self.create_marker(self.idx, Point(x=msg.real_pos_x, y=msg.real_pos_y, z=0.15), color="green", name_space="OdomPosition")
        # add measured point (CoM) in the odom frame
        self.idx += 1
        planned_marker = self.create_marker(self.idx, Point(x=msg.planned_pos_x, y=msg.planned_pos_y, z=0.15), color="blue", name_space="MeasuredPosition")
        marker_array.markers.extend([odom_marker, r_marker, planned_marker])
        self.points_publisher.publish(marker_array)
        # reset index of points to limit within range of MAX_MARKER_INDEX
        self.idx = self.idx % MAX_MARKER_INDEX
        self.get_logger().warn(f"Total visualization time: {time.time() - start_time} (sec)")


def main(args=None):
    rclpy.init(args=args)
    visualizer = RobotPathVisualizer()
    rclpy.spin(visualizer)
    visualizer.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()
    