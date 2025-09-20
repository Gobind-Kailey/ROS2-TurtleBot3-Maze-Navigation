import rclpy 
# Initialize Nav2 simple command API 
from nav2_simple_commander.robot_navigator import BasicNavigator

from geometry_msgs.msg import PoseStamped
import tf_transformations

def main():
    # Init ros2 communication
    rclpy.init()
    pass
    nav = BasicNavigator()


    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)

    # Set initial pose 
    initial_pose = PoseStamped()
    # We want to build a pose that is relative to the map frame 
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.position.z = 0.0

    initial_pose.pose.orientation.x = q_x
    initial_pose.pose.orientation.y = q_y
    initial_pose.pose.orientation.z = q_z
    initial_pose.pose.orientation.w = q_w
    nav.setInitialPose(initial_pose)
    

    # Wait for Nav2
    nav.waitUntilNav2Active()


    # Shutdown 


    rclpy.shutdown()


# Only run the file if it is directly executed, not imported by another file.  
if __name__ == "__main__":
    main()