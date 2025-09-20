import rclpy 
# Initialize Nav2 simple command API 
from nav2_simple_commander.robot_navigator import BasicNavigator

from geometry_msgs.msg import PoseStamped
import tf_transformations


def create_pose_stamped(navigator, position_x, position_y, orentation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orentation_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0

    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w


    return pose



    # nav.followWaypoints



def main():
    # Init ros2 communication
    rclpy.init()
    pass
    nav = BasicNavigator()

    # set initial pose 
    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)
    nav.setInitialPose(initial_pose)


   
    

    # Wait for Nav2
    nav.waitUntilNav2Active()

    #Send nav2 goal

    # Send Nav2 goal 
    # 1.57 = Pi/2 -- so 90 degrees CCW
    goal_pose1 = create_pose_stamped(2.5, 1.0, 1.57)
    goal_pose2 = create_pose_stamped(2.5, 1.0, 1.57)
    goal_pose3 = create_pose_stamped(2.5, 1.0, 1.57)

    # if you want to go to only 1 waypoint
    # nav.goToPose(goal_pose1)
    # while not nav.isTaskComplete():
    #     feedback = nav.getFeedback()
    #     # print(feedback)

    # Follow waypoints
    waypoints = [goal_pose1, goal_pose2, goal_pose3]
    nav.followWaypoints(waypoints)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()


    # Seeing if we reached the navigation goal 
    print(nav.getResult())

    # Shutdown 
    rclpy.shutdown()


# Only run the file if it is directly executed, not imported by another file.  
if __name__ == "__main__":
    main()