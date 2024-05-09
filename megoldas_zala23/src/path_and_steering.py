#!/usr/bin/env python3

# Import necessary packages
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from math import cos, sin, tan
from geometry_msgs.msg import Point, PoseStamped, Pose, Twist, Quaternion
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
# Define global variables
global speed_cmd, steering_angle, actual_pose
speed_cmd = 0.0
steering_angle = 0.0
actual_pose = Pose()
global path
path = Path()
max = 0.5
wheelbase = 0.33

# Function to map a value from one range to another
def mapval(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Callback function for receiving vehicle steering commands
def vehicleSteeringCallback(cmd_msg):
    global steering_angle, speed_cmd
    steering_angle = cmd_msg.angular.z
    speed_cmd = cmd_msg.linear.x

# Callback function for receiving vehicle pose information
def vehiclePoseCallback(pos_msg):
    global actual_pose
    actual_pose = pos_msg.pose.pose

# Main loop function
def loop():
    global path
    # print("speed_cmd: ", speed_cmd)
    # print("steering_angle: ", steering_angle)
    # Create and publish steering marker
    if publish_steer_marker:
        steer_marker = Marker()
        steer_marker.header.frame_id = "base_link"
        steer_marker.header.stamp = rospy.Time.now()
        steer_marker.ns = "steering_path"
        steer_marker.id = 0
        steer_marker.type = steer_marker.LINE_STRIP
        steer_marker.action = Marker.ADD
        steer_marker.pose.position.x = 0
        steer_marker.pose.position.y = 0
        steer_marker.pose.position.z = 0
        steer_marker.pose.orientation.x = 0.0
        steer_marker.pose.orientation.y = 0.0
        steer_marker.pose.orientation.z = 0.0
        steer_marker.pose.orientation.w = 1.0
        steer_marker.scale.x = 0.2

        # Set color of the marker based on the value of marker_color
        if marker_color == "r":
            steer_marker.color = ColorRGBA(0.96, 0.22, 0.06, 1.0)
        elif marker_color == "b":
            steer_marker.color = ColorRGBA(0.02, 0.50, 0.70, 1.0)
        elif marker_color == "y":
            steer_marker.color = ColorRGBA(0.94, 0.83, 0.07, 1.0)
        else:
            green_r = 127./255.
            green_g = 255./255.
            green_b = 187./255.
            red_r = 231./255.
            red_g = 54./255.
            red_b = 102./255.
            blue_r = 0./255.
            blue_g = 136./255.
            blue_b = 204./255.

            # Set color of the marker based on the speed_cmd value
            if speed_cmd < 0:
                steer_marker.color = ColorRGBA(green_r, green_g, green_b, 1.0)
                rospy.loginfo("Negative speed (going backwards) %s m/s", speed_cmd)
            elif speed_cmd > max:
                steer_marker.color = ColorRGBA(red_r, red_g, red_b, 1.0)
                rospy.loginfo("Higher than %s m/s (max) current ref: %s m/s", max, speed_cmd)
            elif 0 < speed_cmd and speed_cmd < max / 2:
                c0 = mapval(speed_cmd, 0, max / 2, green_r, blue_r)
                c1 = mapval(speed_cmd, 0, max / 2, green_g, blue_g)
                c2 = mapval(speed_cmd, 0, max / 2, green_b, blue_b)
                steer_marker.color = ColorRGBA(c0, c1, c2, 1.0)
            else:
                c0 = mapval(speed_cmd, max / 2, max, blue_r, red_r)
                c1 = mapval(speed_cmd, max / 2, max, blue_g, red_g)
                c2 = mapval(speed_cmd, max / 2, max, blue_b, red_b)
                steer_marker.color = ColorRGBA(c0, c1, c2, 1.0)

        steer_marker.lifetime = rospy.Duration()
        marker_pos_x = 0.0
        marker_pos_y = 0.0
        theta = 0.0

        # Calculate the position of the marker based on speed_cmd and steering_angle
        if speed_cmd < 0:
            for i in range(10):
                marker_pos_x += 0.01 * 10 * cos(theta)
                marker_pos_y += 0.01 * 10 * sin(theta)
                theta += 0.01 * 10 / wheelbase * tan(steering_angle)
                p = Point(marker_pos_x, marker_pos_y, 0)
                steer_marker.points.append(p)
            steer_marker.points.reverse()
            marker_pos_x = 0.0
            marker_pos_y = 0.0
            theta = 0.0
            for i in range(10 + int(-20 * speed_cmd)):
                marker_pos_x -= 0.01 * 10 * cos(theta)
                marker_pos_y -= 0.01 * 10 * sin(theta)
                theta -= 0.01 * 10 / wheelbase * tan(steering_angle)
                p = Point(marker_pos_x, marker_pos_y, 0)
                steer_marker.points.append(p)
        else:
            for i in range(10 + int(20 * speed_cmd)):
                marker_pos_x += 0.01 * 10 * cos(theta)
                marker_pos_y += 0.01 * 10 * sin(theta)
                theta += 0.01 * 10 / wheelbase * tan(steering_angle)
                p = Point(marker_pos_x, marker_pos_y, 0)
                steer_marker.points.append(p)

        # Publish the steering marker
        marker_pub.publish(steer_marker)
        steer_marker.points = []

    # Create and publish path marker
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.pose.position = actual_pose.position
    pose.header.frame_id = "odom_combined"
    path.header.frame_id = "odom_combined"
    pose.pose.orientation = actual_pose.orientation
    path.poses.append(pose)
    path.header.stamp = rospy.Time.now()
    path.poses.append(pose)

    # Limit the size of the path
    if len(path.poses) > path_size:
        del path.poses[:len(path.poses) - path_size]

    # Publish the path
    path_pub.publish(path)

    # Create and publish text marker
    text_marker = Marker()
    text_marker.header.frame_id = "base_link"
    text_marker.header.stamp = rospy.Time.now()
    text_marker.ns = "text_marker"
    text_marker.id = 0
    text_marker.type = text_marker.TEXT_VIEW_FACING
    text_marker.action = Marker.ADD
    text_marker.pose.position.x = 0.1
    text_marker.pose.position.y = -1.2
    text_marker.pose.position.z = 0.5
    text_marker.pose.orientation.x = 0.0
    text_marker.pose.orientation.y = 0.0
    text_marker.pose.orientation.z = 0.0
    text_marker.pose.orientation.w = 1.0

    # Set color of the text marker based on the speed_cmd value
    if speed_cmd < 0:
        text_marker.color = ColorRGBA(231./255., 54./255., 102./255., 1.0)
    else:
        text_marker.color = ColorRGBA(0.8, 0.9, 1.0, 1.0)

    text_marker.scale.z = 0.4
    text_marker.lifetime = rospy.Duration()
    text_marker.text = "%.2fm/s" % speed_cmd

    # Publish the text marker
    text_pub.publish(text_marker)

# Get parameters from ROS parameter server
pose_topic = rospy.get_param('~pose_topic', '/odom')
marker_topic = rospy.get_param('~marker_topic', '/marker_steering')
path_topic = rospy.get_param('~path_topic', '/marker_path')
marker_color = rospy.get_param('~marker_color', 'gradient')
publish_steer_marker = rospy.get_param('~publish_steer_marker', True)
path_size = rospy.get_param('~path_size', 100)

# Initialize ROS node
rospy.init_node('path_and_steering')

# Subscribe to necessary topics
sub_cmd = rospy.Subscriber('/cmd_vel', Twist, vehicleSteeringCallback)
sub_current_pose = rospy.Subscriber(pose_topic, Odometry, vehiclePoseCallback)

# Create publishers
marker_pub = rospy.Publisher(marker_topic, Marker, queue_size=1)
path_pub = rospy.Publisher(path_topic, Path, queue_size=1)
text_pub = rospy.Publisher('/marker_text', Marker, queue_size=1)

# Log node information
rospy.loginfo("Node started: %s subscribed: %s publishing: %s %s", rospy.get_name(), pose_topic, marker_topic, path_topic)

# Set the loop rate
rate = rospy.Rate(144) # 20Hz

# Main loop
while not rospy.is_shutdown():
    loop()
    rate.sleep()