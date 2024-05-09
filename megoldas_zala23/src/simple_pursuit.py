#!/usr/bin/env python3
'''
based on 2018 Varundev Suresh Babu (University of Virginia)
                MIT License
'''

from __future__ import print_function
import rospy, tf2_ros, tf2_geometry_msgs
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, PointStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
import std_msgs


ANGLE_RANGE = 360 # LSN10 LIDAR has 360 degrees scan
DESIRED_DISTANCE_RIGHT = 0.2 #0.9 # meters
DESIRED_DISTANCE_LEFT = 0.8 # 0.55
VELOCITY = 1.00 # meters per second
CAR_LENGTH = 0.445 # 0.445 meters
WHEELBASE = 0.3187; # documention based | measured: ~32 cm

## NODE INITIALIZATION
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
pubst1 = rospy.Publisher('pid_data', std_msgs.msg.String, queue_size=10)
marker_pub = rospy.Publisher("/debug_marker2", Marker, queue_size=1)
goalmarker_pub = rospy.Publisher("/algo2_marker", MarkerArray, queue_size=1) # new algorithm wall and follow markers  
bluepoints = rospy.Publisher("/blue",MarkerArray, queue_size=1) 
falmarker = Marker()
marker_arrayblue = MarkerArray()
trans = TransformStamped()
prev_steering_err = 0.0
prev_velocity = 0.0

# MARKER PROPERTIES
for nev in ["falmarker", "goalmarker", "marker_points"]:
    exec(nev + " = Marker()")
    exec(nev + ".header.frame_id = 'laser'") # "laser"
    exec(nev + ".type = Marker.SPHERE_LIST")
    exec(nev + ".action = " + nev + ".MODIFY")
    exec(nev + ".color.r = 0.0")
    exec(nev + ".color.g = 0.0")
    exec(nev + ".color.a = 1.0")
    exec(nev + ".color.b = 1.0")
    exec(nev + ".scale.x = 0.1")
    exec(nev + ".scale.y = 0.1")
    exec(nev + ".scale.z = 0.1")
    exec(nev + ".pose.orientation.x = 0.0")
    exec(nev + ".pose.orientation.y = 0.0")
    exec(nev + ".pose.orientation.z = 0.0")
    exec(nev + ".pose.orientation.w = 1.0")
    exec(nev + ".points = []")


# Methods in call order
def callbackLaser(data):

    # Does a simple follow
    error_steering, velocity = followSimple(data)


    msg_cmd = Twist()
    msg_cmd.linear.x = velocity * 0.5 # TODO: test low speed
    msg_cmd.angular.z = error_steering # angle
    pub.publish(msg_cmd)    

def followSimple(data: LaserScan):
    """
    data: single message from topic /scan
    desired_trajetory: desired distance to the right wall [meters]
    velocity: desired velocity [m/s]
    car_length: length of the car [meters]
    wheelbase: distance between the front and rear wheels [meters]
    returns: steering angle [radians]
    """
    desired_trajetory = DESIRED_DISTANCE_RIGHT

    global pubst1, marker_points, prev_steering_err, prev_velocity
    messageS1 = std_msgs.msg.String()
    messageS1.data = "Egyszeru_pursuit"
    angles = np.arange(data.angle_min, data.angle_max, data.angle_increment)
    if (len(angles) - len(data.ranges) != 0):
        rospy.logwarn("angles and ranges lenght differ")

    target_distance, target_angle, left_d, right_d = getDistance(data.ranges, angles)

    point = Point()
    point.x = target_distance
    point.y = target_angle
    point_st = PointStamped()
    point_st.point = point
    print(point_st)
    print(trans)
    try:
        point_base_link_frame = tf2_geometry_msgs.do_transform_point(point_st, trans)
        point_base_link_frame.point.x *= 0.9 # reduce
        # debug
        marker_points.points.append(point_base_link_frame.point)
    except Exception as e:
        print(e)
        None

    marker_pub.publish(marker_points)
    marker_arrayblue.markers.append(falmarker)
    bluepoints.publish(marker_arrayblue)
    goalmarkerarray = MarkerArray()
    goalmarkerarray.markers.append(goalmarker)
    goalmarker_pub.publish(goalmarkerarray)
    goalmarker.points = []
    goalmarkerarray.markers = []
    marker_arrayblue.markers = []
    falmarker.points = []
    marker_points.points = []  

    messageS1.data += "\ntarget_angle: %.1f" % (target_angle)
    messageS1.data += "\nr: %.1f l: %.1f" % (right_d, left_d) 
    messageS1.data += "\nforward_d: %.1f" % (target_distance)
    velocity = -1.0 * target_distance
    try:
        steering_err = calcPursuitAngle(point_base_link_frame.point.x, point_base_link_frame.point.y)
    except Exception as e:
        rospy.loginfo(e)
        steering_err = calcPursuitAngle(1, -1)
        rospy.loginfo("err")
    messageS1.data += "\nsteer: %.1f" % (steering_err)
    pubst1.publish(messageS1)
    marker_points.points = []
    steering_err = (steering_err + prev_steering_err) / 2
    velocity = (velocity + prev_velocity) / 2
    prev_steering_err = steering_err
    prev_velocity = velocity
    return steering_err, velocity

# pure pursuit steering angle calc
def calcPursuitAngle(goal_x, goal_y):
    alpha = math.atan2(goal_y, goal_x)
    lookahead_distance = math.sqrt(pow(goal_x, 2) + pow(goal_y, 2))
    steering_angle = math.atan2(2.0 * WHEELBASE * np.sin(alpha) / (lookahead_distance), 1)
    return steering_angle

def getDistance(ranges, angles):
    """
    Calculate the distance, angle, left distance, and right distance based on the given ranges and angles.
    
    Args:
        ranges (list): List of range values.
        angles (list): List of angle values.
    
    Returns:
        tuple: A tuple containing the distance, angle, left distance, and right distance.
    """
    global marker_points
    if(len(ranges) > 50):
        center1_min_index = np.where(math.radians(60) < angles)[0][0]
        center1_max_index = np.where(math.radians(179.9) < angles)[0][0]
        tmp1 = np.arange(center1_min_index, center1_max_index, 1)
        center2_min_index = np.where(math.radians(-179.9) < angles)[0][0]
        center2_max_index = np.where(math.radians(-60) < angles)[0][0]
        tmp2 = np.arange(center2_min_index, center2_max_index, 1)
        tmp = np.concatenate((tmp1, tmp2))
        max_x = -10.0
        for t in tmp:
            point = Point()
            point.x, point.y = calcPointPos(ranges[t], angles[t])
            if not math.isinf(point.x):
                point.z = 1
                falmarker.points.append(point)
                # find max (flipped upside min)
                if point.x > max_x:
                    max_x = point.x
        # Get points for new algorithm
        p1, p2 = getFarthestNeighbours(falmarker.points)
        goalmarker.points.append(p1)
        goalmarker.points.append(p2)
        p3 = Point()
        p3.x = (p1.x+p2.x)/2
        p3.y = (p1.y+p2.y)/2
        p3.z = 1
        goalmarker.points.append(p3)
        left_d = p1.y
        right_d = p2.y
        angle = p3.y
        distance = p3.x
        if math.isinf(max_x):
            max_x = -5.0
        # within 40 cm reverse - tolatas    
        if max_x > -0.4:
            max_x = 0.5
        distance = p3.x
        if math.isinf(right_d):
            right_d = -99.0
            # angle = 0.0
        if math.isinf(left_d):
            left_d = 99.0
            # angle = 0.0
    else: 
        distance = 0
        # angle = 0
    return distance, angle, left_d, right_d

def calcPointPos(range, angle):
    x1 = range * math.cos(angle)
    y1 = range * math.sin(angle)
    return x1, y1

def getFarthestNeighbours(points):
    p1, p2 = 0, 0
    maxdiff = 0
    prev_point = None
    for i in points:
        if prev_point is not None:
            if math.sqrt((prev_point.x-i.x)**2+(prev_point.y-i.y)**2) > maxdiff:
                p1 = i
                p2 = prev_point
                maxdiff = math.sqrt((prev_point.x-i.x)**2+(prev_point.y-i.y)**2)
        prev_point = i
    return p1,p2

if __name__ == '__main__':
    first_run = True
    rospy.init_node('dist_finder',anonymous = True)
    rospy.Subscriber("scan",LaserScan,callbackLaser)
    rospy.loginfo("Simple pursuit node started")
    """
    rosrun tf tf_echo base_link laser
    - Translation: [0.260, 0.000, 0.228]
    - Rotation: in Quaternion [0.000, 0.000, 1.000, 0.001]
                in RPY (radian) [0.000, -0.000, 3.140]
                in RPY (degree) [0.000, -0.000, 179.909]
    header:
        frame_id: "base_link"
    child_frame_id: "laser"
    transform:
        translation:
            x: 0.26
            y: 0.0
            z: 0.228
        rotation:
            x: 0.0
            y: 0.0
            z: 0.999999682932
            w: 0.000796326710733
    """
    rate = rospy.Rate(2) # 2hz
    buf = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buf)

    while not rospy.is_shutdown():
        if first_run:
            try:
                trans = buf.lookup_transform("base_link", "laser", rospy.Time.now())
                rospy.loginfo("Got laser fransform")
                first_run = False
                #rospy.loginfo(trans)
            except:
                trans.transform.translation.x = 0.26
                trans.transform.translation.y = 0.0
                trans.transform.translation.z = 0.228
                trans.transform.rotation.x = 0.0
                trans.transform.rotation.y = 0.0
                trans.transform.rotation.z = 0.999999682932
                trans.transform.rotation.w = 0.000796326710733
                #rospy.loginfo(trans)
                #rospy.logerr("No transform to laser assuming original!")
        try:
            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass