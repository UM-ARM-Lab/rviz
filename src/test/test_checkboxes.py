#!/usr/bin/env python

import math
import time
from math import cos, pi
from math import sin

import numpy as np
import rosgraph
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import Point32, AccelStamped, Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path, GridCells
from sensor_msgs import point_cloud2
from sensor_msgs.msg import JointState, Image, FluidPressure, Illuminance, LaserScan
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs.msg import Range
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray


def get_connected_publisher(topic_path: str, type, queue_size=10):
    rospy.loginfo(f"Checking for publisher on topic {topic_path} with type {type}")
    pub = rospy.Publisher(topic_path, type, queue_size=queue_size)
    num_subs = len(_get_subscribers(topic_path))
    for i in range(10):
        num_cons = pub.get_num_connections()
        if num_cons == num_subs:
            return pub
        time.sleep(0.1)
    rospy.logwarn("failed to get publisher")
    return pub


def _get_subscribers(topic_path: str):
    ros_master = rosgraph.Master('/rostopic')
    topic_path = rosgraph.names.script_resolve_name('rostopic', topic_path)
    state = ros_master.getSystemState()
    subs = []
    for sub in state[1]:
        if sub[0] == topic_path:
            subs.extend(sub[1])
    return subs


def publish_accel_stamped():
    publisher = get_connected_publisher("accel", AccelStamped)
    m = AccelStamped()
    m.header.frame_id = 'base_link'
    m.accel.linear.x = 1
    m.accel.angular.x = 1
    publisher.publish(m)


def publish_depthcloud():
    rng = np.random.RandomState(0)

    color_pub = get_connected_publisher('depthcloud_color', Image)
    depth_pub = get_connected_publisher('depthcloud_depth', Image)

    color = Image()
    color.header.frame_id = 'base_link'
    color.header.stamp = rospy.Time.now()
    color.data = rng.uniform(0, 255, size=64 * 64 * 3).astype(np.uint8).flatten().tolist()
    color.encoding = 'rgb8'
    color.width = 64
    color.height = 64
    color.step = 3

    depth = Image()
    depth.header.stamp = rospy.Time.now()
    depth.header.frame_id = 'base_link'
    depth.data = rng.uniform(0, 100, size=64 * 64).astype(np.uint16).flatten().tolist()
    depth.encoding = '16UC1'
    depth.width = 64
    depth.height = 64
    depth.step = 1

    color_pub.publish(color)
    depth_pub.publish(depth)


def publish_effort():
    topic = 'effort'
    publisher = get_connected_publisher(topic, JointState, queue_size=1)

    t = 0
    msg = JointState()
    msg.name = ['head_pan_joint']
    N = len(msg.name)
    msg.position = [0. for _ in range(N)]

    msg.header.stamp = rospy.Time.now()
    msg.effort = [10. * sin(t) for _ in range(N)]
    publisher.publish(msg)


def publish_fluid_pressure():
    topic = 'pressure'
    publisher = get_connected_publisher(topic, FluidPressure, queue_size=1)

    msg = FluidPressure()
    msg.header.frame_id = 'base_link'
    msg.fluid_pressure = 50000.0
    msg.variance = 0.3
    publisher.publish(msg)


def publish_grid_cells():
    topic = 'grid_cells'
    publisher = get_connected_publisher(topic, GridCells, queue_size=1)

    msg = GridCells()
    msg.header.frame_id = 'base_link'
    msg.cell_width = 0.1
    msg.cell_height = 0.1
    s = 0.1
    for i, j in np.ndindex(3, 2):
        msg.cells.append(Point(2.0 + i * s, 2.0 + j * s, s))
    publisher.publish(msg)


def publish_illuminance():
    topic = 'illuminance'
    publisher = get_connected_publisher(topic, Illuminance, queue_size=1)

    msg = Illuminance()
    msg.header.frame_id = 'base_link'
    msg.illuminance = 1.0
    msg.variance = 0.3
    publisher.publish(msg)


def publish_image():
    rng = np.random.RandomState(0)

    pub = get_connected_publisher('image', Image)

    color = Image()
    color.header.frame_id = 'base_link'
    color.header.stamp = rospy.Time.now()
    color.data = rng.uniform(0, 255, size=64 * 64 * 3).astype(np.uint8).flatten().tolist()
    color.encoding = 'rgb8'
    color.width = 64
    color.height = 64
    color.step = 3

    pub.publish(color)


def publish_scan():
    pub = get_connected_publisher('scan', LaserScan)

    msg = LaserScan()
    msg.header.frame_id = 'base_link'
    msg.header.stamp = rospy.Time.now()
    n = 100
    msg.angle_increment = np.pi / n
    msg.angle_min = 0.0
    msg.angle_max = np.pi
    msg.ranges = [1] * n
    msg.range_min = 0
    msg.range_max = 10
    msg.intensities = np.linspace(0, 1, n)

    pub.publish(msg)


def publish_map():
    topic = 'map'
    publisher = get_connected_publisher(topic, OccupancyGrid)
    grid = OccupancyGrid()

    t = 0
    grid.header.frame_id = "base_link"
    grid.header.stamp = rospy.Time.now()
    grid.info.map_load_time = rospy.Time.now()
    grid.info.resolution = 1.0
    grid.info.width = 3
    grid.info.height = 3
    grid.info.origin.position.x = math.cos(t)
    grid.info.origin.position.y = math.sin(t)
    grid.info.origin.orientation.w = 1.0
    grid.data = [0, 20, 40, 60, 80, 100, 120, -10, -100]

    publisher.publish(grid)


def publish_path():
    topic = 'test_path'
    publisher = get_connected_publisher(topic, Path)

    t = 0

    p = Path()
    p.header.frame_id = "base_link"
    p.header.stamp = rospy.Time.now()

    num_points = 50
    for i in range(0, num_points):
        ps = PoseStamped()
        ps.header.stamp = p.header.stamp
        ps.header.frame_id = p.header.frame_id
        ps.pose.position.x = 10.0 * i / (num_points - 1) - 5
        ps.pose.position.y = math.sin(10.0 * i / num_points + t)
        ps.pose.position.z = 0
        angle = 2 * math.pi * i / num_points + t
        quat = quaternion_from_euler(angle, 0, 0)
        ps.pose.orientation.x = quat[0]
        ps.pose.orientation.y = quat[1]
        ps.pose.orientation.z = quat[2]
        ps.pose.orientation.w = quat[3]
        p.poses.append(ps)

    publisher.publish(p)


def publish_tf():
    br = tf.TransformBroadcaster()
    t = 0
    for m in range(1, 2):
        br.sendTransform((0, 1, 0),
                         quaternion_from_euler(0, .8 * m, 0),
                         rospy.Time.now(),
                         "frame0_{0}".format(m),
                         "base_link")
        for n in range(1, 3):
            br.sendTransform((.2 * math.sin(t * 10 + n), 0, .2),
                             quaternion_from_euler(0, .6 * math.sin(t), .2),
                             rospy.Time.now(),
                             "frame{0}_{1}".format(n, m),
                             "frame{0}_{1}".format(n - 1, m))


def publish_covariance():
    publisher_cov = get_connected_publisher('pose_with_cov', PoseWithCovarianceStamped, queue_size=5)
    publisher_pose = get_connected_publisher('pose', PoseStamped, queue_size=5)

    br = tf2_ros.TransformBroadcaster()
    # radius = 1
    angle = 0
    # r = 0
    # p = 0
    # y = 0

    linear_deviation = 0.5

    stamp = rospy.Time.now()

    # Define static pose with covariance
    pose_with_cov = PoseWithCovarianceStamped()
    pose_with_cov.header.frame_id = "base_link"
    pose_with_cov.header.stamp = stamp

    pose_with_cov.pose.pose.position.x = 3
    pose_with_cov.pose.pose.position.y = 3
    pose_with_cov.pose.pose.position.z = 3

    ori = pose_with_cov.pose.pose.orientation
    ori.x, ori.y, ori.z, ori.w = quaternion_from_euler(pi / 2, pi / 3, 0)

    pose_with_cov.pose.covariance[0] = linear_deviation ** 2.0
    pose_with_cov.pose.covariance[6 + 1] = 0.0001
    pose_with_cov.pose.covariance[12 + 2] = 0.0001
    pose_with_cov.pose.covariance[18 + 3] = 0.01
    pose_with_cov.pose.covariance[24 + 4] = 0.01
    pose_with_cov.pose.covariance[30 + 5] = 0.01

    # Define a dynamic pose that should move inside the deviation
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.header.stamp = stamp

    pose.pose.position.x = pose_with_cov.pose.pose.position.x + linear_deviation * cos(10 * angle)
    pose.pose.position.y = pose_with_cov.pose.pose.position.y
    pose.pose.position.z = pose_with_cov.pose.pose.position.z

    ori = pose.pose.orientation
    ori.x, ori.y, ori.z, ori.w = quaternion_from_euler(pi / 2, pi / 3, 0)

    publisher_cov.publish(pose_with_cov)
    publisher_pose.publish(pose)

    t = TransformStamped()
    t.header.frame_id = 'base_link'
    t.header.stamp = stamp
    t.child_frame_id = 'pose'

    t.transform.translation = pose_with_cov.pose.pose.position
    t.transform.rotation = pose_with_cov.pose.pose.orientation
    br.sendTransform(t)


def publish_range():
    topic = 'test_range'
    publisher = get_connected_publisher(topic, Range)

    dist = 3

    r = Range()
    r.header.frame_id = "base_link"
    r.header.stamp = rospy.Time.now()

    r.radiation_type = 0
    r.field_of_view = 2.0 / dist
    r.min_range = .4
    r.max_range = 10
    r.range = dist

    publisher.publish(r)

def publish_robot_model():
    topic = 'joint_states'
    publisher = get_connected_publisher(topic, JointState)

    j = JointState()
    publisher.publish(j)

def pubish_polygon():
    br = tf.TransformBroadcaster()
    for _ in range(10):
        br.sendTransform((0.0, -2.0, 0.5),
                         quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "head_tilt_link",
                         "base_link",
                         )
        rospy.sleep(0.1)


def publish_pointcloud2():
    pub = get_connected_publisher('points2', PointCloud2, queue_size=100)

    width = 100
    height = 100

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('intensity', 12, PointField.FLOAT32, 1)]

    header = Header()
    header.frame_id = "base_link"
    header.stamp = rospy.Time.now()

    x, y = np.meshgrid(np.linspace(-2, 2, width), np.linspace(-2, 2, height))
    z = 0.5 * np.sin(2 * x / 10.0) * np.sin(2 * y)
    points = np.array([x, y, z, z]).reshape(4, -1).T

    pc2 = point_cloud2.create_cloud(header, fields, points)
    pub.publish(pc2)


def publish_pose_array():
    topic = 'test_poses'
    publisher = get_connected_publisher(topic, PoseArray)

    ps = PoseArray()
    ps.header.frame_id = "base_link"
    ps.header.stamp = rospy.Time.now()

    pose = Pose()
    pose.position.x = 2
    pose.position.y = 2
    pose.position.z = 0
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = .7071
    pose.orientation.w = .7071

    ps.poses.append(pose)

    pose = Pose()
    pose.position.x = 1
    pose.position.y = 1
    pose.position.z = 0
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1

    ps.poses.append(pose)

    publisher.publish(ps)


def publish_pose():
    topic = 'test_pose'
    publisher = get_connected_publisher(topic, PoseStamped)

    t = 0
    p = PoseStamped()
    p.header.frame_id = "base_link"
    p.header.stamp = rospy.Time.now()

    r = 5.0
    p.pose.position.x = r * math.cos(t)
    p.pose.position.y = r * math.sin(t)
    p.pose.position.z = 0
    p.pose.orientation.x = 0
    p.pose.orientation.y = 0
    p.pose.orientation.z = math.sin(.5 * t)
    p.pose.orientation.w = math.cos(.5 * t)

    publisher.publish(p)


def publish_point_stamped():
    topic = 'test_point'
    publisher = get_connected_publisher(topic, PointStamped)

    t = 0
    p = PointStamped()
    p.header.frame_id = "base_link"
    p.header.stamp = rospy.Time.now()

    r = 5.0
    p.point.x = r * math.cos(t)
    p.point.y = r * math.sin(t)
    p.point.z = 0

    publisher.publish(p)


def publish_marker_array():
    marker_pub = get_connected_publisher('markerarray', MarkerArray)

    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id = 'base_link'
    m.header.stamp = rospy.Time.now()
    m.ns = 'test'
    m.id = 0
    m.type = Marker.CUBE
    m.pose.position.x = 0
    m.pose.position.y = 2
    m.pose.orientation.y = 0
    m.pose.orientation.w = 1
    m.scale.x = 2.1
    m.scale.y = 0.15
    m.scale.z = 0.1
    m.color.r = 1.0
    m.color.g = 0.5
    m.color.b = 0.2
    m.color.a = 0.3
    m.color.r = 0.0
    m.color.g = 1.0
    m.color.b = 1.0
    m.color.a = 1.0
    msg = MarkerArray()
    msg.markers.append(m)

    marker_pub.publish(msg)

def publish_marker():
    marker_pub = get_connected_publisher('marker_test', Marker)

    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id = 'base_link'
    m.header.stamp = rospy.Time.now()
    m.ns = 'test'
    m.id = 0
    m.type = Marker.CUBE
    m.pose.position.x = 0
    m.pose.position.y = 2
    m.pose.orientation.y = 0
    m.pose.orientation.w = 1
    m.scale.x = 0.1
    m.scale.y = 0.15
    m.scale.z = 0.1
    m.color.r = 1.0
    m.color.g = 0.5
    m.color.b = 0.2
    m.color.a = 0.3
    m.color.r = 1.0
    m.color.g = 0.0
    m.color.b = 1.0
    m.color.a = 1.0

    marker_pub.publish(m)


def publish_odometry():
    topic = 'test_odometry'
    publisher = get_connected_publisher(topic, Odometry)

    y = 0

    odo = Odometry()
    odo.header.frame_id = "base_link"
    odo.header.stamp = rospy.Time.now()

    odo.pose.pose.position.x = 0
    odo.pose.pose.position.y = y
    odo.pose.pose.position.z = 0

    odo.pose.pose.orientation.x = 0
    odo.pose.pose.orientation.y = 0
    odo.pose.pose.orientation.z = 0
    odo.pose.pose.orientation.w = 1

    publisher.publish(odo)


def publish_wrench():
    topic = 'test_wrench'
    publisher = get_connected_publisher(topic, WrenchStamped)

    t = 0
    p = WrenchStamped()
    p.header.frame_id = "base_link"
    p.header.stamp = rospy.Time.now()

    f = 0.5 * math.sin(t)
    p.wrench.force.x = 0
    p.wrench.force.y = 0
    p.wrench.force.z = f * math.sin(t)

    q = 0.5 + 0.5 * math.sin(t / 3.14)
    p.wrench.torque.x = q * math.sin(t)
    p.wrench.torque.y = q * math.cos(t)
    p.wrench.torque.z = 0

    publisher.publish(p)



def main():
    rospy.init_node("test_checkboxes")
    # publish_accel_stamped()
    pubish_polygon()
    publish_covariance()
    publish_depthcloud()
    publish_effort()
    publish_fluid_pressure()
    publish_grid_cells()
    publish_illuminance()
    publish_image()
    publish_map()
    publish_marker()
    publish_marker_array()
    publish_odometry()
    publish_path()
    publish_point_stamped()
    publish_pointcloud2()
    publish_pose()
    publish_pose_array()
    publish_range()
    publish_robot_model()
    publish_scan()
    publish_tf()
    publish_wrench()
    rospy.sleep(1)


if __name__ == '__main__':
    main()
