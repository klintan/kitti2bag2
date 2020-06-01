#!env python
# -*- coding: utf-8 -*-

import sys
import glob
import ctypes
import struct

try:
    import pykitti
except ImportError as e:
    print('Could not load module \'pykitti\'. Please run `pip install pykitti`')
    sys.exit(1)

# import tf2_ros
import os
import cv2
import rclpy
from rclpy.time import Time
from rclpy.serialization import serialize_message
import rosbag2_py._rosbag2_py as rosbag2_py

import progressbar
from tf2_msgs.msg import TFMessage
from datetime import datetime
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, Imu, PointField, NavSatFix, PointCloud2
from geometry_msgs.msg import TransformStamped, TwistStamped, Transform
from transforms3d.euler import euler2quat as quaternion_from_euler
from transforms3d.quaternions import mat2quat as quaternion_from_matrix
from cv_bridge import CvBridge
import numpy as np
import argparse

_DATATYPES = {}
_DATATYPES[PointField.INT8] = ('b', 1)
_DATATYPES[PointField.UINT8] = ('B', 1)
_DATATYPES[PointField.INT16] = ('h', 2)
_DATATYPES[PointField.UINT16] = ('H', 2)
_DATATYPES[PointField.INT32] = ('i', 4)
_DATATYPES[PointField.UINT32] = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)


def create_topic(topic_name, msg_type, serialization_format):
    topic = rosbag2_py.TopicMetadata()
    topic.name = topic_name
    topic.serialization_format = serialization_format
    topic.type = msg_type
    return topic


def save_imu_data(bag, kitti, imu_frame_id, topic_name):
    print("Exporting IMU")
    topic = create_topic(topic_name, "sensor_msgs/msg/Imu", "cdr")
    bag.create_topic(topic)

    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        q = quaternion_from_euler(oxts.packet.roll, oxts.packet.pitch, oxts.packet.yaw)
        imu = Imu()
        imu.header.frame_id = imu_frame_id
        imu.header.stamp = Time(seconds=float(timestamp.strftime("%s.%f"))).to_msg()
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        imu.linear_acceleration.x = oxts.packet.af
        imu.linear_acceleration.y = oxts.packet.al
        imu.linear_acceleration.z = oxts.packet.au
        imu.angular_velocity.x = oxts.packet.wf
        imu.angular_velocity.y = oxts.packet.wl
        imu.angular_velocity.z = oxts.packet.wu
        bag.write((topic_name, serialize_message(imu), imu.header.stamp.sec))


def save_dynamic_tf(bag, kitti, kitti_type, initial_time):
    print("Exporting time dependent transformations")
    topic_name = '/tf'
    topic = create_topic(topic_name, "tf2_msgs/msg/TFMessage", "cdr")
    bag.create_topic(topic)

    if kitti_type.find("raw") != -1:
        for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
            tf_oxts_msg = TFMessage()
            tf_oxts_transform = TransformStamped()
            tf_oxts_transform.header.stamp = Time(seconds=float(timestamp.strftime("%s.%f"))).to_msg()
            tf_oxts_transform.header.frame_id = 'world'
            tf_oxts_transform.child_frame_id = 'base_link'

            transform = (oxts.T_w_imu)
            t = transform[0:3, 3]
            q = quaternion_from_matrix(transform[0:3, 0:3])
            oxts_tf = Transform()

            oxts_tf.translation.x = t[0]
            oxts_tf.translation.y = t[1]
            oxts_tf.translation.z = t[2]

            oxts_tf.rotation.x = q[0]
            oxts_tf.rotation.y = q[1]
            oxts_tf.rotation.z = q[2]
            oxts_tf.rotation.w = q[3]

            tf_oxts_transform.transform = oxts_tf
            tf_oxts_msg.transforms.append(tf_oxts_transform)

            bag.write((topic_name, serialize_message(tf_oxts_msg), tf_oxts_msg.transforms[0].header.stamp.sec))

    elif kitti_type.find("odom") != -1:
        timestamps = map(lambda x: initial_time + x.total_seconds(), kitti.timestamps)
        for timestamp, tf_matrix in zip(timestamps, kitti.T_w_cam0):
            tf_msg = TFMessage()
            tf_stamped = TransformStamped()
            tf_stamped.header.stamp = Time(seconds=timestamp).to_msg()
            tf_stamped.header.frame_id = 'world'
            tf_stamped.child_frame_id = 'camera_left'

            t = tf_matrix[0:3, 3]
            q = quaternion_from_matrix(tf_matrix)
            transform = Transform()

            transform.translation.x = t[0]
            transform.translation.y = t[1]
            transform.translation.z = t[2]

            transform.rotation.x = q[0]
            transform.rotation.y = q[1]
            transform.rotation.z = q[2]
            transform.rotation.w = q[3]

            tf_stamped.transform = transform
            tf_msg.transforms.append(tf_stamped)

            bag.write('/tf', serialize_message(tf_msg), tf_msg.transforms[0].header.stamp)


def save_camera_data(bag, kitti_type, kitti, util, bridge, camera, camera_frame_id, topic_name, initial_time):
    print(f"Exporting camera {camera}")

    if kitti_type.find("raw") != -1:
        camera_pad = '{0:02d}'.format(camera)
        image_dir = os.path.join(kitti.data_path, 'image_{}'.format(camera_pad))
        image_path = os.path.join(image_dir, 'data')
        image_filenames = sorted(os.listdir(image_path))
        with open(os.path.join(image_dir, 'timestamps.txt')) as f:
            image_datetimes = map(lambda x: datetime.strptime(x[:-4], '%Y-%m-%d %H:%M:%S.%f'), f.readlines())

        calib = CameraInfo()
        calib.header.frame_id = camera_frame_id
        calib.width, calib.height = tuple([int(x) for x in util[f'S_rect_{camera_pad}']])
        calib.distortion_model = 'plumb_bob'
        calib.k = util[f'K_{camera_pad}']
        calib.r = util[f'R_rect_{camera_pad}']
        calib.d = [float(x) for x in util[f'D_{camera_pad}']]
        calib.p = util[f'P_rect_{camera_pad}']

    elif kitti_type.find("odom") != -1:
        camera_pad = '{0:01d}'.format(camera)
        image_path = os.path.join(kitti.sequence_path, f'image_{camera_pad}')
        image_filenames = sorted(os.listdir(image_path))
        image_datetimes = map(lambda x: initial_time + x.total_seconds(), kitti.timestamps)

        calib = CameraInfo()
        calib.header.frame_id = camera_frame_id
        calib.p = util[f'P{camera_pad}']

    iterable = zip(image_datetimes, image_filenames)
    bar = progressbar.ProgressBar()
    for dt, filename in bar(iterable):
        image_filename = os.path.join(image_path, filename)
        cv_image = cv2.imread(image_filename)
        calib.height, calib.width = cv_image.shape[:2]
        if camera in (0, 1):
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        encoding = "mono8" if camera in (0, 1) else "bgr8"
        image_message = bridge.cv2_to_imgmsg(cv_image, encoding=encoding)
        image_message.header.frame_id = camera_frame_id
        if kitti_type.find("raw") != -1:
            image_message.header.stamp = Time(seconds=float(datetime.strftime(dt, "%s.%f"))).to_msg()
            topic_ext = "/image_raw"
        elif kitti_type.find("odom") != -1:
            image_message.header.stamp = Time(seconds=dt).to_msg()
            topic_ext = "/image_rect"
        calib.header.stamp = image_message.header.stamp

        camera_info_topic = create_topic(topic_name + '/camera_info', "sensor_msgs/msg/CameraInfo", "cdr")
        bag.create_topic(camera_info_topic)

        camera_topic = create_topic(topic_name + topic_ext, "sensor_msgs/msg/Image", "cdr")
        bag.create_topic(camera_topic)

        bag.write((topic_name + topic_ext, serialize_message(image_message), image_message.header.stamp.sec))
        bag.write((topic_name + '/camera_info', serialize_message(calib), calib.header.stamp.sec))


# Helper functions from ROS1 http://docs.ros.org/melodic/api/sensor_msgs/html/point__cloud2_8py_source.html
def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt

# Helper functions from ROS1 http://docs.ros.org/melodic/api/sensor_msgs/html/point__cloud2_8py_source.html
def create_cloud(header, fields, points):
    """
    Create a L{sensor_msgs.msg.PointCloud2} message.

    @param header: The point cloud header.
    @type  header: L{std_msgs.msg.Header}
    @param fields: The point cloud fields.
    @type  fields: iterable of L{sensor_msgs.msg.PointField}
    @param points: The point cloud points.
    @type  points: list of iterables, i.e. one iterable for each point, with the
                   elements of each iterable being the values of the fields for
                   that point (in the same order as the fields parameter)
    @return: The point cloud.
    @rtype:  L{sensor_msgs.msg.PointCloud2}
    """

    cloud_struct = struct.Struct(_get_struct_fmt(False, fields))

    buff = ctypes.create_string_buffer(cloud_struct.size * len(points))

    point_step, pack_into = cloud_struct.size, cloud_struct.pack_into
    offset = 0
    for p in points:
        pack_into(buff, offset, *p)
        offset += point_step

    return PointCloud2(header=header,
                       height=1,
                       width=len(points),
                       is_dense=False,
                       is_bigendian=False,
                       fields=fields,
                       point_step=cloud_struct.size,
                       row_step=cloud_struct.size * len(points),
                       data=buff.raw)

def save_velo_data(bag, kitti, velo_frame_id, topic_name):
    print("Exporting velodyne data")

    topic = create_topic(topic_name + '/pointcloud', "sensor_msgs/msg/PointCloud2", "cdr")
    bag.create_topic(topic)

    velo_path = os.path.join(kitti.data_path, 'velodyne_points')
    velo_data_dir = os.path.join(velo_path, 'data')
    velo_filenames = sorted(os.listdir(velo_data_dir))
    with open(os.path.join(velo_path, 'timestamps.txt')) as f:
        lines = f.readlines()
        velo_datetimes = []
        for line in lines:
            if len(line) == 1:
                continue
            dt = datetime.strptime(line[:-4], '%Y-%m-%d %H:%M:%S.%f')
            velo_datetimes.append(dt)

    iterable = zip(velo_datetimes, velo_filenames)
    bar = progressbar.ProgressBar()
    for dt, filename in bar(iterable):
        if dt is None:
            continue

        velo_filename = os.path.join(velo_data_dir, filename)

        # read binary data
        points = (np.fromfile(velo_filename, dtype=np.float32)).reshape(-1, 4)

        # create header
        header = Header()
        header.frame_id = velo_frame_id
        header.stamp = Time(seconds=float(datetime.strftime(dt, "%s.%f"))).to_msg()

        # fill pcl msg
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                  PointField(name='i', offset=12, datatype=PointField.FLOAT32, count=1)]

        pcl_msg = create_cloud(header, fields, points)

        bag.write((topic_name + '/pointcloud', serialize_message(pcl_msg), pcl_msg.header.stamp.sec))


def get_static_transform(from_frame_id, to_frame_id, transform):
    t = transform[0:3, 3]
    q = quaternion_from_matrix(transform[0:3, 0:3])
    tf_msg = TransformStamped()
    tf_msg.header.frame_id = from_frame_id
    tf_msg.child_frame_id = to_frame_id
    tf_msg.transform.translation.x = float(t[0])
    tf_msg.transform.translation.y = float(t[1])
    tf_msg.transform.translation.z = float(t[2])
    tf_msg.transform.rotation.x = float(q[0])
    tf_msg.transform.rotation.y = float(q[1])
    tf_msg.transform.rotation.z = float(q[2])
    tf_msg.transform.rotation.w = float(q[3])
    return tf_msg


def inv(transform):
    """
    Invert rigid body transformation matrix
    """

    R = transform[0:3, 0:3]
    t = transform[0:3, 3]
    t_inv = -1 * R.T.dot(t)
    transform_inv = np.eye(4)
    transform_inv[0:3, 0:3] = R.T
    transform_inv[0:3, 3] = t_inv
    return transform_inv


def save_static_transforms(bag, transforms, timestamps):
    print("Exporting static transformations")
    topic_name = '/tf_static'
    topic = create_topic(topic_name, "tf2_msgs/msg/TFMessage", "cdr")
    bag.create_topic(topic)

    tfm = TFMessage()
    for transform in transforms:
        t = get_static_transform(from_frame_id=transform[0], to_frame_id=transform[1], transform=transform[2])
        tfm.transforms.append(t)
    for timestamp in timestamps:
        time = Time(seconds=float(timestamp.strftime("%s.%f"))).to_msg()
        for i in range(len(tfm.transforms)):
            tfm.transforms[i].header.stamp = time
        bag.write((topic_name, serialize_message(tfm), time.sec))


def save_gps_fix_data(bag, kitti, gps_frame_id, topic_name):
    topic = create_topic(topic_name, "sensor_msgs/msg/NavSatFix", "cdr")
    bag.create_topic(topic)

    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header.frame_id = gps_frame_id
        navsatfix_msg.header.stamp = Time(seconds=float(timestamp.strftime("%s.%f"))).to_msg()
        navsatfix_msg.latitude = oxts.packet.lat
        navsatfix_msg.longitude = oxts.packet.lon
        navsatfix_msg.altitude = oxts.packet.alt
        navsatfix_msg.status.service = 1
        bag.write((topic_name, serialize_message(navsatfix_msg), navsatfix_msg.header.stamp.sec))


def save_gps_vel_data(bag, kitti, gps_frame_id, topic_name):
    topic = create_topic(topic_name, "geometry_msgs/msg/TwistStamped", 'cdr')
    bag.create_topic(topic)

    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = gps_frame_id
        twist_msg.header.stamp = Time(seconds=float(timestamp.strftime("%s.%f"))).to_msg()
        twist_msg.twist.linear.x = oxts.packet.vf
        twist_msg.twist.linear.y = oxts.packet.vl
        twist_msg.twist.linear.z = oxts.packet.vu
        twist_msg.twist.angular.x = oxts.packet.wf
        twist_msg.twist.angular.y = oxts.packet.wl
        twist_msg.twist.angular.z = oxts.packet.wu
        bag.write((topic_name, serialize_message(twist_msg), twist_msg.header.stamp.sec))


def create_bag(bag_path):
    for f in glob.glob(f'{bag_path}/*'):
        os.remove(f)

    os.makedirs(f"{bag_path}/", exist_ok=True)
    storage_options = rosbag2_py.StorageOptions()
    storage_options.uri = bag_path
    storage_options.storage_id = 'sqlite3'

    serialization_format = 'cdr'
    converter_options = rosbag2_py.ConverterOptions()
    converter_options.input_serialization_format = serialization_format
    converter_options.output_serialization_format = serialization_format

    bag = rosbag2_py.Writer('SequentialWriter')
    bag.open(storage_options, converter_options)
    return bag


def run_kitti2bag():
    parser = argparse.ArgumentParser(description="Convert KITTI dataset to ROS2 bag file the easy way!")
    # Accepted argument values
    kitti_types = ["raw_synced", "odom_color", "odom_gray"]
    serialization_formats = ['cdr']
    odometry_sequences = []
    for s in range(22):
        odometry_sequences.append(str(s).zfill(2))

    parser.add_argument("kitti_type", choices=kitti_types, help="KITTI dataset type")
    parser.add_argument("dir", nargs="?", default=os.getcwd(),
                        help="base directory of the dataset, if no directory passed the deafult is current working directory")
    parser.add_argument("-t", "--date",
                        help="date of the raw dataset (i.e. 2011_09_26), option is only for RAW datasets.")
    parser.add_argument("-r", "--drive",
                        help="drive number of the raw dataset (i.e. 0001), option is only for RAW datasets.")
    parser.add_argument("-s", "--sequence", choices=odometry_sequences,
                        help="sequence of the odometry dataset (between 00 - 21), option is only for ODOMETRY datasets.")
    parser.add_argument("-f", "--serialization_format", choices=serialization_formats, default="cdr",
                        help="Message serialization format.")
    args = parser.parse_args()

    bridge = CvBridge()

    # CAMERAS
    cameras = [
        (0, 'camera_gray_left', '/kitti/camera_gray_left'),
        (1, 'camera_gray_right', '/kitti/camera_gray_right'),
        (2, 'camera_color_left', '/kitti/camera_color_left'),
        (3, 'camera_color_right', '/kitti/camera_color_right')
    ]

    if args.kitti_type.find("raw") != -1:

        if not args.date:
            print("Date option is not given. It is mandatory for raw dataset.")
            print("Usage for raw dataset: kitti2bag2 raw_synced [dir] -t <date> -r <drive>")
            sys.exit(1)
        elif not args.drive:
            print("Drive option is not given. It is mandatory for raw dataset.")
            print("Usage for raw dataset: kitti2bag2 raw_synced [dir] -t <date> -r <drive>")
            sys.exit(1)

        bag_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.realpath(__file__))), 'resources',
            f'kitti_{args.date}_drive_{args.drive}_{args.kitti_type[4:]}')

        bag = create_bag(bag_path)

        kitti = pykitti.raw(args.dir, args.date, args.drive)
        if not os.path.exists(kitti.data_path):
            print('Path {} does not exists. Exiting.'.format(kitti.data_path))
            sys.exit(1)

        if len(kitti.timestamps) == 0:
            print('Dataset is empty? Exiting.')
            sys.exit(1)

        try:
            # IMU
            imu_frame_id = 'imu_link'
            imu_topic = '/kitti/oxts/imu'
            gps_fix_topic = '/kitti/oxts/gps/fix'
            gps_vel_topic = '/kitti/oxts/gps/vel'
            velo_frame_id = 'velo_link'
            velo_topic = '/kitti/velo'

            T_base_link_to_imu = np.eye(4, 4)
            T_base_link_to_imu[0:3, 3] = [-2.71 / 2.0 - 0.05, 0.32, 0.93]

            # tf_static
            transforms = [
                ('base_link', imu_frame_id, T_base_link_to_imu),
                (imu_frame_id, velo_frame_id, inv(kitti.calib.T_velo_imu)),
                (imu_frame_id, cameras[0][1], inv(kitti.calib.T_cam0_imu)),
                (imu_frame_id, cameras[1][1], inv(kitti.calib.T_cam1_imu)),
                (imu_frame_id, cameras[2][1], inv(kitti.calib.T_cam2_imu)),
                (imu_frame_id, cameras[3][1], inv(kitti.calib.T_cam3_imu))
            ]

            util = pykitti.utils.read_calib_file(os.path.join(kitti.calib_path, 'calib_cam_to_cam.txt'))

            # Export
            save_static_transforms(bag, transforms, kitti.timestamps)
            save_dynamic_tf(bag, kitti, args.kitti_type, initial_time=None)
            save_imu_data(bag, kitti, imu_frame_id, imu_topic)
            save_gps_fix_data(bag, kitti, imu_frame_id, gps_fix_topic)
            save_gps_vel_data(bag, kitti, imu_frame_id, gps_vel_topic)
            for camera in cameras:
                save_camera_data(bag, args.kitti_type, kitti, util, bridge, camera=camera[0],
                                 camera_frame_id=camera[1], topic_name=camera[2], initial_time=None)
            save_velo_data(bag, kitti, velo_frame_id, velo_topic)

        finally:
            print("## OVERVIEW ##")
            print(bag)

    elif args.kitti_type.find("odom") != -1:

        if args.sequence == None:
            print("Sequence option is not given. It is mandatory for odometry dataset.")
            print("Usage for odometry dataset: kitti2bag {odom_color, odom_gray} [dir] -s <sequence>")
            sys.exit(1)

        bag_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.realpath(__file__))), 'resources',
            f"kitti_data_odometry_{args.kitti_type[5:]}_sequence_{args.sequence}"
        )

        bag = create_bag(bag_path)

        kitti = pykitti.odometry(args.dir, args.sequence)
        if not os.path.exists(kitti.sequence_path):
            print('Path {} does not exists. Exiting.'.format(kitti.sequence_path))
            sys.exit(1)

        kitti.load_calib()
        kitti.load_timestamps()

        if len(kitti.timestamps) == 0:
            print('Dataset is empty? Exiting.')
            sys.exit(1)

        if args.sequence in odometry_sequences[:11]:
            print("Odometry dataset sequence {} has ground truth information (poses).".format(args.sequence))
            kitti.load_poses()

        try:
            util = pykitti.utils.read_calib_file(os.path.join(args.dir, 'sequences', args.sequence, 'calib.txt'))
            current_epoch = (datetime.utcnow() - datetime(1970, 1, 1)).total_seconds()
            # Export
            if args.kitti_type.find("gray") != -1:
                used_cameras = cameras[:2]
            elif args.kitti_type.find("color") != -1:
                used_cameras = cameras[-2:]

            save_dynamic_tf(bag, kitti, args.kitti_type, initial_time=current_epoch)
            for camera in used_cameras:
                save_camera_data(bag, args.kitti_type, kitti, util, bridge, camera=camera[0],
                                 camera_frame_id=camera[1], topic=camera[2], initial_time=current_epoch)

        finally:
            print("## OVERVIEW ##")
            print(bag)


def main():
    run_kitti2bag()


if __name__ == '__main__':
    main()
