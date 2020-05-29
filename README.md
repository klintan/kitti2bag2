# kitti2bag2

Convert [KITTI](http://www.cvlibs.net/datasets/kitti/index.php) dataset to ROS2 bag file the easy way!

## Installation
Right now this relies on the latest master of ROS2.

It is very easy! On the machine with ROS2 installed, just run
```bash
pip install kitti2bag2
```

## Usage

One example is better then thousand words so here it is

```bash
$ wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0002/2011_09_26_drive_0002_sync.zip
$ wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_calib.zip
$ unzip 2011_09_26_drive_0002_sync.zip
$ unzip 2011_09_26_calib.zip
$ kitti2bag2 -t 2011_09_26 -r 0002 raw_synced .
Exporting static transformations
Exporting time dependent transformations
Exporting IMU
Exporting camera 0
100% (77 of 77) |##########################| Elapsed Time: 0:00:00 Time: 0:00:00
Exporting camera 1
100% (77 of 77) |##########################| Elapsed Time: 0:00:00 Time: 0:00:00
Exporting camera 2
100% (77 of 77) |##########################| Elapsed Time: 0:00:01 Time: 0:00:01
Exporting camera 3
100% (77 of 77) |##########################| Elapsed Time: 0:00:01 Time: 0:00:01
Exporting velodyne data
100% (77 of 77) |##########################| Elapsed Time: 0:00:15 Time: 0:00:15
## OVERVIEW ##
path:        kitti_2011_09_26_drive_0002_synced.bag
version:     2.0
duration:    7.8s
start:       Sep 26 2011 13:02:44.33 (1317042164.33)
end:         Sep 26 2011 13:02:52.16 (1317042172.16)
size:        417.2 MB
messages:    1078
compression: none [308/308 chunks]
types:       geometry_msgs/TwistStamped [98d34b0043a2093cf9d9345ab6eef12e]
             sensor_msgs/CameraInfo     [c9a58c1b0b154e0e6da7578cb991d214]
             sensor_msgs/Image          [060021388200f6f0f447d0fcd9c64743]
             sensor_msgs/Imu            [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/NavSatFix      [2d3a8cd499b9b4a0249fb98fd05cfa48]
             sensor_msgs/PointCloud2    [1158d486dd51d683ce2f1be655c3c181]
             tf2_msgs/TFMessage         [94810edda583a504dfda3829e70d7eec]
topics:      /kitti/camera_color_left/camera_info    77 msgs    : sensor_msgs/CameraInfo    
             /kitti/camera_color_left/image_raw      77 msgs    : sensor_msgs/Image         
             /kitti/camera_color_right/camera_info   77 msgs    : sensor_msgs/CameraInfo    
             /kitti/camera_color_right/image_raw     77 msgs    : sensor_msgs/Image         
             /kitti/camera_gray_left/camera_info     77 msgs    : sensor_msgs/CameraInfo    
             /kitti/camera_gray_left/image_raw       77 msgs    : sensor_msgs/Image         
             /kitti/camera_gray_right/camera_info    77 msgs    : sensor_msgs/CameraInfo    
             /kitti/camera_gray_right/image_raw      77 msgs    : sensor_msgs/Image         
             /kitti/oxts/gps/fix                     77 msgs    : sensor_msgs/NavSatFix     
             /kitti/oxts/gps/vel                     77 msgs    : geometry_msgs/TwistStamped
             /kitti/oxts/imu                         77 msgs    : sensor_msgs/Imu           
             /kitti/velo/pointcloud                  77 msgs    : sensor_msgs/PointCloud2   
             /tf                                     77 msgs    : tf2_msgs/TFMessage        
             /tf_static                              77 msgs    : tf2_msgs/TFMessage
```


That's it. You'll now have a folder called `kitti_2011_09_26_drive_0002_sync` that contains your data.

Other source files can be found at [KITTI raw data](http://www.cvlibs.net/datasets/kitti/raw_data.php) page.


## TODOs

Feature request list:
 * make [URDF](http://wiki.ros.org/urdf) of a car so transformations between frames are easily done by ROS2 itself.
 * deal with tracklets
 * support for unsynced+unrectified version
 * provide documentation via [ROS wiki](wiki.ros.org)
 * provide simple GUI
 * distribute available bagfiles publically (Marvhub)
 * export only subset of sensors

## Contributions

- Thanks to the work of @jnitsch, _kitti2bag_ can now export velodyne laser data and dynamic _tf_ transformations. 
- Thanks to @emreay-, this tool can now convert odometry datasets too. Thank you both!

## Related works
 * [kitti2bag](https://github.com/tomas789/kitti2bag) same as this repo but for Ros1.
 * [pykitti](https://github.com/utiasSTARS/pykitti) is very simple library for dealing with KITTI dataset in python. 
 * [kitti_player](https://github.com/tomas789/kitti_player) allows to play dataset directly. No bag file needed. I found difficult to get it work. Some bug fixed can be found in [my fork of kitti_player](https://github.com/tomas789/kitti_player) but still not good enough.
