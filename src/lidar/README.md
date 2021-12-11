# Ros Lidar Converter to PCD
Ros provide data inside a ros bag file format *.bag*. In this file we
can find different messages from different topics.
## Dependencies
```
pip install py3rosmsgs
pip install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag
```
## Solve an issue from pypcd
The pypcd package hasn't been updated in a while, if you're working on Colab use the following command to solve an issue about an import, otherwise you have to manually change ```import cStringIO as sio``` into ```from io import StringIO as sio```, then everything should work fine.
```
sed -i "s/import cStringIO as sio/from io import StringIO as sio/g" ../usr/local/lib/python3.7/dist-packages/pypcd/pypcd.py
```
### The lidar messages
The lidar messages coming from the topic **/carla/ego_vehicle/lidar** are
encoded in the PointCloud2 format, where the X,Y,Z are converted in a 
**uint8 list** in a compact definition.

#### Raw Message Definition
```
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the "fields" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points
```

#### Compact Definition
```
std_msgs/Header header
uint32 height
uint32 width
sensor_msgs/PointField[] fields
bool is_bigendian
uint32 point_step
uint32 row_step
uint8[] data
bool is_dense
```

### Tool description
The aim of this tool is to:
- read a .bag file
- filter by lidar topic
- convert the compact definition in Raw one
- convert the datas in numpy array
- convert the numpy array in .pcd file

Where the **pcd** extension mean Point Cloud Data and thi is the
default format for point cloud data. Ready to be labeled for classification.
