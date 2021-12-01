# Cone detection
Sensory data group that works with the cone detection for both stereocamera and lidar.
## Repository structure
```bash
lidar-cone-detection
├── example_grayscale.jpg
├── README.md
└── src
    ├── Lidar
    │   ├── convert_rosbag_to_pcd.py
    │   ├── lidar_bag.bag
    │   ├── pcd_outputs
    │   │   ├── cloud0.pcd
    │   │   ├── cloud1.pcd
    │   │   ├── cloud2.pcd
    │   │   ├── cloud3.pcd
    │   │   ├── cloud4.pcd
    │   │   ├── cloud5.pcd
    │   │   ├── cloud6.pcd
    │   │   ├── cloud7.pcd
    │   │   ├── cloud8.pcd
    │   │   ├── cloud9.pcd
    │   │   └── lidar_pointcloud0.pcd
    │   ├── README.md
    │   └── ros_numpy
    │       ├── CMakeLists.txt
    │       ├── LICENSE
    │       ├── package.xml
    │       ├── README.md
    │       ├── setup.py
    │       ├── src
    │       │   └── ros_numpy
    │       │       ├── geometry.py
    │       │       ├── image.py
    │       │       ├── __init__.py
    │       │       ├── numpy_msg.py
    │       │       ├── occupancy_grid.py
    │       │       ├── point_cloud2.py
    │       │       ├── __pycache__
    │       │       │   ├── geometry.cpython-37.pyc
    │       │       │   ├── image.cpython-37.pyc
    │       │       │   ├── __init__.cpython-37.pyc
    │       │       │   ├── numpy_msg.cpython-37.pyc
    │       │       │   ├── occupancy_grid.cpython-37.pyc
    │       │       │   ├── point_cloud2.cpython-37.pyc
    │       │       │   └── registry.cpython-37.pyc
    │       │       └── registry.py
    │       └── test
    │           ├── test_geometry.py
    │           ├── test_images.py
    │           ├── test_occupancygrids.py
    │           └── test_pointclouds.py
    └── StereoCamera
        ├── CameraDetection.ipynb
        ├── conedataset_grayscale.yaml
        ├── conedataset.yaml
        └── README.md
```
