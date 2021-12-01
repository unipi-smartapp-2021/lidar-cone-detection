# Cone detection
Sensory data group that works with the cone detection for both stereocamera and lidar.
## Repository structure
```bash
lidar-cone-detection
├── example_grayscale.jpg
├── README.md
└── src
    ├── lidar
    │   ├── convert_rosbag_to_pcd.py
    │   ├── lidar_bag.bag
    │   ├── pcd_outputs
    │       ├── lidar_pointcloud.pcd
    │   │   └── ...
    │   ├── README.md
    │   └── ros_numpy
    │       └── ...
    └── stereocamera
        ├── camera_cone_detection.ipynb
        ├── conedataset_grayscale.yaml
        ├── conedataset.yaml
        └── README.md
```
