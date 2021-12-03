# Cone detection
Sensory data group that works with the cone detection for both stereocamera and lidar.
## Repository structure
```bash
📂lidar-cone-detection
├── 📄README.md
├── 📄requirements.txt
└── 📂src
    ├── 📄cone_detection.py
    ├── 📂datasets
    │   ├── 📂example_dataset
    │   │   ├── 📂images
    │   │   │   └── 📄example_image.jpg
    │   │   └── 📂labels
    │   │       └── 📄example_label.txt
    │   └── 📄README.md
    ├── 📂lidar
    │   ├── 📄conedataset_lidar.yaml
    │   ├── 📄convert_rosbag_to_pcd.py
    │   ├── 📄full_workflow.py
    │   ├── 📄img2pcd.py
    │   ├── 📄lidar_bag.bag
    │   ├── 📄lidar_cone_detection.ipynb
    │   ├── 📄pcd2img.py
    │   ├── 📂pcd_outputs
    │   │   ├── 📄cloud0.pcd
    │   │   └── 📄...
    │   ├── 📄README.md
    │   ├── 📕ros_numpy
    │   └── 📄utils.py
    ├── 📄manage_dataset.py
    └── 📂stereocamera
        ├── 📄best.pt
        ├── 📄camera_cone_detection.ipynb
        ├── 📄camera_detection.jpg
        ├── 📄camera_detection_labels.jpg
        ├── 📄conedataset_camera.yaml
        └── 📄README.md
```
