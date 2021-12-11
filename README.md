# Cone detection
Sensory data group that works with the cone detection for both stereocamera and lidar, the ```src``` directory is about the training and inference for the two yolov5 models, the ```smartapp``` directory contains everything needed to let our work run on the simulator.
## Repository structure
```bash
📂lidar-cone-detection
├── 📄README.md
├── 📄requirements.txt
├── 📂smartapp
│   ├── 📄CMakeLists.txt
│   ├── 📄package.xml
│   ├── 📂scripts
│   │   ├── 📂cameras
│   │   │   └── 📄image0.jpg
│   │   ├── 📄camera_weights.pt
│   │   ├── 📂lidars
│   │   │   └── 📄image0.jpg
│   │   ├── 📄lidar_weights.pt
│   │   ├── 📄subscriber_lidar.py
│   │   ├── 📄subscriber_rgb_camera.py
│   │   └── 📄utilities.py
│   └── 📄setup.py
└── 📂src
    ├─ 📂datasets
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
    │   ├── 📄lidar_cone_detection.ipynb
    │   ├── 📄lidar_weights.pt
    │   ├── 📄pcd2img.py
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
