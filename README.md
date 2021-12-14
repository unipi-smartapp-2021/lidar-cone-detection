# Cone detection
Sensory data group that works with the cone detection for both stereocamera and lidar, the ```src``` directory is about the training and inference for the two yolov5 models, the ```sensory``` directory contains everything needed to let our work run on the simulator.
## Repository structure
```bash
📂sensory-cone-detection
├── 📄README.md
├── 📄requirements.txt
├── 📂sensory # this is the directory to let our work run on the simulator
│   ├── 📄CMakeLists.txt
│   ├── 📄package.xml
│   ├── 📂scripts
│   │   ├── 📂cameras
│   │   │   └── 📄image0.jpg
│   │   ├── 📄human_interaction.py
│   │   ├── 📄lidar.py
│   │   ├── 📂lidars
│   │   │   └── 📄image0.jpg
│   │   ├── 📄output_fusion.py
│   │   ├── 📄post_processing.py
│   │   ├── 📄rgb_camera.py
│   │   ├── 📄utilities.py
│   │   ├── 📂weights
│   │   │   ├── 📄camera_weights.pt
│   │   │   └── 📄lidar_weights.pt
│   │   └── 📕yolov5
│   ├── 📄setup.py
│   └── 📂src
│       └── 📂sensory
│           ├── 📄__init__.py
│           ├── 📄post_processing.py
│           └── 📄utilities.py
└── 📂src # this directory is about the models training and message conversion
    ├── 📂datasets
    │   ├── 📂example_dataset
    │   │   ├── 📂images
    │   │   │   └── 📄example_image.jpg
    │   │   └── 📂labels
    │   │       └── 📄example_label.txt
    │   └── 📄README.md
    ├── 📄__init__.py
    ├── 📂lidar
    │   ├── 📄conedataset_lidar.yaml
    │   ├── 📄convert_rosbag_to_pcd.py
    │   ├── 📄debug_box_draw.py
    │   ├── 📄draw_2d_box.py
    │   ├── 📄flatten_pcd.py
    │   ├── 📄full_workflow.py
    │   ├── 📄img2pcd.py
    │   ├── 📄__init__.py
    │   ├── 📄lidar_cone_detection.ipynb
    │   ├── 📄lidar_weights.pt
    │   ├── 📄output_fusion.py
    │   ├── 📄pcd2img.py
    │   ├── 📂post_process_data
    │   │   ├── 📄lidar.csv
    │   │   ├── 📄lidar_minmax.txt
    │   │   ├── 📄lidar.pcd
    │   │   ├── 📄lidar.png
    │   │   └── 📄lidar.txt
    │   ├── 📄post_processing.py
    │   ├── 📄read_matlab_labels.py
    │   ├── 📄README.md
    │   ├── 📕ros_numpy
    │   ├── 📂stereo
    │   │   ├── 📄depth_camera.png
    │   │   ├── 📄minmax.txt
    │   │   ├── 📄postprocessed.txt
    │   │   ├── 📄rgb_camera.png
    │   │   └── 📄stereo_yolo_boxes.txt
    │   └── 📄utils.py
    ├── 📄manage_dataset.py
    └── 📂stereocamera
        ├── 📄best.pt
        ├── 📄camera_cone_detection.ipynb
        ├── 📄camera_detection.jpg
        ├── 📄camera_detection_labels.jpg
        ├── 📄conedataset_camera.yaml
        └── 📄README.md
```
