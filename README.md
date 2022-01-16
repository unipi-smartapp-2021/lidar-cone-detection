# Cone detection
Sensory data group that works with the cone detection for both stereocamera and lidar, the ```src``` directory is about the training and inference for the two yolov5 models, the ```sensory``` directory contains everything needed to let our work run on the simulator.
To clone the repository with its submodules:
```
git clone —recursive https://github.com/unipi-smartapp-2021/sensory-cone-detection
```
After that, follow all the readmes in each directory to understand how to work with both the model training and simulator detection or read the documentation files in the respective directory.
## Repository structure
```bash
📂sensory-cone-detection
├── 📂documentation
│   ├── 📄Sensory_ROS_documentaion.pdf
│   └── 📄Sensory_Recognition_documentation.pdf
├── 📄README.md
├── 📄requirements.txt
├── 📂sensory # this is the directory to let our work run on the simulator
│   ├── 📄CMakeLists.txt
│   ├── 📂launch
│   │   ├── 📄record_sensory.launch
│   │   └── 📄sensory_ws.launch
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
    │   ├── 📂interpolation_data
    │   │   └── 📄...
    │   ├── 📂interpolation_models
    │   │   └── 📄...
    │   ├── 📄lidar_cone_detection.ipynb
    │   ├── 📄lidar_weights.pt
    │   ├── 📂mathlab_exportlabel_script
    │   │   ├── 📄export_interpolation_label.asv
    │   │   ├── 📄export_interpolation_label.m
    │   │   ├── 📄export_label.asv
    │   │   └── 📄export_label.m
    │   ├── 📄output_fusion.py
    │   ├── 📄pcd2img.py
    │   ├── 📂post_process_data
    │   │   └── 📄...
    │   ├── 📄post_processing.py
    │   ├── 📄read_matlab_labels.py
    │   ├── 📄README.md
    │   ├── 📕ros_numpy
    │   ├── 📂stereo
    │   │   └── 📄...
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

Nice directory tree, how can i create it?
```bash
sudo apt install tree
```
Go into the directory you want and run ```tree```, copy the tree where you want and customize it to your needs.
