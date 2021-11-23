# lidar-cone-detection
Sensory data group that works with the lidar cone detection.
## Setup
Before you start working with the project, remember to clone this repository.
```
!git clone "https://github.com/unipi-smartapp-2021/lidar-cone-detection"
```
And the official yolov5 repository too.
```
!git clone "https://github.com/ultralytics/yolov5"
```
Upload the dataset in the apposite directories following what is suggested in https://github.com/ultralytics/yolov5/wiki/Train-Custom-Data, section 1.3.

**Modify the .yaml file to your needs**, this is vital to work with yolov5 on a custom dataset.
```python
# Train/val/test sets as 1) dir: path/to/imgs, 2) file: path/to/imgs.txt, or 3) list: [path/to/imgs1, path/to/imgs2, ..]
path: datasets # dataset root dir
train: train.txt # train images (relative to 'path')
val: validation.txt # val images (relative to 'path')
test: test.txt # test images (optional)
# Classes
nc: 4 # number of classes
names: ['big','orange','yellow','blue'] # class names
```
Then you can find the weights of the various models (base, small, large etc.) on https://github.com/ultralytics/yolov5/releases, the following is an example of importing them:
```python
import requests
url = 'https://github.com/ultralytics/yolov5/releases/download/v6.0/yolov5s.pt'
r = requests.get(url, allow_redirects=True)
open('yolov5s.pt', 'wb').write(r.content)
```
## Train
To train the model we absolutely suggest you to follow this guide https://github.com/ultralytics/yolov5/blob/master/tutorial.ipynb.
```
!python yolov5/train.py --img 640 --batch 16 --epochs 10 --data lidar-cone-detection/conedataset.yaml --weights yolov5s.pt
```
Use ```--cache``` to cache the train and validation sets before starting the training and use ```--project``` to specify where to save the results (the main guide suggests to save them in runs/train).
## Inference
The inference on labeled data (the test set specified on the .yaml file) is done by using val.py.
```
!python yolov5/val.py --weights best.pt --img 640 --conf 0.5 --data lidar-cone-detection/conedataset.yaml --task test
```
The weights should be in /runs/train/exp/weights if you just did the training, otherwise give the acutal path. We suggest, as for the training, using ```--project``` to save the results in runs/test. ```--task test``` **is necessary to infere on the test set otherwise it will work on the validation set**.

If you're using non labeled data, then use detect.py.
```
!python yolov5/detect.py --weights best.pt --img 640 --conf 0.5 --source datasets/test.txt --project runs/detect
```
