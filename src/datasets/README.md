# StereoCamera and lidar datasets
This subdirectory should follow what is suggested in https://github.com/ultralytics/yolov5/wiki/Train-Custom-Data, section 1.3, **this is just an example of how it should look after you've loaded your dataset**.
## Managing the dataset
Upload the dataset in the apposite directories or use manage_dataset.py to build the directories for you.
- _--dataset_, your dataset path.
- _--path_, where the dataset is going to be stored.
- _--m_, move or copy the dataset, the default option is "move".
- _--grayscale_, convert the dataset images to grayscale, the default option is False.
- _--split_, split the dataset into training, validation and test sets, default is [0.8, 0.1, 0.1].
```python
python manage_dataset.py --dataset your_dataset_directory --path datasets/your_dataset
```
The command will also take care of converting the images into grayscale ones and to split them into train, validation and test sets, then it will create the respective files for the sets. Note that the test set is not mandatory.
Remember to delete your old dataset directory only after you've moved or copied the files, then you're ready to work.

**Reminder for you:** the datasets we worked on are on a shared drive since they are too big to be uploaded here on github, so if you want to try the notebook you should build your own dataset (or you can ask us to share the drive, we're not that strict).
