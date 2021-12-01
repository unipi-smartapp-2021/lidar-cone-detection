import shutil
import os
import argparse
import cv2
from glob import glob
from tqdm import tqdm
import numpy as np


def parse_arguments(known=False):
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', type=str, default="", help='dataset path')
    parser.add_argument('--path', type=str, default="datasets/",
                        help='path where the dataset is going to be stored')
    parser.add_argument('--m', type=str, default="move", help='move or copy the dataset')
    parser.add_argument('--grayscale', action='store_true', help='convert the dataset images to grayscale')
    parser.add_argument('--split', nargs='+', type=float, default=[0.8, 0.1, 0.1],
                        help='split the dataset into training, validation and test sets')
    opt = parser.parse_known_args()[0] if known else parser.parse_args()
    return opt


def convert_to_grayscale(images_list, dataset_path) -> None:
    # Convert images ro grayscale
    for image in tqdm(images_list):
        img = cv2.imread(image)
        grayscale_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cv2.imwrite(image, grayscale_image)

    # Fix the labels
    labels_list = glob(dataset_path + "labels/*.txt")
    for label in labels_list:
        with open(label, "r+") as datafile:
            records = datafile.readlines()
            datafile.seek(0)
            for record in records:
                datafile.write("0" + record[1:])


def split_set(dataset: list,
              tr: float = 0.8,
              val: float = 0.1,
              ts: float = 0.1,
              shuffle: bool = True) -> (list, list, list):
    if tr+val+ts != 1:
        raise ValueError("Train, validation and test partition not allowed with such splits")

    dataset_size = len(dataset)
    if shuffle:
        np.random.shuffle(dataset)

    tr_size = int(tr * dataset_size)
    val_size = int(val * dataset_size + tr_size)

    tr_set = list(dataset[:tr_size])
    val_set = list(dataset[tr_size:val_size])
    ts_set = list(dataset[val_size:])
    return tr_set, val_set, ts_set


def write_image_file(file_path: str, dataset: list) -> None:
    with open(file_path, "w") as datafile:
        for image in dataset:
            datafile.write("{0}\n".format(image))


if __name__ == "__main__":
    opt_parser = parse_arguments()
    source = opt_parser.dataset
    target = opt_parser.path
    file_names = os.listdir(source)

    # Create the dataset paths if they don't exist
    if not os.path.exists(target):
        os.mkdir(target)
        os.mkdir(target + "/images")
        os.mkdir(target + "/labels")

    # Move or copy each file from the previous path
    for file_name in file_names:
        if file_name.endswith(".jpg") or file_name.endswith(".png"):
            target_file = target + "/images"
        else:
            target_file = target + "/labels"

        if opt_parser.m == "move":
            shutil.move(os.path.join(source, file_name), target_file)
        elif opt_parser.m == "copy":
            shutil.copy(os.path.join(source, file_name), target_file)
        else:
            raise ValueError("Passed an incorrect value for the --m argument, use \"move\" or \"copy\"")

    images = glob(target+"/images/*.jpg")

    # Convert images to grayscale and fix the labels
    if opt_parser.grayscale:
        convert_to_grayscale(images, target)

    # Split the dataset
    tr_images, val_images, ts_images = split_set(images, tr=opt_parser.split[0], val=opt_parser.split[1],
                                                 ts=opt_parser.split[2])

    # Save the train, validation and test sets on their respective files
    write_image_file(target + "/train.txt", tr_images)
    write_image_file(target + "/validation.txt", val_images)
    write_image_file(target + "/test.txt", ts_images)
