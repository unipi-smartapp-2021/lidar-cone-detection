import torch
# import pandas
import argparse


def parse_arguments(known=False):
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, default="lidar-cone-detection/src/stereocamera/best.pt",
                        help='path where the dataset is going to be stored')
    parser.add_argument('--path', default="", help='images on which the model will work')
    parser.add_argument('--show', action='store_true', help='show the inference result')
    parser.add_argument('--save', action='store_true', help='save the inference result')
    parser.add_argument('--pandas_show', action='store_true', help='show the results by using pandas')
    opt = parser.parse_known_args()[0] if known else parser.parse_args()
    return opt


if __name__ == "__main__":
    # Model
    opt_parser = parse_arguments()
    if opt_parser.weights is not "":
        model = torch.hub.load('ultralytics/yolov5', 'custom', path=opt_parser.weights)
    else:
        model = None

    # Image
    img = opt_parser.path

    # Inference
    results = model(img)
    if opt_parser.show():
        results.show()

    if opt_parser.save():
        results.save()

    if opt_parser.pandas_show():
        print()
