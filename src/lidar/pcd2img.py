import argparse
import sys
from utils import read_pcd_and_filter, cart2sph, min_max_scale, create_image, from_pcd_to_image


def parse_arguments(known=False):
    parser = argparse.ArgumentParser()
    parser.add_argument('--pcd_file', type=str, default="", help='the path to the .pcd file es cloud0.pcd')
    parser.add_argument('--image_name', type=str, default="", help='the name of the image with extension es img.png')
    parser.add_argument('--output', type=str, default=".", help='path where the images are going to be stored es ./img/')
    opt = parser.parse_known_args()[0] if known else parser.parse_args()
    return opt


if __name__ == "__main__":
    opt_parser = parse_arguments()
    if len(sys.argv) <= 5:
        print("Please insert the --input, --image_name and --output arguments")
        exit(0)
    pcd_file__path = opt_parser.pcd_file
    image_name = opt_parser.image_name
    output__path = opt_parser.output
    from_pcd_to_image(pcd_file__path, output__path, image_name)


