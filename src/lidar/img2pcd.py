import argparse
import sys
from utils import *


def parse_arguments(known=False):
    parser = argparse.ArgumentParser()
    parser.add_argument('--image_folder', type=str, default="",
                        help='the path where the images and metadata minmax files are stored')
    parser.add_argument('--filename_name', type=str, default="",
                        help='the name of the image with extension es img.png')
    parser.add_argument('--output', type=str, default=".",
                        help='path file where the pointcloud is going to be stored es ./pcds/cloud.pcd')
    parser.add_argument('--visualize', type=bool, default=False,
                        help='visualize the output in a 3D space')
    opt = parser.parse_known_args()[0] if known else parser.parse_args()
    return opt


if __name__ == "__main__":
    opt_parser = parse_arguments()
    if len(sys.argv) <= 5:
        print("Please insert the --input, --image_name and --output arguments")
        exit(0)
    img_file__path = opt_parser.image_folder
    image_name = opt_parser.filename_name
    output__path = opt_parser.output

    """Read back the image"""
    r, el, az = read_spherical_image(img_file__path+image_name)

    r_minmax, az_minmax, el_minmax = read_minmax_file(img_file__path+(image_name.split(".")[0])+"_minmax.txt")

    """Rescale back"""
    r, _, = min_max_scale(r_minmax, r)
    az, _, = min_max_scale(az_minmax, az)
    el, _, = min_max_scale(el_minmax, el)

    """Convert back to Cartesian"""
    x, y, z = sph2cart(az, el, r) # convert back to cartesian

    """Visualize the point cloud"""
    cloud = np.column_stack((x, y, z))
    save_numpy_as_pcd(cloud, output__path)
    if opt_parser.visualize:
        visualize_pcd_file(output__path)
