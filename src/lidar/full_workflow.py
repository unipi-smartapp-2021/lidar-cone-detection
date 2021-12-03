import argparse
from utils import *


def parse_arguments(known=False):
    parser = argparse.ArgumentParser()
    parser.add_argument('--pcd_path', type=str, default="", help='pcd that need to be converted')
    parser.add_argument('--image_path', type=str, default="img.png", help='where to save the image')
    parser.add_argument('--w', type=int, default=400, help='image width')
    parser.add_argument('--h', type=int, default=600, help='image height')
    parser.add_argument('--show', action='store_true', help='show the pcd')
    opt = parser.parse_known_args()[0] if known else parser.parse_args()
    return opt


if __name__ == "__main__":
    opt_parser = parse_arguments()
    out_arr = read_pcd_and_filter(opt_parser.pcd_path)

    # Convert from X,Y,Z to spherical coordinates
    az, el, r = cart2sph(out_arr[:, 1], out_arr[:, 2], out_arr[:, 0])

    widht = opt_parser.w
    height = opt_parser.h

    # Scale between widht and height
    r, r_minmax = scale((0, 255), r)
    az, az_minmax = scale((0, height), az)
    el, el_minmax = scale((0, widht), el)

    # Create the image
    create_image((widht, height), r, az, el, opt_parser.image_path)

    # Read back the image
    r, el, az = read_spherical_image(opt_parser.image_path)

    # Rescale back
    r, _, = scale(r_minmax, r)
    az, _, = scale(az_minmax, az)
    el, _, = scale(el_minmax, el)

    # Convert back to Cartesian
    x, y, z = sph2cart(az, el, r)

    # Visualize the point cloud
    if opt_parser.show:
        cloud = np.column_stack((x, y, z))
        visualize_cartesian(cloud)
