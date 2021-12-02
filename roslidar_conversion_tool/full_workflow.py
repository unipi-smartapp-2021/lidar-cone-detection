import numpy as np
from PIL import Image
import open3d as o3d
from utils import *

out_arr = read_pcd_and_filter("./pcd_outputs/cloud1.pcd")

"""Convert from X,Y,Z to spherical coordinates"""
az, el, r = cart2sph(out_arr[:, 1], out_arr[:, 2], out_arr[:, 0]) #convert to spherical coordinate

widht = 400
height = 600

"""Scale between widht and height"""
r, r_minmax = scale((0, 255), r)
az, az_minmax = scale((0, height), az)
el, el_minmax = scale((0, widht), el)

"""plt.scatter(x=el, y=az, c=r, cmap='viridis', s=5)
plt.show()"""

"""Create the image"""
create_image((widht, height), r, az, el, "img.png")

"""Read back the image"""
r, el, az = read_spherical_image("img.png")

"""Rescale back"""
r, _, = scale(r_minmax, r)
az, _, = scale(az_minmax, az)
el, _, = scale(el_minmax, el)

"""Convert back to Cartesian"""
x, y, z = sph2cart(az, el, r) # convert back to cartesian

"""Visualize the point cloud"""
cloud = np.column_stack((x, y, z))
visualize_cartesian(cloud)

