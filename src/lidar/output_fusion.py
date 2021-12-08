import open3d as o3d
from matplotlib import pyplot as plt
import numpy as np
from scipy.spatial import transform as trsf
from utils import read_minmax_file

colored = o3d.io.read_image("./stereo/rgb_camera.png")
depth = o3d.io.read_image("./stereo/depth_camera.png")
rgbdimage = o3d.geometry.RGBDImage.create_from_color_and_depth(colored, depth)

r_minmax, az_minmax, el_minmax = read_minmax_file("./stereo/minmax.txt")

pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbdimage, o3d.camera.PinholeCameraIntrinsic(o3d.camera.PrimeSenseDefault))
# pcd = trsf.rotation(pcd, [[1,0,0,0], [0,-1,0,0], [0,0,-1,0], [0,0,0,1]])
# pcd.tranform([[1,0,0,0], [0,-1,0,0], [0,0,-1,0], [0,0,0,1]])
o3d.visualization.draw_geometries([pcd])