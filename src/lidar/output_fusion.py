import open3d as o3d
from matplotlib import pyplot as plt
import numpy as np

colored = o3d.io.read_image("./stereo/rgb_camera.png")
depth = o3d.io.read_image("./stereo/depth_camera.png")
rgbdimage = o3d.geometry.RGBDImage.create_from_color_and_depth(colored, depth)
plt.imshow(rgbdimage.color)
# plt.imshow(rgbdimage.depth)
# plt.show()
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbdimage, o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))

# pcd.tranform([[1,0,0,0], [0,-1,0,0], [0,0,-1,0], [0,0,0,1]])
print(pcd)
o3d.visualization.draw_geometries([pcd])