import numpy as np
from PIL import Image
#import open3d as o3d
#import pandas as pd
import cv2
"""
Utils file which contains all the computation and formulas for
managing point clouds.
"""


'''def visualize_cartesian(arr):
    """It save the numpy array into a Point cloud and visualize it"""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(arr)
    o3d.io.write_point_cloud("./sync.ply", pcd)
    pcd_load = o3d.io.read_point_cloud("./sync.ply")
    o3d.visualization.draw_geometries([pcd_load])'''

'''
def save_numpy_as_pcd(arr, file_path):
    """From numpy array X,Y,Z save it as pcd file"""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(arr)
    o3d.io.write_point_cloud(file_path, pcd)'''


'''def visualize_pcd_file(file_path):
    """From .pcd file visualize it in a 3D view"""
    pcd_load = o3d.io.read_point_cloud(file_path)
    o3d.visualization.draw_geometries([pcd_load])'''




def cart2sph(x, y, z):
    """Conversion from cartesian coordinates in spherical coordinates"""
    hxy = np.hypot(x, y)
    r = np.hypot(hxy, z)
    el = np.arctan2(z, hxy)
    az = np.arctan2(y, x)
    return az, el, r


'''def sph2cart(az, el, r):
    """Conversion from spherical coordinate in cartesian coordinate"""
    rcos_theta = r * np.cos(el)
    x = rcos_theta * np.cos(az)
    y = rcos_theta * np.sin(az)
    z = r * np.sin(el)
    return x, y, z'''


def scale(min_max, data):
    """Method to scale in a specific range of values trought minmax scaling"""
    min = np.min(data)
    max = np.max(data)
    data_std = ((data - min) / (max - min))
    data_scaled = data_std * (min_max[1]-min_max[0]) + min_max[0]
    return data_scaled, (min, max)


'''def read_pcd_and_filter(path):
    """Read the pcd file and filter by x values and height values."""
    cloud = o3d.io.read_point_cloud(path) # Read the point cloud file
    out_arr = np.asarray(cloud.points)
    out_arr = np.asarray(list(filter(lambda x: (0 < x[0] < 40) and (-0.5 < x[2] < 0.90), out_arr)))
    return out_arr'''


def create_image(img_size, r, az, el, output_path='a.png'):
    """Create an image from r, azimuth and elevation"""
    image = np.ones([img_size[0], img_size[1]], np.uint8) * 255
    r = r.astype(int)
    az = az.astype(int)
    el = el.astype(int)
    for i in range(r.shape[0]):
        image[el[i], az[i]] = r[i]
    #cv2.imwrite(output_path, image)
    return image


'''def read_spherical_image(path):
    """Read the spherical image and return the r, elevation and azimuth"""
    image = Image.open(path)
    image_array = np.array(image)
    a = np.argwhere(image_array < 255)
    r = np.array([image_array[ele[0], ele[1]] for ele in a])
    el = np.array(a[:, 0])
    az = np.array(a[:, 1])
    return r, el, az'''


'''def read_minmax_file(path):
    """read the minmax file wich contains the previous min and max to let the conversion back to cartesian."""
    f = open(path, "r")
    values = f.read().split(" ")
    return (float(values[0]), float(values[1])), (float(values[2]), float(values[3])), (float(values[4]), float(values[5]))'''

def from_matrix_to_image(matrix : np.ndarray, img_path='./lidar_out_parsed/', img_name='lidar_frame_out.png', out_img= False):
    """Method to convert a .pcd file to an image with the metadata file contanining the min and max
    for r,az,el"""
    out_arr = matrix #read_pcd_and_filter(pcd_file_path)
    out_arr = np.asarray(list(filter(lambda x: (0 < x[0] < 40) and (-0.5 < x[2] < 0.90), out_arr)))

    """Convert from X,Y,Z to spherical coordinates"""
    az, el, r = cart2sph(out_arr[:, 1], out_arr[:, 2], out_arr[:, 0])  # convert to spherical coordinate

    widht = 400
    height = 600

    """Scale between widht and height"""
    r, r_minmax = scale((0, 255), r) 
    az, az_minmax = scale((0, height - 1), az)
    el, el_minmax = scale((0, widht - 1), el)

    """Create the image"""
    img = create_image((widht, height), r, az, el)
    file_path = img_path+img_name
    if out_img:
        cv2.imwrite(file_path, img)
    return img

        

'''
def from_pcd_to_image(pcd_file_path, img_path, img_name):
    """Method to convert a .pcd file to an image with the metadata file contanining the min and max
    for r,az,el"""
    out_arr = read_pcd_and_filter(pcd_file_path)

    """Convert from X,Y,Z to spherical coordinates"""
    az, el, r = cart2sph(out_arr[:, 1], out_arr[:, 2], out_arr[:, 0])  # convert to spherical coordinate

    widht = 400
    height = 600

    """Scale between widht and height"""
    r, r_minmax = scale((0, 255), r)
    az, az_minmax = scale((0, height), az)
    el, el_minmax = scale((0, widht), el)

    """Save minumum in file"""
    f = open(img_path+img_name.split(".")[0]+"_minmax.txt", "w")
    f.write(str(r_minmax[0])+" "+str(r_minmax[1]))
    f.write(" "+str(az_minmax[0])+" "+str(az_minmax[1]))
    f.write(" "+str(el_minmax[0]) + " " + str(el_minmax[1]))
    f.close()

    """Create the image"""
    create_image((widht, height), r, az, el, img_path+img_name)'''
