import numpy as np
from PIL import Image
import open3d as o3d
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import cv2
"""
Utils file which contains all the computation and formulas for
managing point clouds.
"""

def convert_numpy_to_rosMultiArr(matrix:np.ndarray):
    mat = Float32MultiArray()
    mat.layout.dim.append(MultiArrayDimension())
    mat.layout.dim.append(MultiArrayDimension())
    mat.layout.dim[0].label = "height"
    mat.layout.dim[1].label = "width"
    mat.layout.dim[0].size = matrix.shape[0]
    mat.layout.dim[1].size = matrix.shape[1]
    
    matrix_flat = matrix.flatten().tolist()
    mat.data = matrix_flat
    return mat

def visualization(script_path:str, results):
    results.save(script_path)
    if len(results.files)>0:
        tmp = script_path + results.files[0]
        img = cv2.imread(tmp)
        cv2.imshow("Image window", img) #only for visualization purpose   
        cv2.waitKey(1) #only for visualization purpose
    else:
        cv2.imshow("Image window", results.imgs) #only for visualization purpose   
        cv2.waitKey(1)

def save_numpy_as_pcd(arr, file_path):
    """From numpy array X,Y,Z save it as pcd file"""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(arr)
    o3d.io.write_point_cloud(file_path, pcd)


def writeFile(self, r_minmax, theta_minmax, phi_minmax):
        f = open(self.script_path + "/minmax.txt", "w")
        f.write(str(r_minmax[0])+" "+str(r_minmax[1]))
        f.write(" "+str(theta_minmax[0])+" "+str(theta_minmax[1]))
        f.write(" "+str(phi_minmax[0]) + " " + str(phi_minmax[1]))
        f.close()



def cart2sph(x, y, z):
    """Conversion from cartesian coordinates in spherical coordinates"""
    hxy = np.hypot(x, y)
    r = np.hypot(hxy, z)
    el = np.arctan2(z, hxy)
    az = np.arctan2(y, x)
    return az, el, r


def min_max_scale(min_max, data, max=None, min=None):
    """
    Method to scale in a specific range of values trought minmax scaling
    :param min_max: tuple (min, max)  describing the final min e max of the scaled data
    :param data: np.array to scale
    :param min: Optional. Set a minimum value instead of getting it from data
    :param max: Optional. Set a minimum value instead of getting it from data
    :return:
    """
    if min is None:
        min = np.min(data)
    if max is None:
        max = np.max(data)

    data_std = ((data - min) / (max - min))
    data_scaled = data_std * (min_max[1]-min_max[0]) + min_max[0]
    return data_scaled, (min, max)    


def scale(min_max, data):
    """Method to scale in a specific range of values trought minmax scaling"""
    min = np.min(data)
    max = np.max(data)
    data_std = ((data - min) / (max - min))
    data_scaled = data_std * (min_max[1]-min_max[0]) + min_max[0]
    return data_scaled, (min, max)


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


def read_spherical_image(image_array: np.ndarray):
    """Read the spherical image and return the r, elevation and azimuth"""
    a = np.argwhere(image_array < 255)
    r = np.array([image_array[ele[0], ele[1]] for ele in a])
    el = np.array(a[:, 0])
    az = np.array(a[:, 1])
    return r, el, az    


def from_matrix_to_image(matrix : np.ndarray, img_path='./lidar_out_parsed/', img_name='lidar_frame_out.png', out_img= False):
    """Method to convert a .pcd file to an image with the metadata file contanining the min and max
    for r,az,el"""
    out_arr = matrix #read_pcd_and_filter(pcd_file_path)
    out_arr = np.asarray(list(filter(lambda x: (-0.5 < x[2] < 0.90), out_arr)))

    """Convert from X,Y,Z to spherical coordinates"""
    az, el, r = cart2sph(out_arr[:, 1], out_arr[:, 2], out_arr[:, 0])  # convert to spherical coordinate

    widht = 480
    height = 480

    """Scale between widht and height"""
    r, r_minmax = scale((0, 255), r) 
    az, az_minmax = scale((0, height - 1), az)
    el, el_minmax = scale((0, widht - 1), el)

    """Save minumum in file"""
    '''f = open(img_path+img_name.split(".")[0]+"_minmax.txt", "w")
    f.write(str(r_minmax[0])+" "+str(r_minmax[1]))
    f.write(" "+str(az_minmax[0])+" "+str(az_minmax[1]))
    f.write(" "+str(el_minmax[0]) + " " + str(el_minmax[1]))
    f.close()'''

    """Create the image"""
    img = create_image((widht, height), r, az, el)
    file_path = img_path+img_name
    if out_img:
        cv2.imwrite(file_path, img)
    return img
