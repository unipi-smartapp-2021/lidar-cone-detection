from sensory.utilities import *
#from utilities import *
import pandas as pd
import math
from matplotlib import pyplot as plt
from scipy.signal import find_peaks
import scipy.stats as stats
import numpy as np


class OutputTypes:
    CENTER = "center"
    BOXES = "boxes"
    output_dim = {CENTER: 3, BOXES: 6}

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

def post_processing(raw_pcd, polar_minmaxes, yolo_boxes, img_df, image_size, rmax=255, output_type=OutputTypes.CENTER, first_quantile=0.7, second_quantile=0.6):
    """Post process method to convert the output from the lidar model to spherical coordinate of the cones.
    :param raw_pcd: the numpy array of the point cloud X,Y,Z
    :param polar_minmaxes: the touple with: r_minmax, theta_minmax, phi_minmax
    :param yolo_boxes: the pandas dataframe obtained from the yolo lidar model with (xmin,ymin,xmax,ymax)
    :param img_df: the dataframe of the points in spherical coordinates obtained from the image with (theta,phi,r)
    :param image_size: tuple containing image_width and image_height
    :param rmax: the maximum value for the distance default is 255
    :param output_type: two types of output format:"center","boxes".
    :param first_quantile: the first threshold for outlier detection
    :param second_quantile: the second threshold for outlier detection. It must be smaller than first_quantile
    :return: "center" returns a numpy array with (phi,theta,r) of the centers of the cones.
             "boxes"  return a numpy array with (phi_min, phi_max, theta_min, theta_max, r_min, r_max) describing the boxes containing the cones.
    """
    yolo_boxes["r_mins"] = rmax                                                 #setting the rmax value
    yolo_boxes["r_maxs"] = rmax                                                 #setting the rmax value
    r_minmax, theta_minmax, phi_minmax = polar_minmaxes                         #unzip polar minmaxes
    img_width, img_height = image_size                                          #getting the width and height of the image

    for index, box in yolo_boxes.iterrows():                                    #interate over the boxes found by the lidar model
        theta_min = math.floor(box["xmin"])
        theta_max = math.ceil(box["xmax"])
        phi_min = math.floor(box["ymin"])
        phi_max = math.ceil(box["ymax"])
        # taking only the inner points of the boxes
        r_values = img_df[((img_df['phi'] >= phi_min) & (img_df['phi'] <= phi_max))
                         & ((img_df['theta'] >= theta_min) & (img_df['theta'] <= theta_max))]
        yolo_boxes.loc[index, 'r_mins'] = r_values['r'].min() if r_values.shape[0] > 0 else rmax #rmax points are the background
        yolo_boxes.loc[index, 'r_maxs'] = r_values['r'].max() if r_values.shape[0] > 0 else rmax #rmax points are the background

    yolo_boxes = yolo_boxes[(yolo_boxes['r_mins'] != rmax) & (yolo_boxes['r_maxs'] != rmax)] # remove the background

    """Rescaling from spherical coordinate in the image widht and height in the original spherical coordinate scale"""
    yolo_boxes["theta_mins"], _ = min_max_scale(theta_minmax, yolo_boxes["xmin"], max=img_width-1, min=0)
    yolo_boxes["phi_mins"], _ = min_max_scale(phi_minmax, yolo_boxes["ymin"], max=img_height-1, min=0)
    yolo_boxes["theta_maxs"], _ = min_max_scale(theta_minmax, yolo_boxes["xmax"], max=img_width-1, min=0)
    yolo_boxes["phi_maxs"], _ = min_max_scale(phi_minmax, yolo_boxes["ymax"], max=img_height-1, min=0)
    yolo_boxes["r_mins"], _ = min_max_scale(r_minmax, yolo_boxes["r_mins"], max=rmax-1, min=0)
    yolo_boxes["r_maxs"], _ = min_max_scale(r_minmax, yolo_boxes["r_maxs"], max=rmax-1, min=0)

    """Creating the resulting numpy array according to the output type"""
    filtered_output = np.zeros(shape=(0, OutputTypes.output_dim[output_type]))

    """Filtering all the points between the 3Dboxes"""
    for index, box in yolo_boxes.iterrows():
        cone = raw_pcd[(raw_pcd['phi'].between(left=box['phi_mins'], right=box['phi_maxs'], inclusive='both'))
                       & (raw_pcd['r'].between(left=box['r_mins'], right=box['r_maxs'], inclusive='both'))
                       & (raw_pcd['theta'].between(left=box['theta_mins'], right=box['theta_maxs'], inclusive='both'))]
        if cone.shape[0]>0 and len(cone['r'].values) > 3:
            if output_type == OutputTypes.CENTER:
                cone_center = select_center(cone)
                filtered_output = np.vstack([filtered_output, cone_center])
            elif output_type == OutputTypes.BOXES:
                cone_box = select_box(cone, first_quantile, second_quantile)
                filtered_output = np.vstack([filtered_output, cone_box])
            else:
                raise ValueError("select an output type")
    return filtered_output


def select_box(cone, first_quantile, second_quantile):
    """
    Method that computes the min and max vertexes of the cones.
    :param cone:
    :return:
    """
    nparam_density = stats.kde.gaussian_kde(cone['r'].values)
    data_space = np.linspace(cone['r'].min() - 0.5, cone['r'].max() + cone['r'].max() * .01)
    evaluated = nparam_density.evaluate(data_space)
    peaks, _ = find_peaks(evaluated, height=0)
    threshold = cone['r'].quantile(first_quantile)  # initial quartile
    if peaks.shape[0] > 1:
        x_min = np.argmin(evaluated[peaks[0]:peaks[1]])  # prendo il minimo tra i primi due picchi
        th_density = data_space[x_min]
        threshold = th_density if threshold > th_density else cone['r'].quantile(
            second_quantile)  # if the density threshold is higher than percentile take percentile .60 else use density
    cone = cone[cone['r'] < threshold]
    cone_box = np.array(
        [cone['phi'].min(), cone['phi'].max(), cone['theta'].min(), cone['theta'].max(), cone['r'].min(),
         cone['r'].max()])
    # print("{} {} {} {} {} {}".format(cone['phi'].min(), cone['phi'].max(), cone['theta'].min(), cone['theta'].max(), cone['r'].min(), cone['r'].max()))
    return cone_box


def select_center(cone):
    """
    Method that computes the center coordinates of the cones.
    :param cone: dataframe containing the cone points
    :param filtered_output: the resulting numpy
    :param row:
    :return:
    """     
    nparam_density = stats.kde.gaussian_kde(cone['r'].values)
    data_space = np.linspace(cone['r'].min(), cone['r'].min() + cone['r'].max())
    evaluated = nparam_density.evaluate(data_space)
    peaks, _ = find_peaks(evaluated, height=0)
    if len(peaks>0):
        rcenter = data_space[np.argmin(evaluated[peaks[0]])]
        cone = cone[cone['r'] <= rcenter+rcenter*0.05]

    cone_center = np.array([cone['X'].mean(), cone['Y'].mean(), cone['Z'].max() - cone['Z'].min()])

    return cone_center   

