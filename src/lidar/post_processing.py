from utils import *
import pandas as pd
import math

path = "./post_process_data/"
cloud = o3d.io.read_point_cloud(path+"lidar.pcd") # Read the point cloud file
raw_pcd = pd.DataFrame(cloud.points, columns=["X", "Y", "Z"])
visualize_cartesian(raw_pcd.values)
r_minmax, theta_minmax, phi_minmax = read_minmax_file(path+"lidar_minmax.txt")
yolo_boxes = pd.read_csv(path+"lidar.csv")

img_width = 480
img_height = 400

yolo_boxes["r_mins"] = 255
yolo_boxes["r_maxs"] = 255

"""Reading image"""
img_df = pd.DataFrame()
img_df['r'], img_df['phi'], img_df['theta'] = read_spherical_image(path+"lidar.png")
filtered_image = pd.DataFrame(columns=["r", "phi", "theta"])
for index, row in yolo_boxes.iterrows():
    theta_min = math.floor(row["xmin"])
    theta_max = math.ceil(row["xmax"])
    phi_min = math.floor(row["ymin"])
    phi_max = math.ceil(row["ymax"])
    r_values = img_df[((img_df['phi'] >= phi_min) & (img_df['phi'] <= phi_max)) & ((img_df['theta'] >= theta_min) & (img_df['theta']<=theta_max))]
    if r_values.shape[0] > 0:
        filtered_image = pd.concat((filtered_image, r_values.astype(float)), axis=0)
    yolo_boxes.loc[index, 'r_mins'] = r_values['r'].min() if r_values.shape[0] > 0 else 255 #255 are points to be removed
    yolo_boxes.loc[index, 'r_maxs'] = r_values['r'].max() if r_values.shape[0] > 0 else 255 #255 are points to be removed

"""visualize filterd image"""
filtered_image['r'], _, = min_max_scale(r_minmax, filtered_image['r'], max=255-1, min=0)
filtered_image['theta'], _, = min_max_scale(theta_minmax, filtered_image['theta'], max=img_width-1, min=0)
filtered_image['phi'], _, = min_max_scale(phi_minmax, filtered_image['phi'], max=img_height-1, min=0)

"""Convert back to Cartesian"""
x, y, z = sph2cart(filtered_image['theta'], filtered_image['phi'], filtered_image['r'])        # convert back to cartesian

"""Visualize the point cloud"""
cloud = np.column_stack((x, y, z))
visualize_cartesian(cloud)

print(yolo_boxes.shape)
yolo_boxes = yolo_boxes[(yolo_boxes['r_mins'] != 255) & (yolo_boxes['r_maxs'] != 255)]
print(yolo_boxes.shape)

"""They are columns added to our dataframe"""
yolo_boxes["theta_mins"], _ = min_max_scale(theta_minmax, yolo_boxes["xmin"], max=img_width-1, min=0)
yolo_boxes["phi_mins"], _ = min_max_scale(phi_minmax, yolo_boxes["ymin"], max=img_height-1, min=0)
yolo_boxes["theta_maxs"], _ = min_max_scale(theta_minmax, yolo_boxes["xmax"], max=img_width-1, min=0)
yolo_boxes["phi_maxs"], _ = min_max_scale(phi_minmax, yolo_boxes["ymax"], max=img_height-1, min=0)
yolo_boxes["r_mins"], _ = min_max_scale(r_minmax, yolo_boxes["r_mins"], max=255-1, min=0)
yolo_boxes["r_maxs"], _ = min_max_scale(r_minmax, yolo_boxes["r_maxs"], max=255-1, min=0)

"""convert back to cartesian"""
yolo_boxes['xmin'], yolo_boxes['ymin'], yolo_boxes['zmin'] = sph2cart(yolo_boxes["theta_mins"], yolo_boxes["phi_mins"], yolo_boxes["r_mins"])
yolo_boxes['xmax'], yolo_boxes['ymax'], yolo_boxes['zmax'] = sph2cart(yolo_boxes["theta_maxs"], yolo_boxes["phi_maxs"], yolo_boxes["r_maxs"])

filtered_output = pd.DataFrame(columns=["X", "Y", "Z"])
"""Filtering all the points between these boxes"""
for index, row in yolo_boxes.iterrows():
    cono = raw_pcd[(raw_pcd['X'].between(left=row['xmin'], right=row['xmax'], inclusive=True))
                   & (raw_pcd['Z'].between(left=row['zmin'], right=row['zmax'], inclusive=True))]
    filtered_output = pd.concat((filtered_output, cono.astype(float)), axis=0)


visualize_cartesian(filtered_output[["X", "Y", "Z"]].values)
print(yolo_boxes)
print(yolo_boxes.head())