from utils import *
import pandas as pd

path = "./post_process_data/"
cloud = o3d.io.read_point_cloud(path+"lidar_pcd.pcd") # Read the point cloud file
out_arr = np.asarray(cloud.points)
raw_pcd = pd.DataFrame(out_arr, columns=["X", "Y", "Z"])
visualize_cartesian(out_arr)
r_minmax, theta_minmax, phi_minmax = read_minmax_file(path+"a_minmax.txt")
yolo_boxes = pd.read_csv(path+"yolo_out.csv")

img_width = 480
img_height = 480

"""They are columns added to our dataframe"""
yolo_boxes["theta_mins"], _ = min_max_scale(theta_minmax, yolo_boxes["xmin"], 0, img_width-1)
yolo_boxes["phi_mins"], _ = min_max_scale(phi_minmax, yolo_boxes["ymin"], 0, img_height-1)
yolo_boxes["theta_maxs"], _ = min_max_scale(theta_minmax, yolo_boxes["xmax"], 0, img_width-1)
yolo_boxes["phi_maxs"], _ = min_max_scale(phi_minmax, yolo_boxes["ymax"], 0, img_height-1)

"""Reading image"""
img_df = pd.DataFrame()
img_df['r'], img_df['phi'], img_df['theta'] = read_spherical_image(path+"a.png")
for index, row in yolo_boxes.iterrows():
    max = img_df[(img_df['phi'].between(left=row["phi_mins"], right=row["phi_maxs"])) & (img_df['theta'].between(
        left=row["theta_mins"], right=row["theta_maxs"]))]['r'].max()
    min = img_df[(img_df['phi'].between(left=row["phi_mins"], right=row["phi_maxs"])) & (img_df['theta'].between(
        left=row["theta_mins"], right=row["theta_maxs"]))]['r'].min()
    print(str(min)+" "+str(max))

yolo_boxes["rmin"]
yolo_boxes["rmax"]
sph2cart()








print(yolo_boxes.head())