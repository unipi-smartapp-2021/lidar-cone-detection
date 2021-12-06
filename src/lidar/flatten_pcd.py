import os

from src.lidar.utils import from_pcd_to_image

img_width = 600
img_height = 400
out_path_img = "converted_pcds/"
source_dir = "C:/dev/lidar-labeling/pcd_outputs/"
for pcd in os.listdir(source_dir):
    from_pcd_to_image(source_dir+pcd, out_path_img, pcd.replace(".pcd", ".png"),
                      (img_width, img_height),
                      front_cut=False)
