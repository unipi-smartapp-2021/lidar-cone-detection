import os

from matplotlib import pyplot as plt

from src.lidar.read_matlab_labels import read_matlab_labels
from utils import *

out_path_labels = "debug/gtruth_labels/"
out_path_img = "debug/converted_pcds/"
try:
    os.mkdir(out_path_labels)
except:
    pass
try:
    os.mkdir(out_path_img)
except:
    pass

source_dir = "pcd_labels/"
source_pcd_dir = "C:/dev/lidar-labeling/pcd_outputs/"

matlab_labels = "lidar_pointcloud12.txt"

boxes = read_matlab_labels(source_dir + matlab_labels)

boxes['theta'], boxes['phi'], boxes['R'] = cart2sph(boxes.y.values, boxes.z.values, boxes.x.values)
boxes['dtheta'], boxes['dphi'], boxes['dR'] = cart2sph(boxes.dy.values, boxes.dz.values, boxes.dx.values)

img_width = 600
img_height = 400

df = pd.DataFrame()
pcd_filename = source_pcd_dir + matlab_labels.replace(".txt", ".pcd")
print("reading ", pcd_filename)
image, outarr, (theta, phi, r), (df["theta"], df["phi"], df["R"]) = from_pcd_to_image(pcd_filename,
                                                                                      out_path_img,
                                                                                      matlab_labels.replace(".txt",
                                                                                                            ".png"),
                                                                                      (img_width, img_height),
                                                                                      front_cut=False)

df = pd.concat([df, pd.DataFrame(outarr, columns=["X", "Y", "Z"])], axis=1)

boxes['R'], minmax = min_max_scale((0, 255), boxes['R'], r.max(), r.min())
boxes['dR'], minmax = min_max_scale((0, 255), boxes['dR'], r.max(), r.min())

boxes['theta'], minmax = min_max_scale((0, img_width), boxes['theta'], theta.max(), theta.min())
boxes['dtheta'], minmax = min_max_scale((0, img_width), boxes['dtheta'], theta.max(), theta.min())

boxes['phi'], minmax = min_max_scale((0, img_height), boxes['phi'], phi.max(), phi.min())
boxes['dphi'], minmax = min_max_scale((0, img_height), boxes['dphi'], phi.max(), phi.min())
#
fig = df.plot.scatter(x='theta', y='phi', c='R', colormap='gray', s=1)
boxes.plot.scatter(x='theta', y='phi', c='#00FF00', s=10, ax = fig)
boxes.plot.scatter(x='dtheta', y='dphi', c='#FF0000', s=10, ax = fig)
plt.show()

label_file = out_path_labels + matlab_labels
with open(label_file, "w") as fp:
    for box in boxes.iterrows():
        x, y, z, dx, dy, dz, R, theta, phi, dR, dtheta, dphi = box[1]

        inner_points = df[df["X"].between(x, dx)]
        inner_points = inner_points[inner_points["Y"].between(y, dy)]
        inner_points = inner_points[inner_points["Z"].between(z, dz)]

        # clip out points on the floor
        inner_points = inner_points[(inner_points["theta"] > 50)]
        print("{} points selected".format(len(inner_points)),
              # int(inner_points.theta.min()), int(inner_points.theta.max()),  int(inner_points.phi.min()), int(inner_points.phi.max())
              )

        if (len(inner_points) > 0):
            start_pt = (int(inner_points.theta.min()), int(inner_points.phi.min()))
            end_pt = (int(inner_points.theta.max()), int(inner_points.phi.max()))

            center = (start_pt[0] + (end_pt[0] - start_pt[0]) / 2, start_pt[1] + (end_pt[1] - start_pt[1]) / 2)
            width = abs(end_pt[0] - start_pt[0])
            height = abs(end_pt[1] - start_pt[1])

            # print("drawing", int(inner_points.theta.min()), int(inner_points.theta.max()),  int(inner_points.phi.min()), int(inner_points.phi.max()))
            image = cv2.rectangle(
                img=image,
                pt1=start_pt,
                pt2=end_pt,
                color=(0, 0, 255),
                thickness=1)
            # cv2.imshow("coloringpoints", image)
            # cv2.waitKey(0)

            fp.write("0 {} {} {} {}\n".format(center[0] / img_width, center[1] / img_height, width / img_width,
                                              height / img_height))
    cv2.imwrite(out_path_img + matlab_labels.replace(".txt", "_box.png"), image)
    fp.close()