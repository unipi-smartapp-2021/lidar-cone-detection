import os

from read_matlab_labels import *
from utils import *

foldername = "post_proc"
out_path_labels = "T:/pcds/"+foldername+"/out/" #output for the labels
out_path_img = "T:/pcds/"+foldername+"/out/" #output for the imgs
try:
    os.mkdir(out_path_labels)
except:
    pass

source_dir = "T:/pcds/"+foldername+"/labels/"  #source with the mathlab labels
source_pcd_dir = "T:/pcds/"+foldername+"/pcd/" #source with the pcds
pcd_names = os.listdir(source_pcd_dir)
label_names = os.listdir(source_dir)
minimum_inner_points = 0
# for i, name in enumerate(label_names):
#     os.rename(source_dir+name, source_dir+(pcd_names[i].split("."))[0]+".txt")
for i, matlab_labels in enumerate(os.listdir(source_dir)):

    boxes = read_matlab_labels(source_dir + matlab_labels)

    boxes['theta'], boxes['phi'], boxes['R'] = cart2sph(boxes.y.values, boxes.z.values, boxes.x.values)
    boxes['dtheta'], boxes['dphi'], boxes['dR'] = cart2sph(boxes.dy.values, boxes.dz.values, boxes.dx.values)

    img_width = 480
    img_height = 480

    df = pd.DataFrame()
    pcd_filename = source_pcd_dir + pcd_names[i]
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

    boxes['theta'], minmax = min_max_scale((0, img_width-1), boxes['theta'], theta.max(), theta.min())
    boxes['dtheta'], minmax = min_max_scale((0, img_width-1), boxes['dtheta'], theta.max(), theta.min())

    boxes['phi'], minmax = min_max_scale((0, img_height-1), boxes['phi'], phi.max(), phi.min())
    boxes['dphi'], minmax = min_max_scale((0, img_height-1), boxes['dphi'], phi.max(), phi.min())
    #
    # fig = df.plot.scatter(x='theta', y='phi', c='R', colormap='gray', s=1)
    # boxes.plot.scatter(x='theta', y='phi', c='#00FF00', s=10, ax = fig)
    # boxes.plot.scatter(x='dtheta', y='dphi', c='#FF0000', s=10, ax = fig)
    # plt.show()

    label_file = out_path_labels + matlab_labels
    with open(label_file, "w") as fp:
        for box in boxes.iterrows():
            x, y, z, dx, dy, dz, theta, phi, r, dtheta, dphi, dR = box[1]

            inner_points = df[df["X"].between(x, dx)]
            inner_points = inner_points[inner_points["Y"].between(y, dy)]
            inner_points = inner_points[inner_points["Z"].between(z, dz)]

            # clip out points on the floor
            inner_points = inner_points[(inner_points["theta"] > 50)]
            print("{} points selected".format(len(inner_points)),
                  # int(inner_points.theta.min()), int(inner_points.theta.max()),  int(inner_points.phi.min()), int(inner_points.phi.max())
                  )

            if len(inner_points)  > minimum_inner_points:
                start_pt = ((inner_points.theta.min()), (inner_points.phi.min())) #(int(theta), int(phi))
                end_pt = ((inner_points.theta.max()), (inner_points.phi.max()))   #(int(dtheta), int(dphi))

                center = (start_pt[0] + (end_pt[0] - start_pt[0]) / 2, start_pt[1] + (end_pt[1] - start_pt[1]) / 2)
                width = abs(end_pt[0] - start_pt[0])
                height = abs(end_pt[1] - start_pt[1])

                # print("drawing", int(inner_points.theta.min()), int(inner_points.theta.max()),  int(inner_points.phi.min()), int(inner_points.phi.max()))
                """image = cv2.rectangle(
                    img=image,
                    pt1=start_pt,
                    pt2=end_pt,
                    color=(0, 0, 255),
                    thickness=1)"""
                # cv2.imshow("coloringpoints", image)
                # cv2.waitKey(0)

                fp.write("0 {} {} {} {}\n".format(round(center[0]/img_width, 4), round(center[1]/img_height,4), round(width/img_width,4), round(height/img_height),4))
                # fp.write("{},{},{},{}\n".format(round(start_pt[0], 4), round(start_pt[1],4), round(end_pt[0],4), round(end_pt[1],4)))
        cv2.imwrite(out_path_img + matlab_labels.replace(".txt", "_box.png"), image)
        fp.close()
