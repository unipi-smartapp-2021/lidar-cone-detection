import pandas as pd


def read_matlab_labels(label_file):
    boxes = pd.read_csv(label_file, delim_whitespace=True, header=None)
    boxes.columns = ["x", "y", "z", "dx", "dy", "dz"]
    boxes.x = boxes.x - boxes.dx / 2
    boxes.y = boxes.y - boxes.dy / 2
    boxes.z = boxes.z - boxes.dz / 2
    boxes.dx = boxes.x + boxes.dx
    boxes.dy = boxes.y + boxes.dy
    boxes.dz = boxes.z + boxes.dz

    return boxes