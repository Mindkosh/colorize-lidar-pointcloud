import open3d as o3d
import numpy as np


def int2str(x): return str(int(x))
def float2str(x): return str(float(x))

# Read a binary or ascii pcd file
def read_pcd_file(pointcloud_filename):
    pointcloud_data_raw = o3d.io.read_point_cloud(pointcloud_filename)
    pointcloud_data = np.array(pointcloud_data_raw.points)
    return pointcloud_data

# Read a binary file with 4 properties in each point, and the data saved in 4-byte float format
def read_binary_file(pointcloud_filename):
    pointcloud_data = np.fromfile(pointcloud_filename, dtype=np.float32)
    pointcloud_data = pointcloud_data.reshape(-1, 4)
    return pointcloud_data

# Save a PLY file with x, y, z and RGB values
def save_ply(points, outfile):
    out_data = points.tolist()
    out_lines = [
        "ply",
        "format ascii 1.0",
        "element vertex " + str(len(out_data)),
        "property float x",
        "property float y",
        "property float z",
        "property uchar red",
        "property uchar green",
        "property uchar blue",
        "end_header",
    ]
    for pt in out_data:
        # Convert points into strings
        line = [float2str(co_ord_value) for co_ord_value in pt[:3]]

        # Convert RGB values into strings
        line += [int2str(color_value) for color_value in pt[3:]]
        out_lines.append(" ".join(line))

    out_lines += "\n"

    fout = open(outfile, "w")
    fout.write("\n".join(out_lines))
    fout.close()
