import json
import numpy as np
from PIL import Image
import lidarProjectionUtils
import pointcloudIO

camera_parameters_filepath = "example_files/nuscenes/nuscenes-camera-parameters.json"
raw_params = json.load(open(camera_parameters_filepath))
camera_params = {i["device_id"]:i for i in raw_params}

pointcloud_file = "example_files/nuscenes/n008-2018-08-01-15-16-36-0400__LIDAR_TOP__1533151614948536.pcd"
image_files = {
    1: 'example_files/nuscenes/n008-2018-08-01-15-16-36-0400__CAM_BACK__1533151614937558.jpg',
    2: 'example_files/nuscenes/n008-2018-08-01-15-16-36-0400__CAM_FRONT__1533151614912404.jpg',
    3: "example_files/nuscenes/n008-2018-08-01-15-16-36-0400__CAM_FRONT_LEFT__1533151616404799.jpg",
    4: "example_files/nuscenes/n008-2018-08-01-15-16-36-0400__CAM_FRONT_RIGHT__1533151614920482.jpg",
    5: "example_files/nuscenes/n008-2018-08-01-15-16-36-0400__CAM_BACK_LEFT__1533151611897405.jpg",
    6: "example_files/nuscenes/n008-2018-08-01-15-16-36-0400__CAM_BACK_RIGHT__1533151614928113.jpg",
}

outfile = pointcloud_file.replace(".pcd", "_colorized.ply")

MAX_DEPTH = 9000
DEPTH_THRESHOLD = 0.1

# Read pointcloud file
pointcloud_data = pointcloudIO.read_pcd_file(pointcloud_file)
print(pointcloud_data.shape)


point_rgb = np.full((pointcloud_data.shape[0], 3), 255, dtype=np.uint8)
point_depth = np.full(pointcloud_data.shape[0], MAX_DEPTH, dtype=np.float32)
min_depth = MAX_DEPTH
max_depth = 0

for device_id in [1, 2, 3, 4, 5, 6]:
    image_file = image_files[device_id]
    image = np.array(Image.open(image_file))
    image_size = image.shape[:2]

    # Save which point cloud point is mapped to a pixel
    pixel_point_map = np.full(image_size, 0, dtype=np.uint32)

    # If a point cloud point has been mapped to a pixel
    pixel_point_map_set = np.full(image_size, False, dtype=bool)

    for ind, pt in enumerate(pointcloud_data):
        # Move the point from lidar frame to camera frame
        cam_pt = lidarProjectionUtils.world2cam(
            camera_params[device_id]["extrinsic"], pt[:3])

        # Project the point from camera frame to the image plane
        pixel = lidarProjectionUtils.cam2image(
            camera_params[device_id], cam_pt, camera_params[device_id]["cameraModel"])

        x = int(pixel[0])
        y = int(pixel[1])

        depth = cam_pt[2]
        min_depth = min(min_depth, depth)
        max_depth = max(max_depth, depth)

        # Check if the projected pixel is within the image and if the depth is greater than a threshold to avoid mapping points
        # that might be on the other side of the camera
        if y < image_size[0] and y >= 0 and x < image_size[1] and x >= 0 and depth > DEPTH_THRESHOLD:
            if point_depth[ind] > depth:
                point_depth[ind] = depth

                # Map the point to a pixel if
                # 1. The pixel has not been mapped to a point cloud point yet, 
                # 2. If the point is closer to the camera than the previous point that was mapped to this pixel
                # 3. If the new point is within a certain depth range of the previous point that was mapped to this pixel
                # Removing this may give you more colored points, at the expense of accuracy
                if pixel_point_map_set[y][x] is False or (point_depth[pixel_point_map[y][x]] > depth) or abs(point_depth[pixel_point_map[y][x]] - depth) < 0.5:
                    old_ind = pixel_point_map[y][x]
                    pixel_point_map[y][x] = ind
                    pixel_point_map_set[y][x] = True
                    point_rgb[ind] = image[y][x]

                    # If a point was already mapped to this pixel, but the new point is closer, then set the old point to white
                    if (point_depth[old_ind] > depth) and abs(point_depth[old_ind] - depth) > 0.5:
                        point_rgb[old_ind] = [255, 255, 255]


# Add the new columns to the existing array
rgb_points = np.hstack((pointcloud_data[:,:3], np.array(point_rgb)))

pointcloudIO.save_ply(rgb_points, outfile)