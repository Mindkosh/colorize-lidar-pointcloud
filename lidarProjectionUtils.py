import math
import numpy as np


def world2cam(extrinsic, pt):
    extrinsic = np.array(extrinsic)
    pt = np.array(pt)
    pt = np.append(pt, 1)
    pt = np.matmul(extrinsic, pt)
    return pt[:3]

def brownDistortion(distortion, point):
    k1 = distortion[0]
    k2 = distortion[1]
    p1 = distortion[2]
    p2 = distortion[3]

    x = point[0]
    y = point[1]

    r2 = x*x + y*y
    r4 = r2*r2

    x = x * (1 + k1*r2 + k2*r4) + 2*p1*x*y + p2*(r2 + 2*x*x)
    y = y * (1 + k1*r2 + k2*r4) + 2*p2*x*y + p1*(r2 + 2*y*y)

    return [x, y]

def cam2image(params, pointCameraFrame, cameraModel):
    u_d = 0
    v_d = 0

    if (pointCameraFrame[2] <= 0):
        pointCameraFrame[2] = 1e-6

    if (cameraModel == 'FISHEYE'):
        norm = np.linalg.norm(pointCameraFrame)
        X = pointCameraFrame[0] / norm
        Y = pointCameraFrame[1] / norm
        Z = pointCameraFrame[2] / norm

        X /= Z + params["mirrorParameter"]
        Y /= Z + params["mirrorParameter"]
        u_d = X
        v_d = Y
    else:
        if "x_is_z" in params:
            u_d = -pointCameraFrame[1] / pointCameraFrame[0]
            v_d = -pointCameraFrame[2] / pointCameraFrame[0]
        else:
            u_d = pointCameraFrame[0] / pointCameraFrame[2]
            v_d = pointCameraFrame[1] / pointCameraFrame[2]
    
    if "distortion" in params:
        distorted = brownDistortion(params["distortion"], [u_d, v_d])
        u_d = distorted[0]
        v_d = distorted[1]

    F_U = params["intrinsic"][0]
    F_V = params["intrinsic"][1]
    C_U = params["intrinsic"][2]
    C_V = params["intrinsic"][3]

    u_d = u_d * F_U + C_U
    v_d = v_d * F_V + C_V

    return [u_d, v_d]