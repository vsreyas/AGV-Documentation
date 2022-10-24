import numpy as np
import open3d as o3d


def icp_point_to_point_lm(source_points, dest_points, initial, loop, iter = 50):

    J = []
    e = []

    for i in range(0, dest_points.shape[0] - 1):
        dx = dest_points[i][0]
        dy = dest_points[i][1]
        dz = dest_points[i][2]

        sx = source_points[i][0]
        sy = source_points[i][1]
        sz = source_points[i][2]

        alpha = initial[0][0]
        beta = initial[1][0]
        gamma = initial[2][0]
        tx = initial[3][0]
        ty = initial[4][0]
        tz = initial[5][0]

        a1 = (-2 * beta * sx * sy) - (2 * gamma * sx * sz) + (2 * alpha * ((sy * sy) + (sz * sz))) + (
                    2 * ((sz * dy) - (sy * dz))) + 2 * ((sy * tz) - (sz * ty))
        a2 = (-2 * alpha * sx * sy) - (2 * gamma * sy * sz) + (2 * beta * ((sx * sx) + (sz * sz))) + (
                    2 * ((sx * dz) - (sz * dx))) + 2 * ((sz * tx) - (sx * tz))
        a3 = (-2 * alpha * sx * sz) - (2 * beta * sy * sz) + (2 * gamma * ((sx * sx) + (sy * sy))) + (
                    2 * ((sy * dx) - (sx * dy))) + 2 * ((sx * ty) - (sy * tx))
        a4 = 2 * (sx - (gamma * sy) + (beta * sz) + tx - dx)
        a5 = 2 * (sy - (alpha * sz) + (gamma * sx) + ty - dy)
        a6 = 2 * (sz - (beta * sx) + (alpha * sy) + tz - dz)

        residual = (a4 * a4 / 4) + (a5 * a5 / 4) + (a6 * a6 / 4)

        J = np.array([a1, a2, a3, a4, a5, a6])
        e = np.array([residual])

        J.append(J)
        e.append(e)

    jacobian = np.array(J)
    residual = np.array(e)

    update = -np.dot(np.dot(np.linalg.pinv(np.dot(np.transpose(jacobian), jacobian)), np.transpose(jacobian)), residual)

    initial = initial + update

    print (np.transpose(initial))

    loop = loop + 1

    if (loop < iter):

        icp_point_to_point_lm(source_points, dest_points, initial, loop)


def read(file_path):
    a = []
    with open(file_path) as f:
        content = f.readlines()
        for line in content:
            x = float(re.split('\s', line)[0])
            y = float(re.split('\s', line)[1])
            z = float(re.split('\s', line)[2])

            b = np.array([x, y, z])
            a.append(b)

    data = np.array(a)
    return data

src = read("/home/venky/PycharmProjects/AGV/ICP/point_cloud_sample1.pts")
dest = read("/home/venky/PycharmProjects/AGV/ICP/point_cloud_sample2.pts")

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(src)