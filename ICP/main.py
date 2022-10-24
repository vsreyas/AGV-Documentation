import open3d as o3d
import numpy as np
from sklearn.neighbors import NearestNeighbors
import copy


def transform(A, B):
    # get number of dimensions
    n = A.shape[1]

    # translate points to their centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    a = A - centroid_A
    b = B - centroid_B

    # rotation matrix
    H = np.dot(a.T, b)
    U, S, V = np.linalg.svd(H)
    R = np.dot(V.T, U.T)

    # translation
    t = centroid_B.T - np.dot(R, centroid_A.T)

    # homogeneous transformation
    T = np.identity(n + 1)
    T[:n, :n] = R
    T[:n, n] = t

    return T, R, t


def correspondence(source, final):

    match = NearestNeighbors(n_neighbors=1)
    match.fit(final)
    distances, indices = match.kneighbors(source, return_distance=True)
    return distances.ravel(), indices.ravel()


def icp(A, B, init_pose=None, max_iterations=20, tolerance=0.001):
    # get number of dimensions
    n = A.shape[1]

    # make points homogeneous, copy them to maintain the originals
    src = np.ones((n + 1, A.shape[0]))
    dst = np.ones((n + 1, B.shape[0]))
    src[:n, :] = np.copy(A.T)
    dst[:n, :] = np.copy(B.T)

    prev_error = 0

    for iteration in range(max_iterations):
        # find the nearest neighbors between the current source and destination points
        distances, indices = correspondence(src[:n, :].T, dst[:n, :].T)

        # compute the transformation between the current source and nearest destination points
        T, _, _ = transform(src[:n, :].T, dst[:n, indices].T)

        # update the current source
        src = np.dot(T, src)

        # check error
        mean_error = np.mean(distances)
        if np.abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    # calculate final transformation
    T, _, _ = transform(A, src[:n, :].T)

    return T, distances, iteration


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0, 0])
    target_temp.paint_uniform_color([0, 0, 1])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


source = o3d.io.read_point_cloud("//home/venky/PycharmProjects/AGV/ICP/cloud_bin_0.pcd")
target = o3d.io.read_point_cloud("/home/venky/PycharmProjects/AGV/ICP/cloud_bin_1.pcd")

source_t = np.asarray(source.points)
target_t = np.asarray(target.points)
T, d, iteration = icp(source_t, target_t)

print("Transformation is:")
print(T)

draw_registration_result(source, target, T)
