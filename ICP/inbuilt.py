import numpy as np
import open3d as o3d
import copy


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0, 0])
    target_temp.paint_uniform_color([0, 0, 1])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


source = o3d.io.read_point_cloud("/home/venky/PycharmProjects/AGV/ICP/cloud_bin_0.pcd")
target = o3d.io.read_point_cloud("/home/venky/PycharmProjects/AGV/ICP/cloud_bin_1.pcd")
source_temp = copy.deepcopy(target)
target_temp = copy.deepcopy(target)

o3d.visualization.draw_geometries([source_temp, target_temp],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])
threshold = 0.9
trans_init = np.eye(4)
RigidBodyTransform = o3d.pipelines.registration.registration_generalized_icp(
    source, target, threshold)

print(RigidBodyTransform)
print("Transformation is:")
print(RigidBodyTransform.transformation)
draw_registration_result(source, target, RigidBodyTransform.transformation)
