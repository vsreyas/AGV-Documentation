#!/usr/bin/env python
from glob import glob
from unittest import result
import ros_numpy, message_filters
import rospy, numpy as np, time, open3d as o3d
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from tf2_msgs.msg import TFMessage
import tf.transformations as trans


out = Point()
start = 0
M = np.eye(4)
t = t_x = 0
origin = np.ones((4,1))
origin[0] = -6.44610357e+00
origin[1] =  7.90548630e+01
origin[2] = -7.71444291e-03 




def preprocess_point_cloud(pcd, voxel_size = 0.05):
    
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def callback(pcl,tfs):
    rospy.loginfo("Received ")
    global points, M, p_i, origin, start, out, t, pub
    print("Received point cloud")

    t_vec = tfs.transform.translation

    q = tfs.transform.rotation
    q_m = trans.euler_from_quaternion(q, axes='sxyz')
    
    M = trans.compose_matrix(angles=q_m, translate=t_vec)

    pc = ros_numpy.numpify(pcl)
    ps=np.zeros((pc.shape[0],3))
    ps[:,0]=pc['x']
    ps[:,1]=pc['y']
    ps[:,2]=pc['z']
    p = o3d.geometry.PointCloud()
    p.points = o3d.utility.Vector3dVector(np.array(ps, dtype=np.float32))
    p_down, p_fdpfh = preprocess_point_cloud(p)
    points = p_down
    print("Processed point cloud")
    if t ==0:
        p_i = points
        M = np.eye(4)
        t = 1
        return

    points.transform(M)
    p_i = p_i + points
    radius_feature = 0.05 * 5
    print("transformed")
    p_i_fdpfh = o3d.pipelines.registration.compute_fpfh_feature(
        p_i,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))

    result = o3d.pipelines.registration.registration_fast_based_on_feature_matching(
        p_down, p_i, p_fdpfh, p_i_fdpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(maximum_correspondence_distance=0.0025))
    print("result obtained")
    p = np.dot(result.transformation,origin)
    out.x = p[0]
    out.y = p[1]
    out.z = p[2]

    rospy.loginfo(out)
    pub.publish(out)
    print("published")
    return

def subs():
    global start, pub, t_x
    start = time.time()
    pub = rospy.Publisher('myfinallocation', Point, queue_size=10)
    

    pcl_s = message_filters.Subscriber("carla/vehicle/086/lidar/front/point_cloud",PointCloud2)
    tqt_s = message_filters.Subscriber("tf", TFMessage)

    ts =  message_filters.ApproximateTimeSynchronizer([pcl_s, tqt_s], 10, 5, allow_headerless=True)
    ts.registerCallback(callback)
    
    rospy.spin()

  


if __name__ == '__main__':
    rospy.init_node('ICP', anonymous= True)
    while True:
        try:
            subs()
        except rospy.ROSInterruptException:
            pass