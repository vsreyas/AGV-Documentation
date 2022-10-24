#!/usr/bin/env python
from glob import glob
import ros_numpy, message_filters
import rospy, numpy as np, time, open3d as o3d
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point

out = Point()
start = 0
M = np.eye(4)
t = t_x = 0
origin = np.ones((4,1))

def callback(odo, pcl):
    rospy.loginfo("Received ")
    global odm, points, M, p_i, origin, start, out, t, pub
    odm = odo

    pc = ros_numpy.numpify(pcl)
    ps=np.zeros((pc.shape[0],3))
    ps[:,0]=pc['x']
    ps[:,1]=pc['y']
    ps[:,2]=pc['z']
    p = o3d.geometry.PointCloud()
    p.points = o3d.utility.Vector3dVector(np.array(ps, dtype=np.float32))
    points = p
    
    if t ==0:
        origin[0] = odo.pose.pose.position.x 
        origin[1] = odo.pose.pose.position.y 
        origin[2] = odo.pose.pose.position.z 
  
        p_i = p
        t = 1
        return


    t_vec  = np.eye(4)
    end = time.time()
    dt = end-start
    start = end

    t_vec[0][3] = odm.twist.twist.linear.x * dt
    t_vec[1][3] = odm.twist.twist.linear.y * dt
    t_vec[2][3] = odm.twist.twist.linear.z * dt
    source = p_i
    target = points

    threshold = 0.9
    RigidBodyTransform = o3d.pipelines.registration.registration_icp(
        source, target, threshold, t_vec)
        
    M = np.dot(np.asarray(RigidBodyTransform.transformation), M)
    p_i = points
    p = np.dot(M,origin)
        
    out.x = p[0]
    out.y = p[1]
    out.z = p[2]

    rospy.loginfo(out)
    pub.publish(out)
    return

def subs():
    global start, pub
    start = time.time()
    pub = rospy.Publisher('myfinallocation', Point, queue_size=10)
    rospy.init_node('ICP', anonymous= True)
    odo_s = message_filters.Subscriber("carla/vehicle/086/odometry", Odometry)
    pcl_s = message_filters.Subscriber("carla/vehicle/086/lidar/front/point_cloud",PointCloud2)

    ts =  message_filters.ApproximateTimeSynchronizer([odo_s, pcl_s], 10, 0.01, allow_headerless=True)
    ts.registerCallback(callback)
    
    rospy.spin()

  


if __name__ == '__main__':
    while True:
        try:
            subs()
        except rospy.ROSInterruptException:
            pass