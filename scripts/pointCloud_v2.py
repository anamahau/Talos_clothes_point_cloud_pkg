#!/usr/bin/env python

import numpy as np
import open3d as o3d
import rospy

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int32, Float32MultiArray

from tf_reader import getTfTransform

# =====================
# CONFIG
# =====================

MAX_X1 = 2.0
MAX_X2 = 1.5
# MAX_Y = -0.1
# MAX_Z = 2.0
PLOT = False   # turn OFF for speed

# =====================
# FAST PointCloud2 → NumPy
# =====================

def pointcloud2_to_xyz_fast(cloud_msg):
    '''Convert PointCloud2 → Nx3 NumPy array (FAST)'''
    dtype = np.dtype([
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32)
    ])

    cloud_arr = np.frombuffer(cloud_msg.data, dtype=dtype)

    xyz = np.vstack((cloud_arr['x'], cloud_arr['y'], cloud_arr['z'])).T

    # Remove NaNs
    mask = np.isfinite(xyz).all(axis=1)
    return xyz[mask]


# =====================
# Open3D helpers
# =====================

def create_cloud(xyz):
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(xyz)
    pc.colors = o3d.utility.Vector3dVector(np.zeros_like(xyz))
    return pc


# =====================
# Low / High detection
# =====================

def find_low_high(points):
    z = points[:, 2]

    z_low = np.percentile(z, 1)
    z_high = np.percentile(z, 99)

    low_pts  = points[z <= z_low]
    high_pts = points[z >= z_high]

    low_mean  = low_pts.mean(axis=0)
    high_mean = high_pts.mean(axis=0)

    return high_mean, low_mean


# =====================
# MAIN PROCESSING
# =====================

class PCAnalyzer:
    def __init__(self):
        rospy.init_node('pointCloud', anonymous=True)

        self.run_analysis = False
        self.iteration = 0

        # TF (compute once!)
        # self.T = getTfTransform('base_link', 'rgbd_depth_optical_frame')

        # Publishers (ONLY ONCE)
        self.pub_high = rospy.Publisher('/highPCpoint', Float32MultiArray, queue_size=1, latch=True)
        self.pub_low  = rospy.Publisher('/lowPCpoint', Float32MultiArray, queue_size=1, latch=True)

        rospy.Subscriber('/PCrequest', Int32, self.trigger_cb)

        rospy.loginfo('PCAnalyzer ready')

    def trigger_cb(self, msg):
        self.run_analysis = True
        self.iteration = msg.data

    def transform_points(self, xyz):
        '''Apply homogeneous transform'''
        ones = np.ones((xyz.shape[0], 1))
        xyz_h = np.hstack((xyz, ones))

        self.T = getTfTransform('base_link', 'rgbd_depth_optical_frame')

        xyz_tf = (self.T @ xyz_h.T).T[:, :3]
        return xyz_tf

    def filter_points(self, xyz):
        '''Apply iteration-specific filtering'''
        if self.iteration == 1:
            # return xyz[xyz[:, 2] < MAX_Z]
            return xyz[xyz[:, 0] < MAX_X1]

        elif self.iteration == 2:
            # return xyz[(xyz[:, 2] < MAX_Z) & (xyz[:, 1] < MAX_Y)]
            return xyz[xyz[:, ] < MAX_X2]

        return xyz

    def process(self):
        rospy.loginfo('Waiting for point cloud...')
        pc_msg = rospy.wait_for_message('/rgbd/depth/points', PointCloud2)

        # FAST conversion
        xyz = pointcloud2_to_xyz_fast(pc_msg)

        # Transform
        xyz = self.transform_points(xyz)

        # Filter
        xyz = self.filter_points(xyz)

        if xyz.shape[0] == 0:
            rospy.logwarn('No points after filtering!')
            return

        # =====================
        # ITERATION 1 (plane removal)
        # =====================
        if self.iteration == 1:
            pc = create_cloud(xyz)

            plane_model, inliers = pc.segment_plane(
                distance_threshold=0.01,
                ransac_n=3,
                num_iterations=300
            )

            points = np.asarray(pc.points)

            mask_plane = np.zeros(len(points), dtype=bool)
            mask_plane[inliers] = True

            a, b, c, d = plane_model
            mask_above = (a*points[:,0] + b*points[:,1] + c*points[:,2] + d > 0)

            objects = points[mask_above & ~mask_plane]

            if objects.shape[0] == 0:
                rospy.logwarn('No object points found!')
                return

            high, low = find_low_high(objects)

        # =====================
        # ITERATION 2
        # =====================
        elif self.iteration == 2:
            high, low = find_low_high(xyz)

        # =====================
        # Publish
        # =====================
        rospy.sleep(0.2)

        if self.iteration == 1:
            msg = Float32MultiArray()
            msg.data = high.tolist()
            self.pub_high.publish(msg)
            rospy.loginfo('Published HIGH point')

        elif self.iteration == 2:
            msg = Float32MultiArray()
            msg.data = low.tolist()
            self.pub_low.publish(msg)
            rospy.loginfo('Published LOW point')

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.run_analysis:
                self.run_analysis = False
                self.process()

            rate.sleep()


# =====================
# MAIN
# =====================

if __name__ == '__main__':
    node = PCAnalyzer()
    node.run()