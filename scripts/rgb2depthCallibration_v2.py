#!/usr/bin/env python

import cv2 as cv
import numpy as np
import open3d as o3d

import copy
import math
import rospy
import struct
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
from point_cloud.msg import highLowPoint
from std_msgs.msg import Bool
from geometry_msgs.msg import Polygon, Point32
from cv_bridge import CvBridge


clicked_point = None

def mouse_callback(event, x, y, flags, param):
    global clicked_point
    if event == cv.EVENT_LBUTTONDOWN:
        clicked_point = (x, y)
        print('Left click at:', x, y)


if __name__ == '__main__':

    rospy.init_node('rgb2depthCallibration', anonymous=True)

    bridge = CvBridge()

    img = rospy.wait_for_message('/rgbd/rgb/image_color', Image)
    depth = rospy.wait_for_message('/rgbd/depth/image_raw', Image)

    img = bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
    depth = bridge.imgmsg_to_cv2(depth, 'passthrough')

    fx = 554.254691191187
    fy = 554.254691191187
    cx = 320.5
    cy = 240.5
    height = 480
    width = 640

    pc = []
    colors = []
    
    # pixel -> point index map
    index_map = -np.ones((height, width), dtype=int)

    idx = 0

    for v in range(height):
        for u in range(width):

            z = depth[v, u]

            if np.isnan(z) or np.isinf(z) or z <= 0:
                continue

            if z < 2:

                x = (u - cx) * z / fx
                y = (v - cy) * z / fy

                pc.append([x, -y, -z])
                colors.append([0, 0, 0])

                index_map[v, u] = idx
                idx += 1

    pc = np.array(pc)
    colors = np.array(colors)

    o3d_pc = o3d.geometry.PointCloud()
    o3d_pc.points = o3d.utility.Vector3dVector(pc)
    o3d_pc.colors = o3d.utility.Vector3dVector(colors)

    vis = o3d.visualization.Visualizer()
    vis.create_window('Point cloud')
    vis.add_geometry(o3d_pc)

    cv.namedWindow('RGB Image')
    cv.namedWindow('Depth Image')
    cv.setMouseCallback('RGB Image', mouse_callback)

    while True:

        depth_vis = cv.normalize(depth, None, 0, 255, cv.NORM_MINMAX)
        depth_vis = depth_vis.astype('uint8')
        depth_vis = cv.cvtColor(depth_vis, cv.COLOR_GRAY2BGR)

        if clicked_point is not None:

            u, v = clicked_point
            cv.circle(depth_vis, (u,v), 4, (0,0,255), -1)

            idx = index_map[v, u]

            if idx >= 0:
                colors[idx] = [1, 0, 0]  # red
                o3d_pc.colors = o3d.utility.Vector3dVector(colors)
                vis.update_geometry(o3d_pc)

        vis.poll_events()
        vis.update_renderer()

        cv.imshow('RGB Image', img)
        cv.imshow('Depth Image', depth_vis)

        if cv.waitKey(10) == 27:
            break

        if cv.getWindowProperty('RGB Image', cv.WND_PROP_VISIBLE) < 1:
            break

    vis.destroy_window()
    cv.destroyAllWindows()