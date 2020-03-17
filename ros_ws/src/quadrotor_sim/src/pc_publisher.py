#!/usr/bin/env python
import rospy
from plyfile import PlyData, PlyElement
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import time


def xyzrgb_array_to_pointcloud2(points, colors, seq=None):
    '''
    Create a sensor_msgs.PointCloud2 from an array
    of points.
    '''
    msg = PointCloud2()
    assert(points.shape == colors.shape)
    N = len(points)

    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'world'

    if seq:
        msg.header.seq = seq
    if len(points.shape) == 3:
        msg.height = points.shape[1]
        msg.width = points.shape[0]
    else:
        msg.height = 1
        msg.width = N
    xyzrgb = np.array(np.hstack([points, colors]), dtype=np.float32)
    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('r', 12, PointField.FLOAT32, 1),
        PointField('g', 16, PointField.FLOAT32, 1),
        PointField('b', 20, PointField.FLOAT32, 1),
    ]
    msg.is_bigendian = False
    msg.point_step = 24
    msg.row_step = msg.point_step * N
    msg.is_dense = True;
    msg.data = xyzrgb.tostring()
    return msg
def publish_pointcloud(points, colors):
    topic_name='/fountain/pointcloud2'
    msg = xyzrgb_array_to_pointcloud2(points, colors)
    pub = rospy.Publisher(topic_name, PointCloud2, queue_size=1)
    pub.publish(msg)



if __name__ == '__main__':
	rospy.init_node('pointcloud_node')
	fountain_pc = PlyData.read('/home/rus/Desktop/LTU/quadrotor_simulator/data/fountain.ply')
	vertex = fountain_pc['vertex']
	(x, y, z, r,g,b) = (vertex[t] for t in ('x', 'y', 'z', 'red', 'green', 'blue'))
	points = np.vstack([x,y,z]).T
	colors = np.vstack([r,g,b]).T

	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		publish_pointcloud(points, colors)
    	time.sleep(1.0)