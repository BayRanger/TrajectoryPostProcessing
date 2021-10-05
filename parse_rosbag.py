#!/usr/bin/env python
# coding=utf-8

import rosbag
import os
import argparse
import numpy as np
import argparse


def getTopicList(directory,filename):
    bag = rosbag.Bag(directory+ filename)
    topics = bag.get_type_and_topic_info()[1].keys()
    types = []
    for i in range(0,len(bag.get_type_and_topic_info()[1].values())):
        types.append(bag.get_type_and_topic_info()[1].values()[i][0])
    print(types)

def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix
def extract(bagfile, pose_topic, msg_type, out_filename):
    n = 0
    f = open(out_filename, 'w')
    f.write('# timestamp tx ty tz qx qy qz qw\n')
    with rosbag.Bag(bagfile, 'r') as bag:
        for (topic, msg, ts) in bag.read_messages(topics=str(pose_topic)):
            if msg_type == "PoseWithCovarianceStamped":
                f.write('%.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f\n' %
                        (msg.header.stamp.to_sec(),
                         msg.pose.pose.position.x, msg.pose.pose.position.y,
                         msg.pose.pose.position.z,
                         msg.pose.pose.orientation.x,
                         msg.pose.pose.orientation.y,
                         msg.pose.pose.orientation.z,
                         msg.pose.pose.orientation.w))
            elif msg_type == "odometry":
                quat =[ msg.pose.pose.orientation.x,
                         msg.pose.pose.orientation.y,
                         msg.pose.pose.orientation.z,
                         msg.pose.pose.orientation.w]
		'''
                R = quaternion_rotation_matrix(quat)
                f.write('%.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f\n' %
                        (msg.header.stamp.to_sec(),
                        R[0][0],R[0][1],R[0][2],msg.pose.pose.position.x,
                        R[1][0],R[1][1],R[1][2],msg.pose.pose.position.y,
                        R[2][0],R[2][1],R[2][2],msg.pose.pose.position.z
                        ))
                '''
                f.write('%.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f\n' %
                        (msg.header.stamp.to_sec(),
                         msg.pose.pose.position.x, msg.pose.pose.position.y,
                         msg.pose.pose.position.z,
                         msg.pose.pose.orientation.x,
                         msg.pose.pose.orientation.y,
                         msg.pose.pose.orientation.z,
                         msg.pose.pose.orientation.w))
                
            elif msg_type == "PoseStamped":
                f.write('%.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f\n' %
                        (msg.header.stamp.to_sec(),
                         msg.pose.position.x, msg.pose.position.y,
                         msg.pose.position.z,
                         msg.pose.orientation.x, msg.pose.orientation.y,
                         msg.pose.orientation.z, msg.pose.orientation.w))
            else:
                assert False, "Unknown message type"
            n += 1
    print('wrote ' + str(n) + ' odom messages to the file: ' + out_filename)

def parse_args():
    args = argparse.ArgumentParser()
    args.add_argument('--file', type=str, required=True,help="rosbag file")
    args.add_argument('--topic_name', type=str, required=True,help="Topic of the odometry message")
    args.add_argument('--topic_type', type=str, required=True,help="Type of the odometry message")
    args.add_argument('--output_file', type=str, required=True,help="output file")
    return args.parse_args()
    
if __name__ =='__main__':
    # directory = '/media/chahe/Transcend/changxin/fusion_data/hk'#'/home/chahe/project/sensor-fusion-for-localization-and-mapping/workspace/data/allan_variance_analysis'
    # filename = '/urban_sample_daytime.bag'
    args=parse_args();
    extract(args.file,args.topic_name ,args.topic_type,args.output_file)