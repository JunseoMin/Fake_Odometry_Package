#! /usr/bin/python3

import rospy

from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import pyproj
import tf
import numpy as np

import math
from queue import Queue


class MakeOdom:
    def __init__(self):
        rospy.init_node('make_odom', anonymous=True)
        
        rospy.Subscriber("/gps", GPSMessage, self.gpsCB)
        rospy.Subscriber("/imu", Imu, self.imuCB)
        self.odom_pub = rospy.Publisher('odom',Odometry, queue_size=1)

        self.odom = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_link'

        self.is_gps = False
        self.is_imu = False

        self.x_offset = 334209.0789507285
        self.y_offset = 4143042.501430065
        
        self.prev_gps = None

        self.q_size = 10
        self.prev_gap_queue = Queue(self.q_size)
        self.curr = None


        self.proj_UTM = pyproj.Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            if self.is_gps and self.is_imu:    
                self.set_gps()
                self.makeOdomMsg()
                self.makeOdomTF()
                self.prev_gps = self.curr_gps
                rate.sleep()

    def makeOdomTF(self):
        br =tf.TransformBroadcaster()
        position_x = self.xy_zone[0] - self.x_offset
        position_y = self.xy_zone[1] - self.y_offset
        
        br.sendTransform((position_x, position_y, 0),
                        tf.transformations.quaternion_from_euler(self.euler_data[0], self.euler_data[1], self.euler_data[2]),
                        rospy.Time.now(),
                        "base_link",
                        "odom")
                        
    def makeOdomMsg(self):
        quaternion = tf.transformations.quaternion_from_euler(self.euler_data[0], self.euler_data[1], self.euler_data[2])
        self.odom.pose.pose.position.x=self.xy_zone[0] - self.x_offset
        self.odom.pose.pose.position.y=self.xy_zone[1] - self.y_offset
        self.odom.pose.pose.position.z=0
        self.odom.pose.pose.orientation.x=quaternion[0]
        self.odom.pose.pose.orientation.y=quaternion[1]
        self.odom.pose.pose.orientation.z=quaternion[2]
        self.odom.pose.pose.orientation.w=quaternion[3]
        self.odom_pub.publish(self.odom)

    def gpsCB(self, data):
        self.curr_gps = data        
        self.is_gps = True

    def imuCB(self, data):
        self.euler_data = tf.transformations.euler_from_quaternion((data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
        self.is_imu = True

    def set_gps(self):
        copy_Q = self.prev_gap_queue
    
        if self.prev_gps == None:
            self.prev_gps = self.curr_gps
        
        ############################################# Set Queue initial
        if not self.prev_gap_queue.full():
            long_gap = self.curr_gps.longitude - self.prev_gps.longitude
            lat_gap = self.curr_gps.latitude - self.prev_gps.latitude

            gap = np.array([long_gap,lat_gap,0.0],dtype=float)

            self.prev_gap_queue.put(gap)

            self.xy_zone = self.proj_UTM(self.curr_gps.longitude, self.curr_gps.latitude)
            return
        ############################################## Set full Queue
        
        # Queue fulled
        nearest = copy_Q.get()
        # GPS overshooting
        if (math.abs(self.curr_gps.longitude - self.prev_gps.longitude) > 0.0001) or math.abs((self.curr_gps.latitude - self.prev_gps.latitude) > 0.0001):
            self.curr_gps.longitude = self.prev_gps.longitude + nearest[0]
            self.curr_gps.latitude = self.prev_gps.latitude + nearest[1]
            self.xy_zone = self.proj_UTM(self.curr_gps.longitude, self.curr_gps.latitude)
            gap = np.array([nearest[0],nearest[1],0.0],dtype=float)
            self.prev_gap_queue.get()
            self.prev_gap_queue.put(gap)

        else:
            ######################################
            self.prev_gap_queue.get()
            self.curr[0] = self.curr_gps.longitude
            self.curr[1] = self.curr_gps.latitude

            long_gap = self.curr[0] - self.prev_gps.longitude
            lat_gap = self.curr[1] - self.prev_gps.latitude

            gap = np.array([long_gap,lat_gap,0.0],dtype=float)

            self.prev_gap_queue.put(gap)
            self.xy_zone = self.proj_UTM(self.curr_gps.longitude, self.curr_gps.latitude)

            ##############################################################


def main():
    try:
        makeodom=MakeOdom()    
    except rospy.ROSInterruptException:
        pass
    
if __name__ == '__main__':
    main()