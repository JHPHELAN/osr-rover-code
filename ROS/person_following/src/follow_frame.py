#!/usr/bin/env python

import rospy
import math
import tf2_ros

from geometry_msgs.msg import Twist
# from ar_track_alvar_msgs.msg import AlvarMarkers
from fiducial_msgs.msg import FiducialTransformArray


class FrameFollower(object):
    timeout = rospy.Duration(0.6)

    def __init__(self):

        self.target_frame = rospy.get_param("target_frame", "follow_target")
        self.last_received = rospy.Time.now() - self.timeout
	self.last_slown_down = self.last_received
        self.safe_distance = 0.7
        self.low_pass_frac_x = 0.05
        self.low_pass_frac_z = 0.1
        self.p_gain_x = 1.2
        self.p_gain_z = 0.3
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
        self.ar_tag_sub = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, callback=self.ar_frame_cb)
	rospy.on_shutdown(self.shutdown)
        # tfBuffer = tf2_ros.Buffer()
        # listener = tf2_ros.TransformListener(tfBuffer)

        self.cmd_twist = Twist()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            # try:
                # trans = tfBuffer.lookup_transform("base_link", self.target_frame, rospy.Time())
            # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                # rospy.logwarn("While looking up transform in frame follower: {}".format(e))
                # cmd_twist = Twist()
                # rate.sleep()
                # continue

            # use diff in y to control steering angle. If dy > 0 -> positive twist
            # cmd_twist.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
            # cmd_twist.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

	    now = rospy.Time.now()
            if now - self.last_received >= self.timeout \
               and now - self.last_slown_down > self.timeout/4.:
                if self.cmd_twist.linear.x > 0.01 and self.cmd_twist.angular.z > 0.005:
                    self.cmd_twist.linear.x *= 0.7
		    self.cmd_twist.angular.z *= 0.3
		    self.last_slown_down = now
		else:
                    self.cmd_twist = Twist()
               	    self.vel_pub.publish(self.cmd_twist)
                if not self.cmd_twist == Twist():
               	    self.vel_pub.publish(self.cmd_twist)

            # rospy.logdebug_throttle(1, self.cmd_twist)

            rate.sleep()

    def ar_frame_cb(self, msg):
        try:
            transform = msg.transforms[0].transform  # assume only one for now
            rospy.logdebug("received fiducial!")
            self.last_received = msg.header.stamp
            # self.cmd_twist.angular.z = 0.02 * math.atan2(transform.pose.pose.position.y, (transform.pose.pose.position.x - self.safe_distance))
            self.cmd_twist.angular.z = self.low_pass_frac_z * self.cmd_twist.angular.z \
                                       + (1. - self.low_pass_frac_z) * self.p_gain_z * math.atan2(transform.translation.y, transform.translation.z)
            # self.cmd_twist.linear.x = 0.2 * math.sqrt((transform.pose.pose.position.x - self.safe_distance) ** 2 + transform.pose.pose.position.y ** 2)
            self.cmd_twist.linear.x = self.low_pass_frac_x * self.cmd_twist.linear.x \
                                      + (1. - self.low_pass_frac_x) * self.p_gain_x * (transform.translation.z - self.safe_distance)
            self.vel_pub.publish(self.cmd_twist)
        except IndexError as ke:
            rospy.logdebug_throttle(2, "no fiducial detected.")
        except Exception as e:
            rospy.logdebug("exception while receiving fiducial: {}".format(e))

    def shutdown(self):
        self.vel_pub.publish(Twist())


if __name__ == '__main__':
    rospy.init_node('frame_follower', log_level=rospy.DEBUG)
    rospy.logdebug("Starting the frame following node")
    follower = FrameFollower()
    # rospy.spin()
