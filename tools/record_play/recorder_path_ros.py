#!/usr/bin/env python
# Software License Agreement (BSD License)

###########################
#
#   Record GPS And IMU Data
#
###########################
"""
Record GPS and IMU data
"""

import rospy
import math
import argparse
import signal
import tf
import sys
import os
from std_msgs.msg import String
from out import global_adc_status_pb2


class RtkRecord(object):
    """
    rtk recording class
    """

    def __init__(self, constspeed, carbranch, light):
        try:
            file_path = open('filename_path', 'r').readline().split('\n')[0]
            self.file = open(file_path, 'w')
        except:
            print "cannot find file"
            sys.exit(0)

        self.file.write("x,y,z,speed,acceleration,curvature,"
                        "curvature_change_rate,time,theta,gear,s,left_turn,right_turn\n")

        self.entity = global_adc_status_pb2.Status()
        self.cars = 0
        self.carcurvature = 0
        self.startmoving = False
        self.terminating = False
        self.constspeed = constspeed
        self.recordlight = light == 't'

        if carbranch == 'lincoln':
            self.maxsteer = 470
            self.steerratio = 16
            self.wheelbase = 2.85
        elif carbranch == 'byd':
            self.maxsteer = 540
            self.steerratio = 17.5
            self.wheelbase = 2.67
        elif carbranch == 'kinglong':
            self.maxsteer = 540
            self.steerratio = 17.2
            self.wheelbase = 2.75
        else:
            print "car branch does not exist"
            sys.exit(0)

    def callback(self, data):
        """
        New message received
        """
        if self.terminating:
            return

        self.entity.ParseFromString(data.data)

        # pose
        quat = (self.entity.pose.orientation.qx,
                self.entity.pose.orientation.qy,
                self.entity.pose.orientation.qz,
                self.entity.pose.orientation.qw)
        heading = tf.transformations.euler_from_quaternion(quat)
        cartheta = (heading[2] + math.pi / 2 + math.pi) % (2 * math.pi) - math.pi
        carsteer = self.entity.chassis.steering_percentage
        curvature = math.tan(math.radians(carsteer / 100 * self.maxsteer) / self.steerratio) \
                    / self.wheelbase

        # speed
        if self.entity.chassis.speed_mps > 0:
            # constant speed
            if self.constspeed > 0:
                carspeed = self.constspeed
            else:
                carspeed = self.entity.chassis.speed_mps
            # acceleration
            if self.entity.chassis.gear_location == global_adc_status_pb2.GEAR_REVERSE:
                caracceleration = -self.entity.pose.linear_acceleration.y
            else:
                caracceleration = self.entity.pose.linear_acceleration.y
            # curvature change rate
            carcurvature_change_rate = (curvature - self.carcurvature) / (carspeed * 0.01)
        else:
            carspeed = 0
            caracceleration = 0
            carcurvature_change_rate = 0

        # other info
        self.carcurvature = curvature
        self.cars += carspeed * 0.01
        cartime = self.entity.header.timestamp_sec
        cargear = self.entity.chassis.gear_location
        left_turn = self.entity.chassis.left_turn_signal
        right_turn = self.entity.chassis.right_turn_signal

        # start moving
        if carspeed > 0 and not self.startmoving:
            print "Start Recording"
            self.startmoving = True

        # write
        if self.startmoving:
            if self.recordlight:
                self.file.write("%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s\n" %
                                (self.entity.pose.position.x,
                                 self.entity.pose.position.y,
                                 self.entity.pose.position.z,
                                 carspeed,
                                 caracceleration,
                                 curvature,
                                 carcurvature_change_rate,
                                 cartime,
                                 cartheta,
                                 cargear,
                                 self.cars,
                                 left_turn,
                                 right_turn))
            else:
                self.file.write("%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s\n" %
                                (self.entity.pose.position.x,
                                 self.entity.pose.position.y,
                                 self.entity.pose.position.z,
                                 carspeed,
                                 caracceleration,
                                 curvature,
                                 carcurvature_change_rate,
                                 cartime,
                                 cartheta,
                                 cargear,
                                 self.cars))

    def quit(self, signum, frame):
        """
        shutdown rosnode
        """
        print "Shutting Down..."
        self.terminating = True
        self.file.close()
        rospy.sleep(0.1)
        os._exit(0)


def main():
    """
    Main rosnode
    """
    parser = argparse.ArgumentParser(description='Record trajectory into garage.csv')
    parser.add_argument('-s', '--constspeed', help='record as constant speed',
                                              type=float, default='0')
    parser.add_argument('-b', '--carbranch', help='car branch', default='lincoln')
    parser.add_argument('-l', '--light', help='record car light (t/f)', default='f')
    args = vars(parser.parse_args())

    recorder = RtkRecord(args['constspeed'], args['carbranch'], args['light'])
    rospy.init_node('rtk_recorder', anonymous=True)
    rospy.Subscriber('/pnc/carstatus', String, recorder.callback)

    signal.signal(signal.SIGINT, recorder.quit)

    rospy.spin()


if __name__ == '__main__':
    main()
