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
import cybertron
import time
import math
import signal
import argparse
import os
import sys
from filters import Filters
from out import global_adc_status_pb2


class RtkRecord(object):
    """
    rtk recording class
    """

    def __init__(self, constspeed, carbranch, light):
        # load data
        try:
            file_path = open('filename_path', 'r').readline().split('\n')[0]
            self.file = open(file_path, 'w')
        except Exception:
            print "cannot find file"
            sys.exit(0)
        self.file.write("x,y,z,speed,acceleration,curvature,"
                        "curvature_change_rate,time,theta,gear,s,left_turn,right_turn\n")

        self.cars = 0
        self.carcurvature = 0
        self.startmoving = False
        self.terminating = False
        self.constspeed = constspeed
        self.acc_filter = Filters(10)
        self.recordlight = light == 't'

        if carbranch == 'lincoln':
            print "current branch is lincoln"
            self.maxsteer = 470
            self.steerratio = 16
            self.wheelbase = 2.85
        elif carbranch == 'byd':
            print "current branch is byd"
            self.maxsteer = 540
            self.steerratio = 17.5
            self.wheelbase = 2.67
        elif carbranch == 'kinglong':
            print "current branch is kinglong"
            self.maxsteer = 540
            self.steerratio = 17.2
            self.wheelbase = 2.75
        elif carbranch == 'h7phev':
            print "current branch is h7phev"
            self.maxsteer = 540
            self.steerratio = 15.33
            self.wheelbase = 2.96
        elif carbranch == 'daimler':
            print "current branch is daimler"
            self.maxsteer = 38.49
            self.steerratio = 1.0
            self.wheelbase = 3.43
        elif carbranch == 'hongqi':
            print "current branch is hongqi"
            self.maxsteer = 480
            self.steerratio = 16.18
            self.wheelbase = 2.75
        else:
            print "car branch does not exist"
            print "Usage: python recorder_path_cyber.py -b lincoln/byd/kinglong/h7phev/daimler"
            sys.exit(0)

    def callback(self, data):
        """
        New message received
        """
        if self.terminating:
            return

        # pose
        qx = data.pose.orientation.qx
        qy = data.pose.orientation.qy
        qz = data.pose.orientation.qz
        qw = data.pose.orientation.qw
        heading = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        cartheta = (heading + math.pi / 2 + math.pi) % (2 * math.pi) - math.pi
        carsteer = data.chassis.steering_percentage
        curvature = math.tan(math.radians(carsteer / 100 * self.maxsteer) / self.steerratio) \
                    / self.wheelbase

        # speed
        if data.chassis.speed_mps > 0:
            # constant speed
            if self.constspeed > 0:
                carspeed = self.constspeed
            else:
                carspeed = data.chassis.speed_mps
            # acceleration
            if data.chassis.gear_location == global_adc_status_pb2.GEAR_REVERSE:
                caracceleration = -data.pose.linear_acceleration.y
            else:
                caracceleration = data.pose.linear_acceleration.y
            caracceleration = self.acc_filter.mean_filter(caracceleration)
            # curvature change rate
            carcurvature_change_rate = (curvature - self.carcurvature) / (carspeed * 0.01)
        else:
            carspeed = 0
            caracceleration = 0
            carcurvature_change_rate = 0

        # other info
        self.carcurvature = curvature
        self.cars += carspeed * 0.01
        cartime = data.header.timestamp_sec
        cargear = data.chassis.gear_location
        left_turn = data.chassis.left_turn_signal
        right_turn = data.chassis.right_turn_signal

        # start moving
        if carspeed > 0 and not self.startmoving:
            print "Start Recording"
            self.startmoving = True

        # write
        if self.startmoving:
            if self.recordlight:
                self.file.write("%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s\n" %
                                (data.pose.position.x,
                                 data.pose.position.y,
                                 data.pose.position.z,
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
                                (data.pose.position.x,
                                 data.pose.position.y,
                                 data.pose.position.z,
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
        shutdown the keypress thread
        """
        print "Shutting Down..."
        self.terminating = True
        self.file.close()
        time.sleep(0.1)
        os._exit(0)


def main():
    """
    Main cybernode
    """
    parser = argparse.ArgumentParser(description='Record trajectory into garage.csv')
    parser.add_argument('-s', '--constspeed', help='record as constant speed',
                                              type=float, default='0')
    parser.add_argument('-b', '--carbranch', help='car branch', default='lincoln')
    parser.add_argument('-l', '--light', help='record car light (t/f)', default='f')
    args = vars(parser.parse_args())

    cybertron.init.log_init()
    recorder = RtkRecord(args['constspeed'], args['carbranch'], args['light'])
    node = cybertron.node.Node("rtk_recorder")
    node.create_reader('/pnc/carstatus', global_adc_status_pb2.Status, recorder.callback)

    signal.signal(signal.SIGINT, recorder.quit)

    while 1:
        node.spin_once()


if __name__ == '__main__':
    main()
