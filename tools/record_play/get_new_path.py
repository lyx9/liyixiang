#!/usr/bin/env python
# Software License Agreement (BSD License)
###########################
#
#   get path data from recorded data
#
###########################
"""
get Path data
"""

import rospy
import sys
import math
import matplotlib.pyplot as plt
import numpy as np
import tf
import thread
import signal
from std_msgs.msg import String
from out import global_adc_status_pb2
from out import planning_pb2
from numpy import genfromtxt
import argparse
class RtkPlayer(object):
    """
    rtk player class
    """

    def __init__(self):
        self.firstvalid = False
        file_path = open('filename_path', 'r').readline().split('\n')[0]
        try:
            origin_file = open(file_path, 'r')
        except:
            print "Cannot find file: " + file_path
            sys.exit(0)

        self.file = open('path_new.csv', 'w')
        self.file.write("x,y,z,speed,acceleration,curvature,"\
                        "curvature_change_rate,time,theta,gear,s\n")
        self.pathfile_data =\
                genfromtxt(origin_file, delimiter=',', skip_footer = 1, skip_header = 1)
        self.x_ori = []
        self.y_ori = []
        self.theta_ori = []
        self.curv_ori = []
        self.x = []
        self.y = []
        self.z = []
        self.speed = []
        self.acceleration = []
        self.curv = []
        self.curvature_change_rate = []
        self.time = []
        self.theta = []
        self.gear = []
        self.s = []
        self.filter_size = 50
        self.calu_size = 5
        self.x_filter = []
        self.y_filter = []
        self.theta_filter = []
        self.unify_theta = []
        self.curv_calu = []
        self.dtheta = []
        self.curvature_change_rate_cal = []

    def theta_unify(self, angle_in):
        """
        unify angle
        """
        angle_out = angle_in + math.pi
        angle_out = math.fmod(angle_out, 2 * math.pi) - math.pi
        while angle_out < -math.pi:
            angle_out = angle_out + 2.0 * math.pi
        while angle_out > math.pi:
            angle_out = angle_out - 2.0 * math.pi
        return angle_out

    def get_data(self):
        """
        get path data from recoard data
        """
        for i in range(len(self.pathfile_data)):
            self.x_ori.append(self.pathfile_data[i, 0])
            self.y_ori.append(self.pathfile_data[i, 1])
            self.z.append(self.pathfile_data[i, 2])
            self.speed.append(self.pathfile_data[i, 3])
            self.acceleration.append(self.pathfile_data[i, 4])
            self.curv_ori.append(self.pathfile_data[i, 5])
            self.curvature_change_rate.append(self.pathfile_data[i, 6])
            self.time.append(self.pathfile_data[i, 7])
            self.theta_ori.append(self.pathfile_data[i, 8])
            self.gear.append(self.pathfile_data[i, 9])
            self.s.append(self.pathfile_data[i, 10])
            if len(self.x_filter) < self.filter_size:
                self.x_filter.append(self.pathfile_data[i, 0])
                self.y_filter.append(self.pathfile_data[i, 1])
                if i > 0:
                    if self.pathfile_data[i, 8] - self.theta_filter[0] > math.pi:
                        filter_theta = self.pathfile_data[i, 8] - 2 * math.pi
                    elif self.pathfile_data[i, 8] - self.theta_filter[0] < -math.pi:
                        filter_theta = self.pathfile_data[i, 8] + 2 * math.pi
                    else:
                        filter_theta = self.pathfile_data[i, 8]
                else:
                    filter_theta = self.pathfile_data[i, 8]
            else:
                self.x_filter.pop(0)
                self.x_filter.append(self.pathfile_data[i, 0])
                self.y_filter.pop(0)
                self.y_filter.append(self.pathfile_data[i, 1])
                self.theta_filter.pop(0)
                if self.pathfile_data[i, 8] - self.theta_filter[0] > math.pi:
                    filter_theta = self.pathfile_data[i, 8] - 2 * math.pi
                elif self.pathfile_data[i, 8] - self.theta_filter[0] < -math.pi:
                    filter_theta = self.pathfile_data[i, 8] + 2 * math.pi
                else:
                    filter_theta = self.pathfile_data[i, 8]
            self.theta_filter.append(filter_theta)
            self.x.append(np.mean(self.x_filter))
            self.y.append(np.mean(self.x_filter))
            self.theta.append(np.mean(self.theta_filter))

    def calcu_curvature_use_ori_theta(self):
        """
        calculate curvature use origin theta
        """
        for i in range(len(self.theta)):
            if i < self.calu_size:
                if i == 0:
                    self.curv.append(0)
                    self.curvature_change_rate_cal.append(0)
                else:
                    unify_angle = self.theta_unify(self.theta[i] - self.theta[0])
                    self.unify_theta.append(unify_angle)
                    curvature = unify_angle / (self.s[i] - self.s[0])
                    if self.speed[i] != 0:
                        self.curvature_change_rate_cal.append((curvature - self.curv[-1]) / \
                                (self.speed[i] * 0.01 * i))
                    else:
                        self.curvature_change_rate_cal.append(0)
                    self.curv.append(curvature)
            else:
                unify_angle = self.theta_unify(self.theta[i] - self.theta[i - self.calu_size])
                self.unify_theta.append(unify_angle)
                curvature = unify_angle / (self.s[i] - self.s[i - self.calu_size])
                if self.speed[i] != 0:
                    self.curvature_change_rate_cal.append((curvature - self.curv[-1]) / \
                            (self.speed[i] * 0.01 * self.calu_size))
                else:
                    self.curvature_change_rate_cal.append(0)
                self.curv.append(curvature)
            self.file.write("%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s\n"%
                    (self.x_ori[i],
                     self.y_ori[i],
                     self.z[i],
                     self.speed[i],
                     self.acceleration[i],
                     self.curv[i],
                     self.curvature_change_rate_cal[i],
                     self.time[i],
                     self.theta_ori[i],
                     self.gear[i],
                     self.s[i]))


def main():
    """
    Main rosnode
    """
    parser = argparse.ArgumentParser(description='Generate Planning Trajectory from Data File')
    args = vars(parser.parse_args())
    player = RtkPlayer()
    player.get_data()
    player.calcu_curvature_use_ori_theta()

    plt.figure(figsize=(10, 5))
    plt.plot(player.theta_ori, 'Green', label='theta')
    plt.plot(player.theta, 'Red', label='theta')
    plt.savefig('theta')

    plt.figure(figsize=(10, 5))
    plt.plot(player.curv_ori, 'Green', label='curvature')
    plt.plot(player.curv, 'Red', label='curvature')
    plt.plot(player.curv_calu, 'Blue', label='curvature')
    plt.savefig('curvature')

if __name__ == '__main__':
    main()
