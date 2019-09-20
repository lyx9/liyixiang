#!/usr/bin/env python
# Software License Agreement (BSD License)

###########################
#
#   Generate planning path from recorded data
#
###########################
"""
Generate Planning Path
"""

import rospy
import sys
import thread
import signal
from std_msgs.msg import String
from out import global_adc_status_pb2
from out import planning_pb2
from out import pad_msg_pb2
from numpy import genfromtxt
import argparse


class RtkPlayer(object):
    """
    rtk player class
    """
    def __init__(self, light):
        # load data
        try:
            file_path = open('filename_path', 'r').readline().split('\n')[0]
            self.file = open(file_path, 'r')
        except:
            print "Cannot find file"
            sys.exit(0)
        self.pathfile_data = genfromtxt(self.file, delimiter=',', skip_footer=1, skip_header=1)
        self.data_length = len(self.pathfile_data)
        self.process_loop_data()

        # initialize
        self.carstatus = global_adc_status_pb2.Status()
        self.padmessage = pad_msg_pb2.PadMessage()
        self.sequence_num = 0
        self.curr_index = 0
        self.planning_length = 1000
        self.curr_planning = 'none'
        self.prev_planning = 'none'
        self.estop = False
        self.terminating = False
        self.carstatus_received = False
        self.padmessage_received = False
        self.have_light = light == 't'
        self.planning_pub = rospy.Publisher('/pnc/planning', String, queue_size=1)
        thread.start_new_thread(self.keypress, (1,))

    def process_loop_data(self):
        """
        process loop data and get connection point
        """
        x = self.pathfile_data[0, 0]
        y = self.pathfile_data[0, 1]
        start_idx = self.data_length // 2
        end_idx = self.data_length
        self.connect_index = self.search_closest_point(x, y, start_idx, end_idx)
        self.overlap_length = self.data_length - self.connect_index
        if self.overlap_length == 1:
            print "Error: no overlap"
        elif self.overlap_length < 1000:
            print "Warnning: overlap length is less than 1000"
        else:
            print "total length: %d", self.data_length
            print "connection point: %d", self.connect_index
        self.overlap_length += self.connect_index // 8

    def search_closest_point(self, x, y, start_idx, end_idx):
        """
        get the shortest length
        """
        shortestdist = float('inf')
        for i in range(start_idx, end_idx):
            dist = (x - self.pathfile_data[i, 0]) ** 2 + (y - self.pathfile_data[i, 1]) ** 2
            if dist <= shortestdist:
                shortestdist = dist
                index = i
        return index

    def keypress(self, id):
        """
        key press action
        """
        while True:
            estop = raw_input()
            if estop.lower() == 's':
                self.estop = True
                print 'ESTOP!'
            elif estop.lower() == 'r':
                self.estop = False
                print 'RESET ESTOP!'

    def callback_padmessage(self, data):
        """
        Pad Terminal Received
        """
        self.padmessage.ParseFromString(data.data)
        if self.padmessage.action == pad_msg_pb2.START:
            self.padmessage_received = True
            print "current padmessage: " + str(self.padmessage_received)

    def callback_carstatus(self, data):
        """
        New Car Status Received
        """
        self.carstatus.ParseFromString(data.data)
        self.carx = self.carstatus.pose.position.x
        self.cary = self.carstatus.pose.position.y
        self.carz = self.carstatus.pose.position.z
        self.carstatus_received = True

    def check_position(self):
        """
        check current position and publish planning
        """
        if not self.carstatus_received:
            print "No car status, unable to generate planning"
            return

        # check current position
        if self.curr_index < self.overlap_length:
            self.curr_index = self.search_closest_point(self.carx, self.cary,
                                                        self.curr_index, self.connect_index)
        else:
            self.curr_index = self.search_closest_point(self.carx, self.cary,
                                                        self.curr_index, self.data_length)
        # determine start and end point
        if self.curr_index < self.connect_index // 2:
            self.start = max(self.curr_index - 10, 0)
            self.end = self.data_length - self.overlap_length
            self.curr_planning = 'first half'
        else:
            if self.curr_index > self.connect_index:
                self.curr_index = self.curr_index - self.connect_index
            self.start = max(self.curr_index - 10, 0)
            self.end = self.data_length
            self.curr_planning = 'second half'

        # if finish half loop or get padmsg to start, publish planning
        if self.curr_planning != self.prev_planning or self.padmessage_received:
            self.publish_planningmsg()
            self.prev_planning = self.curr_planning
            self.padmessage_received = False
        # print "current index: " + str(self.curr_index)

    def localization(self):
        """
        localize current position
        """
        if not self.carstatus_received:
            print "No car status, unable to generate planning"
            return

        # check current position, determine start and end point
        if self.curr_index < self.overlap_length:
            self.curr_index = self.search_closest_point(self.carx, self.cary,
                                                        self.curr_index, self.connect_index)
        else:
            self.curr_index = self.search_closest_point(self.carx, self.cary,
                                                        self.curr_index, self.data_length)
            if self.start >= self.connect_index:
                self.start = 0
        self.start = max(self.curr_index - 10, 0)
        self.end = min(self.start + self.planning_length, self.data_length)

    def publish_planningmsg(self):
        """
        Generate New Path
        """
        if not self.carstatus_received:
            print "No car status, unable to generate planning"
            return

        planningdata = planning_pb2.ADCTrajectory()
        planningdata.header.module_name = "planning"
        planningdata.header.sequence_num = self.sequence_num

        for i in range(self.start, self.end):
            adc_point = planning_pb2.ADCTrajectoryPoint()
            adc_point.x = self.pathfile_data[i, 0]
            adc_point.y = self.pathfile_data[i, 1]
            adc_point.z = self.pathfile_data[i, 2]
            adc_point.speed = self.pathfile_data[i, 3]
            adc_point.acceleration_s = self.pathfile_data[i, 4]
            adc_point.curvature = self.pathfile_data[i, 5]
            adc_point.curvature_change_rate = self.pathfile_data[i, 6]
            time_begin2cur = self.pathfile_data[i, 7] - self.pathfile_data[self.start, 7]
            time_begin2closest = self.pathfile_data[self.curr_index, 7] - \
                                 self.pathfile_data[self.start, 7]
            adc_point.relative_time = time_begin2cur - time_begin2closest
            adc_point.theta = self.pathfile_data[i, 8]
            adc_point.s = self.pathfile_data[i, 10] - self.pathfile_data[self.start, 10]
            adc_point.accumulated_s = adc_point.s
            planningdata.adc_trajectory_point.extend([adc_point])
            if self.have_light:
                if self.pathfile_data[i, 11] and not self.pathfile_data[i, 12]:
                    planningdata.signals.signal.extend([planning_pb2.ADCSignals().LEFT_TURN])
                elif self.pathfile_data[i, 12] and not self.pathfile_data[i, 11]:
                    planningdata.signals.signal.extend([planning_pb2.ADCSignals().RIGTH_TURN])

        if self.estop:
            planningdata.estop.is_estop = True
        else:
            planningdata.estop.is_estop = False

        planningdata.total_path_length = self.pathfile_data[self.end - 1, 10] - \
                                         self.pathfile_data[self.start, 10]
        planningdata.total_path_time = self.pathfile_data[self.end - 1, 7] - \
                                       self.pathfile_data[self.start, 7]
        planningdata.gear = int(self.pathfile_data[self.start, 9])

        planningdata.header.timestamp_sec = rospy.get_time()
        self.planning_pub.publish(planningdata.SerializeToString())
        self.sequence_num += 1
        print "Generated Planning Sequence: " + str(self.sequence_num)

    def quit(self, signum, frame):
        """
        shutdown the keypress thread
        """
        print "Shutting Down..."
        self.terminating = True
        self.file.close()
        rospy.sleep(0.2)
        sys.exit(0)


def main():
    """
    Main rosnode
    """
    parser = argparse.ArgumentParser(description='Generate Planning Trajectory from Data File')
    parser.add_argument('-m', '--mode', help='publish data by time(t) or data(d)', default='t')
    parser.add_argument('-l', '--light', help='publish data including light info', default='f')
    args = vars(parser.parse_args())

    player = RtkPlayer(args['light'])
    rospy.init_node('rtk_player', anonymous=True)
    rospy.Subscriber('/pnc/carstatus', String, player.callback_carstatus)
    rospy.Subscriber('/pnc/control/PadMessage', String, player.callback_padmessage)
    rate = rospy.Rate(10)

    signal.signal(signal.SIGINT, player.quit)
    signal.signal(signal.SIGTERM, player.quit)

    # if publish planning data by time
    if args['mode'] == 't':
        while not rospy.is_shutdown():
            player.localization()
            player.publish_planningmsg()
            rate.sleep()
    # if publish planning data by position data
    elif args['mode'] == 'd':
        while not rospy.is_shutdown():
            player.check_position()
            rate.sleep()
    else:
        print "Input error, choose to publish planning by time(-m t) or data(-m d)"
        sys.exit(0)


if __name__ == '__main__':
    main()
