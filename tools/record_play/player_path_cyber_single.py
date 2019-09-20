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

import time
import cybertron
import sys
import thread
import signal
from out import global_adc_status_pb2
from out import planning_pb2
from out import pad_msg_pb2
from numpy import genfromtxt
import argparse


class RtkPlayer(object):
    """
    rtk player class
    """
    def __init__(self, gpstime, padmsg):
        # check input type
        if gpstime > 0:
            self.gpstime_trigger = True
            self.padmsg_trigger = False
        elif padmsg == 't':
            self.gpstime_trigger = False
            self.padmsg_trigger = True
        else:
            print "Input error, choose to publish planning by gps(-g [time]) or padmsg(-p t)"
            sys.exit(0)

        # load data
        try:
            file_path = open('filename_path', 'r').readline().split('\n')[0]
            self.file = open(file_path, 'r')
        except:
            print "Cannot find file"
            sys.exit(0)
        self.pathfile_data = genfromtxt(self.file, delimiter=',', skip_footer=1, skip_header=1)
        self.data_length = len(self.pathfile_data)

        # initialize
        self.sequence_num = 0
        self.curr_index = 0
        self.gpstime = gpstime
        self.gpstime_reaches = False
        self.padmsg_received = False
        self.carstatus_received = False
        self.terminating = False
        self.estop = False
        self.planning_writer = node.create_writer('/pnc/planning', planning_pb2.ADCTrajectory)
        thread.start_new_thread(self.keypress, (1,))
        thread.start_new_thread(self.carstatus_thread, (1,))
        thread.start_new_thread(self.padterminal_thread, (1,))
        time.sleep(1.0)
        self.fill_data()
        print "Planning Ready"

    def fill_data(self):
        """
        fill in planning data
        """
        if not self.carstatus_received:
            print "No car status, unable to generate planning when filling data"
            sys.exit(0)

        self.start = 0
        self.end = self.data_length
        self.curr_index = self.search_closest_point(self.carx, self.cary, 0, self.data_length)

        self.planningdata = planning_pb2.ADCTrajectory()
        self.planningdata.header.module_name = "planning"
        self.planningdata.header.sequence_num = self.sequence_num
        self.planningdata.error_code = 1

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
            self.planningdata.adc_trajectory_point.extend([adc_point])

        if self.estop:
            self.planningdata.estop.is_estop = True
        else:
            self.planningdata.estop.is_estop = False

        self.planningdata.total_path_length = self.pathfile_data[self.end - 1, 10] - \
                                              self.pathfile_data[self.start, 10]
        self.planningdata.total_path_time = self.pathfile_data[self.end - 1, 7] - \
                                            self.pathfile_data[self.start, 7]
        self.planningdata.gear = int(self.pathfile_data[self.start, 9])

    def search_closest_point(self, x, y, start_idx, end_idx):
        """
        get the shortest length, return the index
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
        New Pad Terminal Received
        """
        if data.action == pad_msg_pb2.START:
            self.padmsg_received = True
            print "current padmessage: " + str(self.padmsg_received)

    def callback_carstatus(self, data):
        """
        New Car Status Received
        """
        self.carstatus_received = True
        self.carx = data.pose.position.x
        self.cary = data.pose.position.y
        self.carz = data.pose.position.z
        # GPS_time = time.localtime(data.header.timestamp_sec)
        # self.timestamp = GPS_time.tm_hour * 10000 + GPS_time.tm_min * 100 + GPS_time.tm_sec
        self.timestamp = data.header.timestamp_sec

    def check_status(self):
        """
        check whether to publish planning data
        """
        if not self.carstatus_received:
            print "No car status, unable to generate planning"
            return

        if self.gpstime_trigger:
            if abs(self.timestamp - self.gpstime) < 0.1 and not self.gpstime_reaches:
                print "gps time reaches"
                self.gpstime_reaches = True
                self.publish_planning()
        elif self.padmsg_trigger:
            if self.padmsg_received:
                print "get pad message and enter automode"
                self.padmsg_received = False
                self.publish_planning()

    def publish_planning(self):
        """
        publish planning data
        """
        self.planningdata.header.timestamp_sec = time.time()
        self.planning_writer.write(self.planningdata)
        self.sequence_num += 1
        print "Generated Planning Sequence: " + str(self.sequence_num)

    def carstatus_thread(self, arg):
        """
        carstatus reader thread
        """
        node.create_reader("/pnc/carstatus", global_adc_status_pb2.Status, self.callback_carstatus)
        while 1:
            node.spin_once()

    def padterminal_thread(self, arg):
        """
        pad terminal reader thread
        """
        node.create_reader("/pnc/control/PadMessage", pad_msg_pb2.PadMessage,
                            self.callback_padmessage)
        while 1:
            node.spin_once()

    def quit(self, signum, frame):
        """
        shutdown the keypress thread
        """
        print "Shutting Down..."
        self.terminating = True
        self.file.close()
        time.sleep(0.2)
        sys.exit(0)


def main():
    """
    Main cybernode
    """
    parser = argparse.ArgumentParser(description='Generate Planning Trajectory from Data File')
    parser.add_argument('-g', '--gpstime', help='publish when gps time reaches',
                                           type=float, default='0')
    parser.add_argument('-p', '--padmsg', help='publish when enter automode (t/f)', default='f')
    args = vars(parser.parse_args())

    global node
    cybertron.init.log_init()
    node = cybertron.node.Node("rtk_player")
    player = RtkPlayer(args['gpstime'], args['padmsg'])

    signal.signal(signal.SIGINT, player.quit)
    signal.signal(signal.SIGTERM, player.quit)

    while 1:
        player.check_status()
        time.sleep(0.1)


if __name__ == '__main__':
    main()
