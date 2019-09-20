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
from numpy import genfromtxt
import argparse


class RtkPlayer(object):
    """
    rtk player class
    """
    def __init__(self, speedmultiplier, completepath):
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
        self.planning_writer = node.create_writer('/pnc/planning', planning_pb2.ADCTrajectory)
        self.speedmultiplier = speedmultiplier / 100
        self.sequence_num = 0
        self.curr_index = 0
        self.planning_length = 1000
        self.carstatus_received = False
        self.terminating = False
        self.estop = False
        self.completepath = completepath == 't'
        thread.start_new_thread(self.keypress, (1,))
        thread.start_new_thread(self.carstatus_thread, (1,))
        print "Planning Ready"

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

    def callback_carstatus(self, data):
        """
        New Car Status Received
        """
        self.carx = data.pose.position.x
        self.cary = data.pose.position.y
        self.carz = data.pose.position.z
        self.carstatus_received = True

    def write_planningmsg(self):
        """
        Generate New Path
        """
        if not self.carstatus_received:
            print "No Car Status, Unable to generate planning"
            return

        # localize, determine start and end point of planning data
        self.curr_index = self.search_closest_point(self.carx, self.cary, 
                                                    self.curr_index, self.data_length)
        self.start = max(self.curr_index - 10, 0)
        self.end = min(self.start + self.planning_length, self.data_length)
        if self.completepath:
            self.start = 0
            self.end = self.data_length

        planningdata = planning_pb2.ADCTrajectory()
        planningdata.header.module_name = "planning"
        planningdata.header.sequence_num = self.sequence_num
        planningdata.error_code = 1

        for i in range(self.start, self.end):
            adc_point = planning_pb2.ADCTrajectoryPoint()
            adc_point.x = self.pathfile_data[i, 0]
            adc_point.y = self.pathfile_data[i, 1]
            adc_point.z = self.pathfile_data[i, 2]
            adc_point.speed = self.pathfile_data[i, 3] * self.speedmultiplier
            adc_point.acceleration_s = self.pathfile_data[i, 4] * self.speedmultiplier
            adc_point.curvature = self.pathfile_data[i, 5]
            adc_point.curvature_change_rate = self.pathfile_data[i, 6]
            time_begin2cur = self.pathfile_data[i, 7] - self.pathfile_data[self.start, 7]
            time_begin2closest = self.pathfile_data[self.curr_index, 7] - \
                                 self.pathfile_data[self.start, 7]
            adc_point.relative_time = (time_begin2cur - time_begin2closest) / self.speedmultiplier
            adc_point.theta = self.pathfile_data[i, 8]
            adc_point.s = self.pathfile_data[i, 10] - self.pathfile_data[self.start, 10]
            adc_point.accumulated_s = adc_point.s
            planningdata.adc_trajectory_point.extend([adc_point])

        if self.estop:
            planningdata.estop.is_estop = True
        else:
            planningdata.estop.is_estop = False

        planningdata.total_path_length = self.pathfile_data[self.end - 1, 10] - \
                                         self.pathfile_data[self.start, 10]
        planningdata.total_path_time = self.pathfile_data[self.end - 1, 7] - \
                                       self.pathfile_data[self.start, 7]
        planningdata.gear = int(self.pathfile_data[self.start, 9])

        planningdata.header.timestamp_sec = time.time()
        self.planning_writer.write(planningdata)
        self.sequence_num += 1
        print "Generated Planning Sequence: " + str(self.sequence_num)

    def carstatus_thread(self, arg):
        """
        carstatus thread
        """
        node.create_reader("/pnc/carstatus", global_adc_status_pb2.Status, self.callback_carstatus)
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
    parser.add_argument('-s', '--speedmulti', help='Speed multiplier in percentage (Default 100)',
                                              type=float, default='100')
    parser.add_argument('-c', '--complete', help='Generate complete path (t/f)', default='f')
    args = vars(parser.parse_args())

    global node
    cybertron.init.log_init()
    node = cybertron.node.Node("rtk_player")
    player = RtkPlayer(args['speedmulti'], args['complete'])

    signal.signal(signal.SIGINT, player.quit)
    signal.signal(signal.SIGTERM, player.quit)

    while 1:
        time.sleep(0.1)  # 10Hz
        player.write_planningmsg()


if __name__ == '__main__':
    main()
