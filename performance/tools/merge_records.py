#!/usr/bin/env python
#coding=utf-8
"""
@file: merge_record.py
@version: 0.1
@brief: merge several bags into one rosbag
"""

import bag as rosbag
import os
import glob
import argparse
import logging
logging.basicConfig(level=logging.INFO)


def sort_bag2(rosbag_files):
    """
    sort rosbags by date
    """
    rosbag_sorted = sorted(rosbag_files,
        key=lambda x:int(os.path.splitext(os.path.basename(x))[0].split('_')[-1]))
    return rosbag_sorted


def merge(bagfile_in, bagfile_out, start=None, end=None, topic_set=None):
    """
    merge rosbags into one
    """
    print(topic_set)
    bag_out = rosbag.Bag(bagfile_out, True)
    if bag_out is not None:
        for bag in bagfile_in:
            bag_in = rosbag.Bag(bag)
            for topic, msg, msgtype, t in bag_in.read_messages([tp for tp in topic_set]):
                if start is not None and t / 1e9 < start:
                    continue
                if end is not None and t / 1e9 > end:
                    continue
                bag_out.write(topic, msg, msgtype, raw=True, t=t)
            bag_in.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description =\
            "merge several rosbags into one rosbag")
    parser.add_argument('rosbags', type = str,
            help = 'arg: rosbag directory contains rosbag')
    parser.add_argument('--merged_dir', type=str, default=None,
                help = 'arg: rosbag directory contains merged rosbag')
    parser.add_argument('--topics', type=str, default=None,
                help = 'arg: topics to store')
    parser.add_argument('--task_id', type=str, default=None,
                help = 'arg: task id')
    parser.add_argument('--start', type=int, default=None,
                help = 'arg: start seconds')
    parser.add_argument('--end', type=int, default=None,
                help = 'arg: end seconds')
    args = parser.parse_args()

    if args.merged_dir is None:
        rosbag_path = os.path.abspath(args.rosbags)
        rosbag_path += '/rosbag_merged'
    else:
        rosbag_path = args.merged_dir

    rosbag_files = glob.glob(args.rosbags + '/*.record')
    rosbag_files += glob.glob(args.rosbags + '/records/*.record')
    if len(rosbag_files) < 1:
        logging.warn("None rosbag file exists in path: {}".format(rosbag_path))
    else:
        logging.info("rosbag path: {}".format(rosbag_path))
        rosbag_sorted = sort_bag2(rosbag_files)
        if os.path.exists(rosbag_path) == False:
            os.system("mkdir -p %s" % rosbag_path)
        bag_out = rosbag_path + '/rosbag_merged_{}_{}_{}.record'.format(args.task_id,
            args.start, args.end)

        topic_set = None
        if args.topics is not None and len(args.topics) > 0:
            topic_set = set()
            topic_set.update(args.topics.split(','))
        else:
            topic_set = set(['/pnc/carstatus',
                '/perception/obstacles',
                '/perception/traffic_light_status'])
        print(topic_set)
        merge(rosbag_sorted, bag_out, start=args.start, end=args.end, topic_set=topic_set)

