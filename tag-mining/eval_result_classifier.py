# !/usr/bin/env python
# -*- coding: utf-8 -*-
###############################################################################
#
# Copyright (c) 2019 Baidu.com, Inc. All Rights Reserved
#
###############################################################################
"""
Authors: liyixiang(liyixiang@baidu.com)
Date:    2019/01/18
"""

import os
import sys
import json
import shutil
import numpy as np

class EvalRresultClassifier(object):
    """
    class to calculate localization precision of each kind of label
    """

    def __init__(self):
        self.tags_seg_dict = {"acc": "car_speed_seg_list", 
                            "traj": "car_trajectory_seg_list", 
                            "grad": "gradient_seg_list",
                            "gnss": "gnss_seg_list"}
        self.tags_catog_dict = {"acc": "car_speed", 
                                "traj": "car_trajectory", 
                                "grad": "gradient",
                                "gnss": "gnss"}

    def load_eval_result(self, eval_result_path):
        """
        load eval result
        """
        try:
            file_list = os.listdir(eval_result_path)
            file_path = os.path.join(eval_result_path, file_list[0])
            self.eval_result = np.loadtxt(file_path)
            del file_list[0]
            for eval_f in file_list:
                file_path = os.path.join(eval_result_path, eval_f)
                temp_data = np.loadtxt(file_path)
                self.eval_result = np.vstack((self.eval_result, temp_data))
        except IOError as e:
            print "load eval result failed:%s" % e

    def _get_tag_file_start_time(self, file_name):
        return file_name.split('_')[0]

    def load_single_task_tag(self, task_tag_path):
        """
        load single task tag
        """
        task_tag_dict = {}
        try:
            file_list = os.listdir(task_tag_path)
            for file_name in file_list:
                file_path = os.path.join(task_tag_path, file_name)
                time_stamp = self._get_tag_file_start_time(file_name)
                tag_f = open(file_path, 'r')
                task_tag_dict[time_stamp] = json.load(tag_f)["tags"]
                tag_f.close()
        except Exception as e:
            print "load single task tag failed:%s" % e
        return task_tag_dict

    def _get_tag_file_taskid(self, task_dir_name):
        return '_'.join(task_dir_name.split('_')[0:1])

    def load_tasks_tag(self, tasks_tag_path):
        """
        load tasks tag
        """
        dir_list = os.listdir(tasks_tag_path)
        self.tasks_tag_dict = {}
        for task_id in dir_list:
            task_tag_file_path = os.path.join(tasks_tag_path, task_id)
            self.tasks_tag_dict[task_id] = self.load_single_task_tag(
                                                    task_tag_file_path)
        # tasks_tag_f = open(os.path.join(tasks_tag_path, "tasks_tag_dict.json"), 'w')
        # json.dump(self.tasks_tag_dict, tasks_tag_f)
        # tasks_tag_f.close()

    def _get_single_task_tag_category_set(self, tag_categ, task_tag_dict):
        tag_list = []
        for ts, ts_tags in task_tag_dict.items():
            tags = ts_tags[self.tags_catog_dict[tag_categ]]
            if tags is None:
                continue
            else:
                tag_list.extend(tags.split(', '))
        return set(tag_list)

    def _get_tasks_tag_category_set(self, tag_categ, tasks_tag_dict):
        tag_set = set()
        for task_tags in tasks_tag_dict.values():
            task_tag_set = self._get_single_task_tag_category_set(tag_categ,
                                                                 task_tags)
            if task_tag_set is not None:
                tag_set |= task_tag_set
        return tag_set

    def _create_tag_eval_result_file(self, tag_eval_result_path, tag_set):
        tag_list = list(tag_set)
        tag_list_f = {}
        for tag in tag_list:
            subpath = os.path.join(tag_eval_result_path, tag)
            if os.path.exists(subpath):
                shutil.rmtree(subpath)
            os.mkdir(subpath)
            tag_eval_result_file = os.path.join(subpath, 
                                                "%s_eval_result.txt" % tag)
            tag_list_f[tag] = open(tag_eval_result_file, 'w')
        return tag_list_f

    def _close_tag_eval_result_file(self, tag_list_f):
        for f in tag_list_f.values():
            f.close()

    def _search_tag_in_seg_list(self, seg_list, idx):
        for tag_list in seg_list:
            if idx >= tag_list[0] and idx < tag_list[1]:
                return tag_list[2]
        return None

    def _search_tag_in_tasks(self, tag_categ, time_stamp):
        for task_tags in self.tasks_tag_dict.values():
            for ts, tags in task_tags.items():
                if time_stamp >= float(ts) and time_stamp < float(ts) + 60:
                    idx = int((time_stamp - float(ts))/0.25)
                    seg_list = tags[self.tags_seg_dict[tag_categ]]
                    if seg_list is None:
                        return None
                    else:
                        return self._search_tag_in_seg_list(seg_list, idx)
        return None            

    def gen_tag_eval_result(self, tag_categ, tag_eval_result_path):
        """
        calculate localization precision for each kind of tag
        """
        if not os.path.exists(tag_eval_result_path):
            os.mkdir(tag_eval_result_path)
        tag_set = self._get_tasks_tag_category_set(tag_categ, 
                                        self.tasks_tag_dict)
        tag_list_f = self._create_tag_eval_result_file(tag_eval_result_path, 
                                                        tag_set)
        for i in range(self.eval_result.shape[0]):
            unix_time = self.eval_result[i][0] + 315964782
            tag = self._search_tag_in_tasks(tag_categ, unix_time)
            if tag is not None:
                eval_str = np.array2string(self.eval_result[i], 
                                            max_line_width = 100,
                                            formatter={'float_kind':
                                            lambda x: "%.6f" % x})
                eval_str = eval_str.lstrip('[').rstrip(']') + '\n'
                tag_list_f[tag].write(eval_str)
        self._close_tag_eval_result_file(tag_list_f)

if __name__ == '__main__':
    eval_result_path = sys.argv[1]
    tasks_tag_path = sys.argv[2]
    tag_categ = sys.argv[3]
    tag_eval_result_path = sys.argv[4]
    erc = EvalRresultClassifier()
    erc.load_eval_result(eval_result_path)
    erc.load_tasks_tag(tasks_tag_path)
    erc.gen_tag_eval_result(tag_categ, tag_eval_result_path)
