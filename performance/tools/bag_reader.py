import os
import sys

import cybertron
import protopy
import google
print(google.__file__)
from protopy import traffic_light_detection_pb2
from protopy import perception_obstacle_pb2
from protopy import global_adc_status_pb2
from protopy import planning_pb2
from protopy import prediction_obstacle_pb2

files = os.popen("ls last_diff/*.record").read().split('\n')
for fname in [x.strip() for x in files if x.strip()]:
    reader = cybertron.bag.Bag(fname)
    print(fname)
    print(reader.start_timestamp)
    print(reader.end_timestamp)
    reader.close()
#fname = "last_diff/5b17846985d299004465b76f.record"
#fname = "5b1db6d035e21f0044e9aaaf.bag"
fname = "5b30a8dd7a998e0044655169.bag"
fname = "5b30a5807a998e004465448c.bag"
fname = "1.record"
if len(sys.argv) > 1:
    fname = sys.argv[1]
reader = cybertron.bag.Bag(fname)

channels = ["/perception/traffic_light_status", "/pnc/carstatus", "/perception/obstacles", "/pnc/prediction", "/pnc/planning"]

traffic = protopy.traffic_light_detection_pb2.TrafficLightDetection()
obstacle = perception_obstacle_pb2.PerceptionObstacles()
status = global_adc_status_pb2.Status()
prediction = prediction_obstacle_pb2.PredictionObstacles()
planning = planning_pb2.ADCTrajectory()
prev_obstacle = 0
prev_status = 0
count_obstacle = 0
count_status = 0
count_traffic = 0
f1 = open("status.txt", 'w')
f2 = open("obstacle.txt", 'w')
f3 = open("traffic.txt", 'w')
f4 = open("prediction.txt", 'w')
f5 = open("planning.txt", 'w')
for message in reader.read_messages(channels):
    if message.topic == "/pnc/carstatus":
        count_status += 1
        status.ParseFromString(message.message)
        stamp = status.header.timestamp_sec
        diff = stamp - prev_status
        seq = status.header.sequence_num
        if diff > 0.1:
            print("status", seq, prev_status, stamp)
            i = 0
        prev_status = stamp
    elif message.topic == "/perception/obstacles":
        count_obstacle += 1
        obstacle.ParseFromString(message.message)
        stamp = obstacle.header.timestamp_sec
        diff = stamp - prev_status
        if diff > 0.2:
            print("obstacle", obstacle.header.sequence_num, prev_obstacle, stamp)
        prev_obstacle = stamp
    elif message.topic == "/perception/traffic_light_status":
        count_traffic += 1
        traffic.ParseFromString(message.message)
        f3.write("%s" % traffic) 
    elif message.topic == "/planning_adapter/prediction":
        print("0000000000")
        prediction.ParseFromString(message.message)
        #print(prediction)
        print("1111111111111")
    elif message.topic == "/pnc/planning":
        planning.ParseFromString(message.message) 
        f5.write("%s" % planning)
f1.close()
f2.close()
f3.close()
